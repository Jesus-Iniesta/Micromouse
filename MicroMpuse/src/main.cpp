#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>  // <--- LIBRERÍA POLOLU
#include <ESP32Encoder.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <vector>
#include <algorithm> // Para el filtro de mediana

// ====== 1. CONFIGURACIÓN DE RED ======
#define STASSID "robotica"      
#define STAPSK  "robotica2025"  
#define UDP_PORT 12345

// ====== 2. HARDWARE Y SENSORES ======
#define TCA_ADDR 0x70
#define CH_FRONT 3   
#define CH_LEFT  7   
#define CH_RIGHT 2   

// Objetos Pololu
VL53L0X sensorFront;
VL53L0X sensorLeft;
VL53L0X sensorRight;

// Variables globales
int distFront = 0, distLeft = 0, distRight = 0;
bool isBusy = false; 

// Ajustes de Navegación
int WALL_TARGET = 45;       
const int WALL_THRESHOLD = 150; 

// PID
const float Kp_wall = 1.5;    
const float Kp_enc = 0.5;     
const int MAX_CORR = 60;
const int BASE_SPEED = 120;
const int MIN_PWM = 60;
const int DEADZONE = 5; // Zona muerta para evitar micro-ajustes

// Variables de estado para suavizado
float lastCorrection = 0.0;
int consecutiveWallLeft = 0;
int consecutiveWallRight = 0;
int consecutiveNoWalls = 0;

// Geometría y Odometría
const float WHEEL_DIAMETER = 43.5; 
const float WHEEL_TRACK = 90.0; 
const float TICKS_PER_REV = 1750.0; 
const float CELL_SIZE = 160.0;  

const float TICKS_PER_MM = 1.0 / ((PI * WHEEL_DIAMETER) / TICKS_PER_REV);
const float TICKS_CELL = CELL_SIZE * TICKS_PER_MM;
const float TICKS_TURN_90 = (PI * WHEEL_TRACK * 0.25) * TICKS_PER_MM; 
const float TICKS_TURN_180 = TICKS_TURN_90 * 2.0;

// Pines
#define ENC_L_A 26
#define ENC_L_B 25
#define ENC_R_A 27
#define ENC_R_B 14

const int AIN1 = 32; const int AIN2 = 33 ; const int PWMA = 18;
const int BIN1 = 23; const int BIN2 = 5; const int PWMB = 19;
const int STBY = 16;

class SimpleMotor {
  int pin1, pin2, pinPWM;
  public:
    SimpleMotor(int p1, int p2, int pwm) {
      pin1 = p1; pin2 = p2; pinPWM = pwm;
      pinMode(pin1, OUTPUT); pinMode(pin2, OUTPUT); pinMode(pinPWM, OUTPUT);
    }
    void drive(int speed) {
      if (abs(speed) < MIN_PWM && speed != 0) speed = (speed > 0) ? MIN_PWM : -MIN_PWM;
      speed = constrain(speed, -255, 255);
      if (speed > 0) { digitalWrite(pin1, HIGH); digitalWrite(pin2, LOW); }
      else if (speed < 0) { digitalWrite(pin1, LOW); digitalWrite(pin2, HIGH); }
      else { digitalWrite(pin1, LOW); digitalWrite(pin2, LOW); }
      analogWrite(pinPWM, abs(speed));
    }
    void brake() { digitalWrite(pin1, HIGH); digitalWrite(pin2, HIGH); analogWrite(pinPWM, 0); }
};

SimpleMotor motorLeft(AIN1, AIN2, PWMA); 
SimpleMotor motorRight(BIN1, BIN2, PWMB);
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;
WiFiUDP udp;

IPAddress remoteIP;
unsigned int remotePort;

// ====== FUNCIONES DE HARDWARE (POLOLU + FILTRO) ======

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void initSensors() {
  Wire.begin(21, 22);
  
  // Configuración Pololu: Alta precisión (High Accuracy)
  // Aumentamos el "budget" de tiempo a 33ms para lecturas más estables
  
  tcaSelect(CH_FRONT); 
  sensorFront.init();
  sensorFront.setTimeout(500);
  sensorFront.setMeasurementTimingBudget(33000); // 33ms por lectura (más preciso)
  sensorFront.startContinuous();

  tcaSelect(CH_LEFT);  
  sensorLeft.init();
  sensorLeft.setTimeout(500);
  sensorLeft.setMeasurementTimingBudget(33000);
  sensorLeft.startContinuous();

  tcaSelect(CH_RIGHT); 
  sensorRight.init();
  sensorRight.setTimeout(500);
  sensorRight.setMeasurementTimingBudget(33000);
  sensorRight.startContinuous();
}

// --- FILTRO DE MEDIANA (Mejora drástica de precisión) ---
// Toma 5 lecturas rápidas, las ordena y devuelve la del medio.
// Esto elimina el ruido eléctrico y rebotes.
int getFilteredDistance(VL53L0X &sensor, int channel) {
  tcaSelect(channel);
  std::vector<int> readings;
  
  // Tomar 5 muestras (ajustable, 3 es más rápido, 5 es más preciso)
  for(int i=0; i<3; i++) {
    int val = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()) val = 8190;
    readings.push_back(val);
  }
  
  // Ordenar de menor a mayor
  std::sort(readings.begin(), readings.end());
  
  // Devolver el valor central (Mediana)
  return readings[readings.size() / 2];
}

void readAllSensors() {
  // Usamos la función con filtro
  distFront = getFilteredDistance(sensorFront, CH_FRONT);
  distLeft  = getFilteredDistance(sensorLeft, CH_LEFT);
  distRight = getFilteredDistance(sensorRight, CH_RIGHT);
}

void sendMsg(String msg) {
  if (remoteIP) {
    udp.beginPacket(remoteIP, remotePort);
    udp.print(msg);
    udp.endPacket();
  }
}

void checkUDPWhileMoving() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    String cmd = String(packetBuffer);
    cmd.trim();
    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();

    if (cmd == "STATUS") {
      // Leemos sensores (usando una sola muestra para rapidez en movimiento)
      tcaSelect(CH_FRONT); int f = sensorFront.readRangeContinuousMillimeters();
      tcaSelect(CH_LEFT);  int l = sensorLeft.readRangeContinuousMillimeters();
      tcaSelect(CH_RIGHT); int r = sensorRight.readRangeContinuousMillimeters();
      String resp = "BUSY:" + String(f) + "," + String(l) + "," + String(r);
      sendMsg(resp);
    } else if (cmd == "STOP") {
      motorLeft.brake(); motorRight.brake();
      isBusy = false;
      sendMsg("STOPPED");
      ESP.restart();
    }
  }
}

// ====== MOVIMIENTOS ======

void moveForward() {
  encoderLeft.setCount(0); encoderRight.setCount(0);
  long targetTicks = TICKS_CELL;
  isBusy = true;
  
  // Reiniciar contadores de estado
  consecutiveWallLeft = 0;
  consecutiveWallRight = 0;
  consecutiveNoWalls = 0;
  lastCorrection = 0.0;
  
  while(isBusy) {
    checkUDPWhileMoving();
    if (!isBusy) break; 

    long countsL = abs(encoderLeft.getCount());
    long countsR = abs(encoderRight.getCount());
    long currentDist = (countsL + countsR) / 2;
    
    // Leemos sensores (Nota: Durante el PID usamos lectura rápida directa para no frenar el loop)
    tcaSelect(CH_FRONT); int f = sensorFront.readRangeContinuousMillimeters();
    tcaSelect(CH_LEFT);  int l = sensorLeft.readRangeContinuousMillimeters();
    tcaSelect(CH_RIGHT); int r = sensorRight.readRangeContinuousMillimeters();

    if(f < 60 && f > 0) break; // Pared enfrente
    if(currentDist >= targetTicks) break;

    // Detección de paredes con histéresis (evita cambios bruscos)
    bool wallLeft = (l < WALL_THRESHOLD);
    bool wallRight = (r < WALL_THRESHOLD);
    
    // Actualizar contadores de estado consecutivo
    if (wallLeft) consecutiveWallLeft++; else consecutiveWallLeft = 0;
    if (wallRight) consecutiveWallRight++; else consecutiveWallRight = 0;
    if (!wallLeft && !wallRight) consecutiveNoWalls++; else consecutiveNoWalls = 0;
    
    // Solo considerar paredes estables (al menos 3 lecturas consecutivas)
    bool stableWallLeft = (consecutiveWallLeft >= 3);
    bool stableWallRight = (consecutiveWallRight >= 3);
    
    float correction = 0;
    float alpha = 0.7; // Factor de suavizado (0.7 = 70% nuevo, 30% anterior)

    // Estrategia de control mejorada
    if (stableWallLeft && stableWallRight) {
      // CASO 1: Ambas paredes detectadas - Centrar entre ellas
      int diff = l - r;
      if (abs(diff) > DEADZONE) {
        correction = diff * Kp_wall;
      } else {
        correction = 0; // Dentro de zona muerta, mantener curso
      }
    } else if (stableWallLeft && !stableWallRight) {
      // CASO 2: Solo pared izquierda - Mantener distancia constante
      int error = l - WALL_TARGET;
      if (abs(error) > DEADZONE) {
        correction = error * Kp_wall;
      } else {
        correction = 0;
      }
    } else if (!stableWallLeft && stableWallRight) {
      // CASO 3: Solo pared derecha - Mantener distancia constante
      int error = r - WALL_TARGET;
      if (abs(error) > DEADZONE) {
        correction = -error * Kp_wall;
      } else {
        correction = 0;
      }
    } else if (consecutiveNoWalls >= 5) {
      // CASO 4: Sin paredes estables - Usar encoders para mantener línea recta
      // Solo después de 5 lecturas consecutivas sin paredes
      long encDiff = countsL - countsR;
      if (abs(encDiff) > 10) {
        correction = -encDiff * Kp_enc;
      } else {
        correction = 0;
      }
    } else {
      // CASO 5: Transición - Mantener la última corrección suavizada
      correction = lastCorrection * 0.5; // Reducir corrección gradualmente
    }

    // Suavizado de la corrección (evita cambios bruscos)
    correction = alpha * correction + (1.0 - alpha) * lastCorrection;
    correction = constrain(correction, -MAX_CORR, MAX_CORR);
    lastCorrection = correction;
    
    int speedL = BASE_SPEED - correction;
    int speedR = BASE_SPEED + correction;
    
    // Desaceleración suave al final
    if ((targetTicks - currentDist) < 300) {
       speedL = map(targetTicks - currentDist, 0, 300, MIN_PWM, speedL);
       speedR = map(targetTicks - currentDist, 0, 300, MIN_PWM, speedR);
    }
    
    motorLeft.drive(speedL);
    motorRight.drive(speedR);
  }
  
  motorLeft.brake(); motorRight.brake();
  isBusy = false;
  delay(100);
}

void turn(int angleDeg) {
  encoderLeft.setCount(0); encoderRight.setCount(0);
  long target = (abs(angleDeg) == 180) ? TICKS_TURN_180 : TICKS_TURN_90;
  bool left = (angleDeg < 0);
  isBusy = true;
  int speed = 100;
  
  while(isBusy) {
    checkUDPWhileMoving();
    if (!isBusy) break;
    long dist = (abs(encoderLeft.getCount()) + abs(encoderRight.getCount())) / 2;
    if(dist >= target) break;
    if(left) { motorLeft.drive(-speed); motorRight.drive(speed); }
    else { motorLeft.drive(speed); motorRight.drive(-speed); }
  }
  motorLeft.brake(); motorRight.brake();
  isBusy = false;
  delay(100);
}

void setup() {
  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderLeft.attachHalfQuad(ENC_L_A, ENC_L_B);
  encoderRight.attachHalfQuad(ENC_R_A, ENC_R_B);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  
  initSensors();

  Serial.print("WiFi...");
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println(WiFi.localIP());
  
  udp.begin(UDP_PORT);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    String cmd = String(packetBuffer);
    cmd.trim();
    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();

    if (cmd == "SENSORS" || cmd == "STATUS") {
      readAllSensors(); // Aquí sí usamos el filtro de mediana
      String resp = "IDLE:" + String(distFront) + "," + String(distLeft) + "," + String(distRight);
      sendMsg(resp);
    } else if (cmd == "FORWARD") {
      sendMsg("ACK"); moveForward(); sendMsg("DONE");
    } else if (cmd == "TURNL") {
      sendMsg("ACK"); turn(-90); sendMsg("DONE");
    } else if (cmd == "TURNR") {
      sendMsg("ACK"); turn(90); sendMsg("DONE");
    } else if (cmd == "TURNU") {
      sendMsg("ACK"); turn(180); sendMsg("DONE");
    } else if (cmd == "STOP") {
      motorLeft.brake(); motorRight.brake(); sendMsg("STOPPED");
    }
  }
}