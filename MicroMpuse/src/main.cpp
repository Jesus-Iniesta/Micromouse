#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SparkFun_TB6612.h>
#include <TCA9548.h>
#include <ESP32Encoder.h>

// ====== CONFIGURACIÓN DE PINES ======
// Motor A (Izquierdo) - TB6612FNG
#define AIN1 26
#define AIN2 27
#define PWMA 18

// Motor B (Derecho) - TB6612FNG
#define BIN1 25
#define BIN2 14
#define PWMB 19

// Pin STBY del TB6612FNG
#define STBY 4

// Encoders
#define ENC_L_A 32  // Encoder izquierdo (Motor A) fase A
#define ENC_L_B 33  // Encoder izquierdo (Motor A) fase B
#define ENC_R_A 34  // Encoder derecho (Motor B) fase A
#define ENC_R_B 35  // Encoder derecho (Motor B) fase B

// I2C para sensores
#define SDA_PIN 21
#define SCL_PIN 22

// ====== CONSTANTES ======
// Dimensiones del laberinto y robot
#define CELL_SIZE 170        // Tamaño de celda del laberinto (mm)
#define ROBOT_WIDTH 70       // Ancho del robot (mm)
#define ROBOT_LENGTH 100     // Largo del robot (mm)

// Distancias objetivo para navegación (mm)
#define WALL_FRONT_MIN 80    // Distancia mínima al frente antes de girar
#define WALL_SIDE_TARGET 50  // Distancia objetivo a pared lateral (centrado)
#define WALL_SIDE_MIN 35     // Distancia mínima a pared lateral
#define WALL_SIDE_MAX 65     // Distancia máxima a pared lateral

// Control de motores
#define OFFSETA 1  // Offset motor A (ajustar según calibración)
#define OFFSETB 1  // Offset motor B (ajustar según calibración)
#define BASE_SPEED 120       // Velocidad base (0-255)
#define TURN_SPEED 150       // Velocidad de giro
#define CORRECTION_FACTOR 30 // Factor de corrección de trayectoria

// Direcciones del multiplexor TCA9548A
#define TCA_ADDRESS 0x70

// Canales del multiplexor para cada sensor
#define SENSOR_FRONT 3   // Sensor frontal en canal 3 (SC3, SD3)
#define SENSOR_LEFT 1    // Sensor izquierdo en canal 1 (SC1, SD1)
#define SENSOR_RIGHT 2   // Sensor derecho en canal 2 (SC2, SD2)

// ====== OBJETOS GLOBALES ======
// Motores
Motor motorLeft = Motor(AIN1, AIN2, PWMA, OFFSETA, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, OFFSETB, STBY);

// Multiplexor I2C
TCA9548 multiplexor(TCA_ADDRESS);

// Sensores VL53L0X
VL53L0X sensorFront;
VL53L0X sensorLeft;
VL53L0X sensorRight;

// Encoders
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// Variables para lecturas de sensores (en mm)
int distanceFront = 0;
int distanceLeft = 0;
int distanceRight = 0;

// Variables para encoders
long encoderLeftCount = 0;
long encoderRightCount = 0;

// ====== FUNCIONES DE MULTIPLEXOR ======
void selectMuxChannel(uint8_t channel) {
    if (channel > 7) return;
    multiplexor.selectChannel(channel);
    delay(5); // Pequeña pausa para estabilizar
}

// ====== FUNCIONES DE SENSORES ======
void initSensors() {
    Serial.println("Inicializando sensores VL53L0X...");
    
    // Inicializar sensor frontal
    selectMuxChannel(SENSOR_FRONT);
    if (sensorFront.init()) {
        sensorFront.setTimeout(500);
        sensorFront.startContinuous();
        Serial.println("Sensor frontal OK");
    } else {
        Serial.println("Error: Sensor frontal");
    }
    
    // Inicializar sensor izquierdo
    selectMuxChannel(SENSOR_LEFT);
    if (sensorLeft.init()) {
        sensorLeft.setTimeout(500);
        sensorLeft.startContinuous();
        Serial.println("Sensor izquierdo OK");
    } else {
        Serial.println("Error: Sensor izquierdo");
    }
    
    // Inicializar sensor derecho
    selectMuxChannel(SENSOR_RIGHT);
    if (sensorRight.init()) {
        sensorRight.setTimeout(500);
        sensorRight.startContinuous();
        Serial.println("Sensor derecho OK");
    } else {
        Serial.println("Error: Sensor derecho");
    }
}

void readSensors() {
    // Leer sensor frontal
    selectMuxChannel(SENSOR_FRONT);
    distanceFront = sensorFront.readRangeContinuousMillimeters();
    if (sensorFront.timeoutOccurred()) {
        distanceFront = 8190; // Valor máximo si hay timeout
    }
    
    // Leer sensor izquierdo
    selectMuxChannel(SENSOR_LEFT);
    distanceLeft = sensorLeft.readRangeContinuousMillimeters();
    if (sensorLeft.timeoutOccurred()) {
        distanceLeft = 8190;
    }
    
    // Leer sensor derecho
    selectMuxChannel(SENSOR_RIGHT);
    distanceRight = sensorRight.readRangeContinuousMillimeters();
    if (sensorRight.timeoutOccurred()) {
        distanceRight = 8190;
    }
}

// ====== FUNCIONES DE MOTORES ======
void stopMotors() {
    motorLeft.brake();
    motorRight.brake();
}

void moveForward(int speed) {
    motorLeft.drive(speed);
    motorRight.drive(speed);
}

void moveBackward(int speed) {
    motorLeft.drive(-speed);
    motorRight.drive(-speed);
}

void turnLeft(int speed) {
    motorLeft.drive(-speed);
    motorRight.drive(speed);
}

void turnRight(int speed) {
    motorLeft.drive(speed);
    motorRight.drive(-speed);
}

// ====== FUNCIONES DE ENCODERS ======
void initEncoders() {
    Serial.println("Inicializando encoders...");
    
    // Configurar encoder izquierdo
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderLeft.attachHalfQuad(ENC_L_A, ENC_L_B);
    encoderLeft.setCount(0);
    
    // Configurar encoder derecho
    encoderRight.attachHalfQuad(ENC_R_A, ENC_R_B);
    encoderRight.setCount(0);
    
    Serial.println("Encoders inicializados");
}

void readEncoders() {
    encoderLeftCount = encoderLeft.getCount();
    encoderRightCount = encoderRight.getCount();
}

void resetEncoders() {
    encoderLeft.setCount(0);
    encoderRight.setCount(0);
    encoderLeftCount = 0;
    encoderRightCount = 0;
}

// ====== FUNCIONES DE DIAGNÓSTICO ======
void printSensorData() {
    Serial.print("Front: ");
    Serial.print(distanceFront);
    Serial.print(" mm | Left: ");
    Serial.print(distanceLeft);
    Serial.print(" mm | Right: ");
    Serial.print(distanceRight);
    Serial.println(" mm");
}

void printEncoderData() {
    Serial.print("Enc Left: ");
    Serial.print(encoderLeftCount);
    Serial.print(" | Enc Right: ");
    Serial.println(encoderRightCount);
}

// ====== SETUP ======
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Robot Laberinto - Inicializando...");
    
    // Inicializar I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // 400kHz I2C
    Serial.println("I2C inicializado");
    
    // Inicializar multiplexor
    multiplexor.begin();
    Serial.println("Multiplexor TCA9548A inicializado");
    
    // Inicializar sensores
    initSensors();
    
    // Inicializar encoders
    initEncoders();
    
    // Pequeña pausa antes de comenzar
    delay(1000);
    stopMotors();
    
    Serial.println("¡Sistema listo!");
    Serial.println("Esperando 3 segundos antes de comenzar...");
    delay(3000);
}

// ====== LOOP PRINCIPAL ======
void loop() {
    // Leer sensores
    readSensors();
    readEncoders();
    
    // Mostrar información (comentar para mejorar performance)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        printSensorData();
        printEncoderData();
        lastPrint = millis();
    }
    
    // ====== LÓGICA DE NAVEGACIÓN OPTIMIZADA ======
    
    // Detectar paredes
    bool wallFront = (distanceFront < WALL_FRONT_MIN);
    bool wallLeft = (distanceLeft < WALL_FRONT_MIN);
    bool wallRight = (distanceRight < WALL_FRONT_MIN);
    
    if (wallFront) {
        // Pared al frente - decidir dirección de giro
        stopMotors();
        delay(100);
        
        // Prioridad: derecha (algoritmo mano derecha)
        if (!wallRight) {
            // Girar derecha 90°
            turnRight(TURN_SPEED);
            delay(350); // Calibrar este valor para giro exacto de 90°
        } else if (!wallLeft) {
            // Girar izquierda 90°
            turnLeft(TURN_SPEED);
            delay(350); // Calibrar este valor para giro exacto de 90°
        } else {
            // Callejón sin salida - girar 180°
            turnRight(TURN_SPEED);
            delay(700); // Calibrar para 180°
        }
        
        stopMotors();
        delay(100);
        resetEncoders(); // Reiniciar contadores después del giro
        
    } else {
        // No hay pared al frente - avanzar con corrección
        
        int speedLeft = BASE_SPEED;
        int speedRight = BASE_SPEED;
        
        // Corrección proporcional basada en pared derecha (algoritmo mano derecha)
        if (distanceRight < 200) { // Hay pared derecha detectada
            int error = distanceRight - WALL_SIDE_TARGET;
            int correction = constrain(error * 0.8, -CORRECTION_FACTOR, CORRECTION_FACTOR);
            
            // Aplicar corrección
            speedLeft = BASE_SPEED + correction;
            speedRight = BASE_SPEED - correction;
            
        } else if (distanceLeft < 200) { // No hay pared derecha, usar izquierda
            int error = WALL_SIDE_TARGET - distanceLeft;
            int correction = constrain(error * 0.8, -CORRECTION_FACTOR, CORRECTION_FACTOR);
            
            // Aplicar corrección
            speedLeft = BASE_SPEED + correction;
            speedRight = BASE_SPEED - correction;
        }
        
        // Limitar velocidades
        speedLeft = constrain(speedLeft, 50, 200);
        speedRight = constrain(speedRight, 50, 200);
        
        // Aplicar velocidades
        motorLeft.drive(speedLeft);
        motorRight.drive(speedRight);
    }
    
    delay(50);
}
