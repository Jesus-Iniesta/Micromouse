#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SparkFun_TB6612.h>
#include <TCA9548.h>
#include <ESP32Encoder.h>
#include <WiFi.h>
#include <WiFiUdp.h>

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
#define ENC_L_A 32
#define ENC_L_B 33
#define ENC_R_A 34
#define ENC_R_B 35

// I2C para sensores
#define SDA_PIN 21
#define SCL_PIN 22

// ====== CONSTANTES WiFi ======
const char* WIFI_SSID = "kali";      // <<<< CAMBIAR
const char* WIFI_PASSWORD = "ben1010xzc";   // <<<< CAMBIAR
const int UDP_PORT = 12345;

// ====== CONSTANTES ======
#define CELL_SIZE 160        // Tamaño de celda: 16cm × 16cm
#define MAZE_COLS 12         // 12 columnas
#define MAZE_ROWS 7          // 7 filas
#define ROBOT_WIDTH 70       // Ancho del robot: 7cm
#define ROBOT_LENGTH 100     // Largo del robot: 10cm
#define WALL_FRONT_MIN 80
#define WALL_SIDE_TARGET 50
#define OFFSETA 1
#define OFFSETB 1
#define BASE_SPEED 120
#define TURN_SPEED 150
#define TCA_ADDRESS 0x70
#define SENSOR_FRONT 3
#define SENSOR_LEFT 1
#define SENSOR_RIGHT 2

// ====== OBJETOS GLOBALES ======
Motor motorLeft = Motor(AIN1, AIN2, PWMA, OFFSETA, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, OFFSETB, STBY);
TCA9548 multiplexor(TCA_ADDRESS);
VL53L0X sensorFront, sensorLeft, sensorRight;
ESP32Encoder encoderLeft, encoderRight;
WiFiUDP udp;

// ====== VARIABLES GLOBALES ======
int distanceFront = 0, distanceLeft = 0, distanceRight = 0;
long encoderLeftCount = 0, encoderRightCount = 0;
String currentCommand = "";
bool commandReady = false;
bool executing = false;
IPAddress clientIP;
unsigned int clientPort = 0;

// Mutex para sincronización
SemaphoreHandle_t xMutex;

// ====== PROTOCOLO DE COMUNICACIÓN ======
// Comandos recibidos:
// FORWARD - Avanzar una celda
// TURNL - Girar izquierda 90°
// TURNR - Girar derecha 90°
// TURNU - Girar 180°
// STOP - Detener motores
// STATUS - Obtener estado (sensores + encoders)
// SENSORS - Obtener solo sensores

// Respuestas enviadas:
// OK - Comando completado
// BUSY - Robot ocupado ejecutando comando
// STATUS:F,L,R,EL,ER - Estado completo
// SENSORS:F,L,R - Solo sensores
// READY - Robot listo

// ====== FUNCIONES DE MULTIPLEXOR ======
void selectMuxChannel(uint8_t channel) {
    if (channel > 7) return;
    multiplexor.selectChannel(channel);
    delay(5);
}

// ====== FUNCIONES DE SENSORES ======
void initSensors() {
    Serial.println("Inicializando sensores...");
    selectMuxChannel(SENSOR_FRONT);
    if (sensorFront.init()) {
        sensorFront.setTimeout(500);
        sensorFront.startContinuous();
        Serial.println("Sensor frontal OK");
    }
    selectMuxChannel(SENSOR_LEFT);
    if (sensorLeft.init()) {
        sensorLeft.setTimeout(500);
        sensorLeft.startContinuous();
        Serial.println("Sensor izquierdo OK");
    }
    selectMuxChannel(SENSOR_RIGHT);
    if (sensorRight.init()) {
        sensorRight.setTimeout(500);
        sensorRight.startContinuous();
        Serial.println("Sensor derecho OK");
    }
}

void readSensors() {
    selectMuxChannel(SENSOR_FRONT);
    distanceFront = sensorFront.readRangeContinuousMillimeters();
    if (sensorFront.timeoutOccurred()) distanceFront = 8190;
    
    selectMuxChannel(SENSOR_LEFT);
    distanceLeft = sensorLeft.readRangeContinuousMillimeters();
    if (sensorLeft.timeoutOccurred()) distanceLeft = 8190;
    
    selectMuxChannel(SENSOR_RIGHT);
    distanceRight = sensorRight.readRangeContinuousMillimeters();
    if (sensorRight.timeoutOccurred()) distanceRight = 8190;
}

// ====== FUNCIONES DE ENCODERS ======
void initEncoders() {
    Serial.println("Inicializando encoders...");
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderLeft.attachHalfQuad(ENC_L_A, ENC_L_B);
    encoderRight.attachHalfQuad(ENC_R_A, ENC_R_B);
    encoderLeft.setCount(0);
    encoderRight.setCount(0);
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

// ====== FUNCIONES DE MOTORES (PRIMITIVAS) ======
void stopMotors() {
    motorLeft.brake();
    motorRight.brake();
}

void moveForward(int speed) {
    motorLeft.drive(speed);
    motorRight.drive(speed);
}

void turnLeft(int speed) {
    motorLeft.drive(-speed);
    motorRight.drive(speed);
}

void turnRight(int speed) {
    motorLeft.drive(speed);
    motorRight.drive(-speed);
}

// ====== PRIMITIVAS DE MOVIMIENTO ======
void primitiveForward() {
    Serial.println("Ejecutando: FORWARD");
    resetEncoders();
    
    // Avanzar aproximadamente una celda (ajustar según calibración)
    // Usar encoders para medir distancia
    long targetCounts = 1000; // Calibrar este valor
    
    while (abs(encoderLeftCount) < targetCounts) {
        readEncoders();
        readSensors();
        
        // Corrección básica mientras avanza
        int speedL = BASE_SPEED;
        int speedR = BASE_SPEED;
        
        if (distanceRight < 200) {
            int error = distanceRight - WALL_SIDE_TARGET;
            int corr = constrain(error, -20, 20);
            speedL += corr;
            speedR -= corr;
        }
        
        moveForward((speedL + speedR) / 2);
        delay(10);
    }
    
    stopMotors();
    delay(100);
}

void primitiveTurnLeft() {
    Serial.println("Ejecutando: TURN LEFT");
    stopMotors();
    delay(100);
    
    turnLeft(TURN_SPEED);
    delay(350); // Calibrar para 90° exactos
    
    stopMotors();
    delay(100);
    resetEncoders();
}

void primitiveTurnRight() {
    Serial.println("Ejecutando: TURN RIGHT");
    stopMotors();
    delay(100);
    
    turnRight(TURN_SPEED);
    delay(350); // Calibrar para 90° exactos
    
    stopMotors();
    delay(100);
    resetEncoders();
}

void primitiveTurnAround() {
    Serial.println("Ejecutando: TURN AROUND (180°)");
    stopMotors();
    delay(100);
    
    turnRight(TURN_SPEED);
    delay(700); // Calibrar para 180° exactos
    
    stopMotors();
    delay(100);
    resetEncoders();
}

// ====== FUNCIONES WiFi ======
void initWiFi() {
    Serial.print("Conectando a WiFi: ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n¡WiFi conectado!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        
        udp.begin(UDP_PORT);
        Serial.print("UDP escuchando en puerto: ");
        Serial.println(UDP_PORT);
    } else {
        Serial.println("\nError: No se pudo conectar a WiFi");
    }
}

void sendResponse(String response) {
    if (clientPort > 0) {
        udp.beginPacket(clientIP, clientPort);
        udp.print(response);
        udp.endPacket();
        Serial.print("Enviado: ");
        Serial.println(response);
    }
}

// ====== TAREA DE COMUNICACIÓN WiFi (FreeRTOS) ======
void taskCommunication(void *pvParameters) {
    Serial.println("Tarea de comunicación iniciada");
    
    while (true) {
        int packetSize = udp.parsePacket();
        
        if (packetSize) {
            clientIP = udp.remoteIP();
            clientPort = udp.remotePort();
            
            char incomingPacket[255];
            int len = udp.read(incomingPacket, 255);
            if (len > 0) {
                incomingPacket[len] = 0;
            }
            
            String command = String(incomingPacket);
            command.trim();
            
            Serial.print("Recibido: ");
            Serial.println(command);
            
            // Procesar comandos
            if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                if (command == "STATUS") {
                    readSensors();
                    readEncoders();
                    String status = "STATUS:" + 
                                  String(distanceFront) + "," +
                                  String(distanceLeft) + "," +
                                  String(distanceRight) + "," +
                                  String(encoderLeftCount) + "," +
                                  String(encoderRightCount);
                    sendResponse(status);
                    
                } else if (command == "SENSORS") {
                    readSensors();
                    String sensors = "SENSORS:" + 
                                   String(distanceFront) + "," +
                                   String(distanceLeft) + "," +
                                   String(distanceRight);
                    sendResponse(sensors);
                    
                } else if (executing) {
                    sendResponse("BUSY");
                    
                } else if (command == "FORWARD" || command == "TURNL" || 
                          command == "TURNR" || command == "TURNU" || 
                          command == "STOP") {
                    currentCommand = command;
                    commandReady = true;
                    sendResponse("ACK");
                }
                
                xSemaphoreGive(xMutex);
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ====== TAREA DE EJECUCIÓN DE MOVIMIENTOS (FreeRTOS) ======
void taskExecution(void *pvParameters) {
    Serial.println("Tarea de ejecución iniciada");
    
    while (true) {
        if (commandReady) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                executing = true;
                String cmd = currentCommand;
                commandReady = false;
                xSemaphoreGive(xMutex);
                
                // Ejecutar comando
                if (cmd == "FORWARD") {
                    primitiveForward();
                } else if (cmd == "TURNL") {
                    primitiveTurnLeft();
                } else if (cmd == "TURNR") {
                    primitiveTurnRight();
                } else if (cmd == "TURNU") {
                    primitiveTurnAround();
                } else if (cmd == "STOP") {
                    stopMotors();
                }
                
                // Marcar como completado
                if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                    executing = false;
                    xSemaphoreGive(xMutex);
                }
                
                sendResponse("OK");
            }
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ====== SETUP ======
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=================================");
    Serial.println("Robot Micromouse - Control WiFi");
    Serial.println("=================================\n");
    
    // Crear mutex
    xMutex = xSemaphoreCreateMutex();
    
    // Inicializar I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Serial.println("I2C inicializado");
    
    // Inicializar multiplexor
    multiplexor.begin();
    Serial.println("Multiplexor inicializado");
    
    // Inicializar sensores
    initSensors();
    
    // Inicializar encoders
    initEncoders();
    
    // Inicializar WiFi
    initWiFi();
    
    // Detener motores
    stopMotors();
    
    Serial.println("\n¡Sistema listo!");
    
    // Enviar mensaje de READY por broadcast
    delay(2000);
    sendResponse("READY");
    
    // Crear tareas FreeRTOS
    xTaskCreatePinnedToCore(
        taskCommunication,
        "Communication",
        4096,
        NULL,
        2,
        NULL,
        0
    );
    
    xTaskCreatePinnedToCore(
        taskExecution,
        "Execution",
        4096,
        NULL,
        1,
        NULL,
        1
    );
    
    Serial.println("Tareas FreeRTOS creadas");
}

// ====== LOOP PRINCIPAL ======
void loop() {
    // El loop principal está libre
    // Las tareas FreeRTOS manejan todo
    delay(1000);
    
    // Mostrar info de debug
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("WiFi OK | IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi desconectado - Reintentando...");
        initWiFi();
    }
}
