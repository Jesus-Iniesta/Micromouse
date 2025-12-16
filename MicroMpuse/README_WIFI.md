# 🤖 Robot Micromouse - Control WiFi con Flood Fill

Sistema completo de control para robot micromouse con comunicación WiFi, algoritmo Flood Fill y interfaz gráfica intuitiva.

## 📋 Características

### ESP32 (Robot)
- ✅ Comunicación WiFi mediante UDP
- ✅ Tareas FreeRTOS para ejecución no bloqueante
- ✅ Control de motores TB6612FNG
- ✅ Lectura de 3 sensores VL53L0X mediante multiplexor TCA9548A
- ✅ Encoders para medición precisa
- ✅ Primitivas de movimiento (Forward, TurnLeft, TurnRight, TurnAround)

### Interfaz Gráfica (PC)
- ✅ Visualización en tiempo real del laberinto 16×16
- ✅ Algoritmo Flood Fill para navegación óptima
- ✅ Mapa actualizado dinámicamente según detección de paredes
- ✅ Control remoto del robot
- ✅ Monitoreo de sensores en tiempo real
- ✅ Log de actividades
- ✅ Diseño moderno con tema oscuro

## 🔧 Configuración

### 1. ESP32

#### Modificar credenciales WiFi en `src/main.cpp`:
```cpp
const char* WIFI_SSID = "TU_RED_WIFI";      // <<<< CAMBIAR
const char* WIFI_PASSWORD = "TU_PASSWORD";   // <<<< CAMBIAR
```

#### Compilar y subir:
```bash
platformio run --target upload
```

#### Obtener la IP del ESP32:
Abre el monitor serial y anota la IP que muestra:
```bash
platformio device monitor
```

Verás algo como:
```
¡WiFi conectado!
IP: 192.168.1.XXX
```

### 2. Interfaz Gráfica

#### Modificar IP del robot en `comunicacion/interfaz_grafica.py`:
```python
ROBOT_IP = "192.168.1.XXX"  # <<<< CAMBIAR a la IP del ESP32
```

#### Instalar dependencias:
```bash
pip install tkinter  # Viene con Python por defecto en la mayoría de sistemas
```

#### Ejecutar:
```bash
cd comunicacion
python interfaz_grafica.py
```

## 📡 Protocolo de Comunicación

### Comandos (PC → Robot)
- `FORWARD` - Avanzar una celda
- `TURNL` - Girar izquierda 90°
- `TURNR` - Girar derecha 90°
- `TURNU` - Girar 180°
- `STOP` - Detener motores
- `STATUS` - Obtener estado completo
- `SENSORS` - Obtener solo lecturas de sensores

### Respuestas (Robot → PC)
- `ACK` - Comando recibido, iniciando ejecución
- `OK` - Comando completado
- `BUSY` - Robot ocupado ejecutando comando
- `STATUS:F,L,R,EL,ER` - Estado: Front, Left, Right, EncoderLeft, EncoderRight
- `SENSORS:F,L,R` - Sensores: Front, Left, Right (en mm)
- `READY` - Robot listo

## 🎮 Uso

1. **Conectar el ESP32** a la alimentación
2. **Verificar conexión WiFi** en el monitor serial
3. **Ejecutar la interfaz gráfica** en la PC
4. **Hacer clic en "Conectar"** en la interfaz
5. **Hacer clic en "Iniciar Exploración"**
6. El robot explorará el laberinto automáticamente usando Flood Fill

### Controles de la Interfaz
- **▶️ Iniciar Exploración** - Comienza la navegación automática
- **⏸️ Pausar** - Pausa/reanuda la exploración
- **⏹️ Detener** - Detiene completamente el robot
- **🔄 Reiniciar Mapa** - Limpia el mapa y reinicia la posición

## 🏗️ Arquitectura

### ESP32 - Tareas FreeRTOS

```
┌─────────────────────────────┐
│  Tarea Comunicación (Core 0)│
│  - Recibir comandos UDP     │
│  - Enviar respuestas        │
│  - Actualizar estado        │
└─────────────────────────────┘
           ↕ Mutex
┌─────────────────────────────┐
│  Tarea Ejecución (Core 1)   │
│  - Ejecutar primitivas      │
│  - Leer sensores            │
│  - Control de motores       │
└─────────────────────────────┘
```

### Interfaz - Arquitectura

```
┌────────────────────────┐
│   Interfaz Tkinter     │
│   (Thread Principal)   │
└────────┬───────────────┘
         │
         ↓
┌────────────────────────┐
│  Thread Exploración    │
│  - Flood Fill          │
│  - Control del robot   │
│  - Actualización UI    │
└────────┬───────────────┘
         │
         ↓
┌────────────────────────┐
│  Robot Controller      │
│  - Socket UDP          │
│  - Comandos/Respuestas │
└────────────────────────┘
```

## 🔍 Algoritmo Flood Fill

El algoritmo implementado:
1. **Inicializa** distancias desde la meta (centro del laberinto)
2. **Explora** detectando paredes con sensores
3. **Actualiza** el mapa en tiempo real
4. **Recalcula** distancias cuando detecta nuevas paredes
5. **Navega** siempre hacia la celda vecina con menor distancia

## ⚙️ Calibración

### Valores a ajustar en `src/main.cpp`:

```cpp
// Tiempos de giro (dependen de tu robot)
delay(350);  // Para giro de 90° - CALIBRAR
delay(700);  // Para giro de 180° - CALIBRAR

// Distancia de avance
long targetCounts = 1000;  // Pulsos del encoder para una celda - CALIBRAR

// Velocidades
#define BASE_SPEED 120     // Velocidad de avance - AJUSTAR
#define TURN_SPEED 150     // Velocidad de giro - AJUSTAR
```

### Valores a ajustar en `interfaz_grafica.py`:

```python
# Umbral de detección de pared
wall_threshold = 100  # mm - AJUSTAR según sensores
```

## 🐛 Solución de Problemas

### Robot no se conecta a WiFi
- Verificar SSID y contraseña
- Asegurar que la PC y el ESP32 están en la misma red
- Verificar que el router permite comunicación entre dispositivos

### No se puede conectar desde la interfaz
- Verificar que la IP del robot sea correcta
- Verificar firewall de la PC
- Probar hacer ping al ESP32: `ping 192.168.1.XXX`

### Robot gira incorrectamente
- Ajustar los valores de `delay()` en las funciones de giro
- Verificar que los motores están conectados correctamente

### Sensores no detectan paredes
- Verificar conexiones del multiplexor
- Ajustar `wall_threshold` en la interfaz
- Verificar canales del multiplexor

## 📁 Estructura del Proyecto

```
MicroMpuse/
├── src/
│   ├── main.cpp              # Código ESP32 con WiFi
│   └── main_backup.cpp       # Backup del código original
├── comunicacion/
│   ├── interfaz_grafica.py   # Interfaz gráfica principal
│   └── interfaz_grafica_backup.py  # Backup
├── platformio.ini            # Configuración PlatformIO
└── README_WIFI.md           # Este archivo
```

## 🎨 Capturas de Pantalla

La interfaz muestra:
- 🗺️ Mapa del laberinto en tiempo real
- 🤖 Posición y orientación del robot
- 📊 Distancias Flood Fill en cada celda
- 📡 Lecturas de sensores
- 📝 Log de actividades
- 🎮 Controles de navegación

## 📝 Notas

- El sistema usa **UDP** para comunicación de baja latencia
- Las tareas FreeRTOS permiten **ejecución no bloqueante**
- El **Flood Fill** se recalcula automáticamente al detectar paredes
- La interfaz actualiza el mapa en **tiempo real**
- El robot mantiene su posición mediante **encoders**

## 🚀 Mejoras Futuras

- [ ] Modo de carrera rápida (fast run)
- [ ] Guardar/cargar mapas
- [ ] Estadísticas de exploración
- [ ] Control manual con teclado
- [ ] Múltiples estrategias de navegación
- [ ] Visualización 3D del laberinto

## 📄 Licencia

Proyecto educativo para competencia de Micromouse

---

**¡Buena suerte en la competencia! 🏆**
