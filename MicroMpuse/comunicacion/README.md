# Sistema de Control WiFi - Micromouse

Este módulo contiene la interfaz gráfica para control del robot Micromouse con algoritmo Flood Fill mediante comunicación WiFi.

## Descripción del Sistema

El robot es controlado completamente desde la computadora mediante **WiFi (UDP)**. El ESP32 ejecuta primitivas de movimiento y lee los sensores, mientras que la computadora:
- Calcula el siguiente movimiento usando Flood Fill
- Procesa los datos de los sensores
- Construye el mapa del laberinto
- Muestra una interfaz gráfica en tiempo real

### Especificaciones del Laberinto
- Matriz: **16 × 16 celdas**
- Tamaño de celda: **170 mm × 170 mm**
- Robot: **70mm ancho × 100mm largo**
- Inicio: Esquina inferior izquierda (0, 0)
- Objetivo: Centro del laberinto (7, 7)

## Estructura de Archivos

```
comunicacion/
├── interfaz_grafica.py      # Interfaz gráfica con Tkinter + Flood Fill + WiFi
├── requirements.txt         # Dependencias Python
└── README.md               # Este archivo
```

## Instalación

### 1. Instalar Python 3.8 o superior

```bash
python3 --version
```

### 2. Crear entorno virtual (recomendado)

```bash
cd comunicacion
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# o
venv\Scripts\activate  # Windows
```

## Instalación

### 1. Instalar Python 3.8 o superior

```bash
python3 --version
```

### 2. Instalar dependencias

```bash
pip install tkinter  # Viene incluido con Python en la mayoría de sistemas
```

## Uso

### 1. Configurar WiFi en el ESP32

Edita `src/main.cpp` y configura tu red WiFi:

```cpp
const char* WIFI_SSID = "TU_RED_WIFI";
const char* WIFI_PASSWORD = "TU_PASSWORD";
```

### 2. Compilar y subir código al ESP32

```bash
platformio run --target upload
```

### 3. Obtener la IP del ESP32

Abre el monitor serial:

```bash
platformio device monitor
```

Verás algo como:
```
¡WiFi conectado!
IP: 192.168.1.XXX
```

### 4. Configurar la IP en la interfaz

Edita `interfaz_grafica.py` en la línea 15:

```python
ROBOT_IP = "192.168.1.XXX"  # IP del ESP32
```

### 5. Ejecutar la interfaz gráfica

```bash
python interfaz_grafica.py
```

### 6. Usar la interfaz

1. **Conectar** - Establece conexión WiFi con el robot
2. **Iniciar Exploración** - El robot explora automáticamente con Flood Fill
3. **Pausar** - Pausa/reanuda la exploración
4. **Detener** - Detiene el robot
5. **Reiniciar Mapa** - Limpia el mapa y reinicia posición

## Protocolo de Comunicación WiFi (UDP)

### Comandos enviados al ESP32

| Comando | Descripción |
|---------|-------------|
| `FORWARD` | Avanzar una celda (170 mm) |
| `TURNL` | Girar 90° a la izquierda |
| `TURNR` | Girar 90° a la derecha |
| `TURNU` | Girar 180° |
| `STOP` | Detener motores |
| `STATUS` | Obtener estado completo |
| `SENSORS` | Obtener lecturas de sensores |

### Respuestas del ESP32

| Respuesta | Descripción |
|-----------|-------------|
| `ACK` | Comando recibido, iniciando ejecución |
| `OK` | Comando completado exitosamente |
| `BUSY` | Robot ocupado ejecutando comando |
| `STATUS:F,L,R,EL,ER` | Estado: Front, Left, Right, EncoderLeft, EncoderRight |
| `SENSORS:F,L,R` | Sensores en mm: Front, Left, Right |
| `READY` | Robot listo |
| `B` | Retroceder una celda |
| `L` | Girar 90° a la izquierda |
| `R` | Girar 90° a la derecha |


## Interfaz Gráfica

La interfaz muestra:
- **Matriz 16×16** del laberinto
- **Paredes detectadas** (líneas blancas)
- **Robot** (círculo rojo con flecha de dirección)
- **Celdas visitadas** (gris oscuro)
- **Objetivo** (verde - centro 7,7)
- **Distancias Flood Fill** (números azules en cada celda)
- **Información en tiempo real** (posición, dirección, sensores)
- **Log de actividades**

### Colores

- 🔴 **Rojo**: Robot
- 🟢 **Verde**: Objetivo (centro)
- 🔵 **Números azules**: Distancias Flood Fill
- ⬛ **Gris oscuro**: Celdas visitadas
- ⬛ **Gris muy oscuro**: Celdas no visitadas
- ⬜ **Blanco**: Paredes detectadas

## Algoritmo Flood Fill

El algoritmo implementado:
1. Inicializa distancias desde la meta usando BFS (Breadth-First Search)
2. El robot explora detectando paredes con sensores
3. Actualiza el mapa en tiempo real
4. Recalcula distancias cuando detecta nuevas paredes
5. Siempre se mueve a la celda vecina con menor distancia
6. Considera paredes para determinar movimientos válidos

## Arquitectura del Sistema

### ESP32 - FreeRTOS
```
┌─────────────────────────────┐
│  Tarea Comunicación (Core 0)│
│  - Recibir comandos UDP     │
│  - Enviar respuestas        │
│  - No bloquea ejecución     │
└─────────────────────────────┘
           ↕ Mutex
┌─────────────────────────────┐
│  Tarea Ejecución (Core 1)   │
│  - Ejecutar primitivas      │
│  - Leer sensores            │
│  - Control de motores       │
└─────────────────────────────┘
```

### Interfaz Python
```
┌────────────────────────┐
│   Interfaz Tkinter     │
│   (Thread Principal)   │
└────────┬───────────────┘
         │
         ↓
┌────────────────────────┐
│  Thread Exploración    │
│  - Algoritmo Flood Fill│
│  - Control del robot   │
│  - Actualización UI    │
└────────┬───────────────┘
         │
         ↓
┌────────────────────────┐
│  Robot Controller      │
│  - Socket UDP          │
│  - Protocolo WiFi      │
└────────────────────────┘
```

## Calibración

### Valores a ajustar en el ESP32 (`src/main.cpp`):

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

### Valores a ajustar en la interfaz (`interfaz_grafica.py`):

```python
# Umbral de detección de pared
wall_threshold = 100  # mm - AJUSTAR según sensores

# IP y puerto
ROBOT_IP = "192.168.1.XXX"  # IP del ESP32
ROBOT_PORT = 12345
```

## Solución de Problemas

### El robot no se conecta a WiFi

- Verificar SSID y contraseña en `src/main.cpp`
- Asegurar que la PC y el ESP32 están en la misma red
- Verificar que el router permite comunicación entre dispositivos
- Reiniciar el ESP32

### No se puede conectar desde la interfaz

- Verificar que la IP del robot sea correcta
- Verificar firewall de la PC
- Probar hacer ping al ESP32: `ping 192.168.1.XXX`
- Verificar que el puerto UDP 12345 no esté bloqueado

### El robot gira incorrectamente

- Ajustar los valores de `delay()` en las funciones de giro
- Verificar que los motores estén conectados correctamente
- Calibrar velocidad de giro `TURN_SPEED`

### Los sensores no detectan paredes

- Verificar conexiones del multiplexor TCA9548A
- Ajustar `wall_threshold` en la interfaz
- Verificar canales del multiplexor (3=frontal, 1=izquierdo, 2=derecho)
- Verificar alimentación de los sensores

### La interfaz no se abre

- Verificar que tkinter esté instalado (viene con Python)
- En Linux: `sudo apt-get install python3-tk`
- Verificar versión de Python >= 3.8

## Personalización

### Cambiar dimensiones del laberinto

En `interfaz_grafica.py`:
```python
MAZE_SIZE = 16  # Cambiar tamaño
TARGET_X, TARGET_Y = 7, 7  # Cambiar posición de meta
```

### Ajustar umbral de detección de paredes

En `interfaz_grafica.py` en la clase `FloodFill`:
```python
wall_threshold = 100  # mm - ajustar según calibración
```

### Cambiar tamaño de celdas en interfaz

En `interfaz_grafica.py`:
```python
CELL_SIZE = 40  # píxeles - ajustar para zoom
```

## Características Principales

### 🎮 Interfaz Intuitiva
- Tema oscuro moderno
- Visualización en tiempo real
- Controles accesibles
- Log detallado de eventos

### 🤖 Control Inteligente
- Algoritmo Flood Fill optimizado
- Ejecución no bloqueante con FreeRTOS
- Detección automática de paredes
- Navegación autónoma

### 📡 Comunicación WiFi
- Protocolo UDP de baja latencia
- Sin cables ni conexiones físicas
- Respuestas en tiempo real
- Sistema de reconexión automática

## Referencias

- [Flood Fill Algorithm - Micromouse Online](https://www.micromouseonline.com/2017/02/04/maze-solving-flooding-algorithm/)
- [Micromouse Design](https://www.micromouseonline.com/)
- [FreeRTOS Documentation](https://www.freertos.org/documentation/)

## Licencia

Proyecto educativo para competencia de Micromouse - 2025

---

**¡Buena suerte en la competencia! 🏆**
