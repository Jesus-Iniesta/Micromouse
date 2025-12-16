# Sistema de Comunicación y Control - Micromouse

Este módulo contiene el sistema de control desde computadora para el robot Micromouse con algoritmo Flood Fill.

## Descripción del Sistema

El robot es controlado completamente desde la computadora. El ESP32 solo ejecuta primitivas de movimiento y lee los sensores, mientras que la computadora:
- Calcula el siguiente movimiento usando Flood Fill
- Procesa los datos de los sensores
- Construye el mapa del laberinto
- Muestra una interfaz gráfica en tiempo real

### Especificaciones del Laberinto
- Matriz: **12 columnas × 7 filas**
- Tamaño de celda: **160 mm × 160 mm**
- Inicio: Esquina inferior izquierda (6, 0)
- Objetivo: Centro del laberinto (4 celdas centrales)

## Estructura de Archivos

```
comunicacion/
├── protocolo_serial.py      # Comunicación serial ESP32-PC
├── flood_fill.py            # Algoritmo Flood Fill
├── interfaz_grafica.py      # Interfaz gráfica con Pygame
├── main_control.py          # Programa principal
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

### 3. Instalar dependencias

```bash
pip install -r requirements.txt
```

## Uso

### 1. Conectar el ESP32

Conecta el ESP32 a la computadora vía USB. Identifica el puerto:

**Linux/Mac:**
```bash
ls /dev/tty* | grep -E "(USB|ACM)"
# Generalmente: /dev/ttyUSB0 o /dev/ttyACM0
```

**Windows:**
- Abrir "Administrador de dispositivos"
- Buscar en "Puertos (COM y LPT)"
- Anotar el puerto (ej: COM3, COM4)

### 2. Configurar el puerto

Edita `main_control.py` y ajusta el puerto:

```python
puerto = '/dev/ttyUSB0'  # Linux/Mac
# o
puerto = 'COM3'  # Windows
```

### 3. Ejecutar el programa

```bash
python main_control.py
```

### 4. Opciones del menú

1. **Explorar laberinto (automático)**: El robot explora usando Flood Fill
2. **Modo manual**: Control manual para pruebas (F=avanzar, L=izquierda, R=derecha)
3. **Leer sensores**: Lee y muestra datos de sensores
4. **Calibrar robot**: Calibra la posición del robot
5. **Salir**: Cierra el programa

## Protocolo de Comunicación

### Comandos enviados al ESP32

| Comando | Descripción |
|---------|-------------|
| `F` | Avanzar una celda (160 mm) |
| `B` | Retroceder una celda |
| `L` | Girar 90° a la izquierda |
| `R` | Girar 90° a la derecha |
| `S` | Leer sensores |
| `X` | Detener motores |
| `C` | Calibrar posición |

### Respuestas del ESP32

| Respuesta | Descripción |
|-----------|-------------|
| `OK` | Comando ejecutado correctamente |
| `ERROR` | Error al ejecutar comando |
| `SENS:F=123,L=456,R=789` | Lecturas de sensores en mm |

## Interfaz Gráfica

La interfaz muestra:
- **Matriz 12×7** del laberinto
- **Paredes detectadas** (líneas negras)
- **Robot** (círculo rojo con flecha de dirección)
- **Celdas visitadas** (verde claro)
- **Objetivo** (dorado)
- **Distancias Flood Fill** (números en cada celda)
- **Información de estado** (posición, dirección, fase)

### Colores

- 🟥 **Rojo**: Robot
- 🟨 **Dorado**: Objetivo
- 🟩 **Verde claro**: Celdas visitadas
- ⬜ **Gris claro**: Celdas no visitadas
- ⬛ **Negro**: Paredes

## Algoritmo Flood Fill

El algoritmo:
1. Calcula distancias desde cada celda al objetivo
2. El robot siempre se mueve a la celda vecina con menor distancia
3. Actualiza el mapa cuando detecta paredes
4. Recalcula distancias después de cada actualización

## Modificación del Código del ESP32

El ESP32 debe implementar el protocolo de comandos. Agrega este código al inicio del `loop()`:

```cpp
void loop() {
    // Leer comando serial
    if (Serial.available() > 0) {
        char comando = Serial.read();
        
        switch(comando) {
            case 'F':
                avanzar_una_celda();
                Serial.println("OK");
                break;
            case 'L':
                girar_izquierda_90();
                Serial.println("OK");
                break;
            case 'R':
                girar_derecha_90();
                Serial.println("OK");
                break;
            case 'S':
                leer_y_enviar_sensores();
                break;
            case 'X':
                detener_motores();
                Serial.println("OK");
                break;
            case 'C':
                calibrar_posicion();
                Serial.println("OK");
                break;
        }
    }
}

void leer_y_enviar_sensores() {
    // Leer sensores VL53L0X
    int frontal = leerSensorFrontal();
    int izquierdo = leerSensorIzquierdo();
    int derecho = leerSensorDerecho();
    
    // Enviar en formato: SENS:F=123,L=456,R=789
    Serial.print("SENS:F=");
    Serial.print(frontal);
    Serial.print(",L=");
    Serial.print(izquierdo);
    Serial.print(",R=");
    Serial.println(derecho);
}
```

## Solución de Problemas

### El programa no se conecta al ESP32

- Verifica el puerto serial correcto
- Asegúrate de que el ESP32 esté conectado
- Cierra otras aplicaciones que usen el puerto (Arduino IDE, etc.)
- En Linux, da permisos: `sudo chmod 666 /dev/ttyUSB0`

### La interfaz gráfica no se muestra

- Verifica que pygame esté instalado: `pip list | grep pygame`
- Reinstala pygame: `pip install --upgrade pygame`

### Los sensores no responden

- Verifica la implementación del protocolo en el ESP32
- Usa el modo manual para probar comandos individuales
- Revisa el monitor serial del ESP32

## Personalización

### Cambiar dimensiones del laberinto

En `flood_fill.py`:
```python
self.flood_fill = FloodFill(filas=7, columnas=12)  # Ajustar valores
```

### Ajustar umbral de detección de paredes

En `flood_fill.py` en el método `detectar_paredes_desde_sensores()`:
```python
umbral_pared = 120  # mm - ajustar según calibración
```

### Cambiar tamaño de celdas en interfaz

En `main_control.py`:
```python
self.interfaz = InterfazLaberinto(self.flood_fill, tamano_celda=80)  # píxeles
```

## Referencias

- [Flood Fill Algorithm](https://www.micromouseonline.com/2017/02/04/maze-solving-flooding-algorithm/)
- [Micromouse Design](https://www.micromouseonline.com/)

## Autor

Proyecto Micromouse - 2025
