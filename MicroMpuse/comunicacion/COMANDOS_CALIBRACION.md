# 🎮 Comandos de Calibración de Motores

## Descripción General

La interfaz gráfica ahora incluye un panel de **Control Manual de Motores** que permite calibrar y probar el movimiento del robot mediante botones individuales.

## 📋 Comandos Implementados

### 1. ⬆️ Avanzar 1 Celda
- **Comando enviado**: `FORWARD`
- **Descripción**: El robot debe avanzar exactamente una celda del laberinto (160mm aproximadamente)
- **Ticks necesarios**: `TICKS_CELL` (calculados según el diámetro de la rueda)

### 2. ↪️ Giro 90° Izquierda
- **Comando enviado**: `TURNL`
- **Descripción**: Girar 90 grados en sentido antihorario
- **Ticks necesarios**: `TICKS_TURN_90`

### 3. ↩️ Giro 90° Derecha
- **Comando enviado**: `TURNR`
- **Descripción**: Girar 90 grados en sentido horario
- **Ticks necesarios**: `TICKS_TURN_90`

### 4. 🔄 Giro 180° (Media Vuelta)
- **Comando enviado**: `TURNU`
- **Descripción**: Girar 180 grados (media vuelta)
- **Ticks necesarios**: `TICKS_TURN_180`

### 5. ⬇️ Retroceder 1 Celda
- **Comando enviado**: `BACKWARD`
- **Descripción**: El robot debe retroceder exactamente una celda del laberinto
- **⚠️ IMPORTANTE**: Este comando necesita ser implementado en el código del ESP32

## 🔧 Implementación del Comando BACKWARD

Para que el control manual funcione completamente, debes añadir el comando `BACKWARD` en el archivo `main.cpp`:

```cpp
// En la función donde procesas los comandos UDP:

if (cmd == "BACKWARD") {
    isBusy = true;
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((const uint8_t*)"ACK", 3);
    udp.endPacket();
    
    // Resetear encoders
    encoderL.clearCount();
    encoderR.clearCount();
    
    int targetTicks = -TICKS_CELL;  // Negativo para ir hacia atrás
    
    while (abs(encoderL.getCount()) < abs(targetTicks)) {
        long ticksL = encoderL.getCount();
        long ticksR = encoderR.getCount();
        
        // Calcular error entre encoders
        int error = ticksR - ticksL;
        int correction = constrain(error * Kp_enc, -MAX_CORR, MAX_CORR);
        
        // Velocidades con corrección (negativas para retroceder)
        int speedL = -(BASE_SPEED - correction);
        int speedR = -(BASE_SPEED + correction);
        
        // Aplicar velocidades
        setMotorSpeed(speedL, speedR);
        
        delay(5);
    }
    
    // Detener motores
    setMotorSpeed(0, 0);
    isBusy = false;
}
```

## 💡 Uso del Control Manual

1. **Conecta el robot** usando el botón "🔌 Conectar"
2. **Asegúrate de que no haya procesos automáticos** activos
3. **Presiona los botones** del panel "Control Manual de Motores" según necesites
4. **Observa el log** para ver el estado de cada comando
5. **El indicador de estado** muestra si el comando se ejecutó correctamente

## 🎯 Casos de Uso

### Calibración de Distancia de Celda
1. Coloca el robot en una posición conocida
2. Presiona "⬆️ Avanzar 1 Celda"
3. Mide la distancia real recorrida
4. Ajusta `CELL_SIZE` o `TICKS_PER_MM` en el código del ESP32 si es necesario

### Calibración de Giros
1. Marca la orientación inicial del robot
2. Presiona los botones de giro
3. Verifica que los ángulos sean exactos (90° o 180°)
4. Ajusta `TICKS_TURN_90` si es necesario

### Prueba de Retroceso
1. Presiona "⬇️ Retroceder 1 Celda"
2. Verifica que la distancia sea la misma que al avanzar
3. Ajusta la implementación si hay diferencias

## ⚠️ Notas Importantes

- Los comandos manuales **NO están disponibles** durante modo automático o calibración de sensores
- La **posición virtual** del robot en la interfaz se actualiza con cada movimiento
- Cada comando espera una respuesta `ACK` del robot antes de continuar
- Si el robot no responde en 10 segundos, se produce un **timeout**
- El **indicador de estado** muestra el progreso de cada comando

## 🔄 Integración con el Mapa

- Los movimientos manuales actualizan la posición virtual del robot
- El mapa visual se redibuja mostrando la nueva posición
- La orientación del robot se actualiza con los giros

## 🐛 Troubleshooting

**Problema**: El comando no se ejecuta
- ✓ Verifica que el robot esté conectado
- ✓ Asegúrate de que no hay procesos automáticos activos
- ✓ Revisa el log para ver el mensaje de error específico

**Problema**: El robot no retrocede
- ✓ Implementa el comando `BACKWARD` en el ESP32
- ✓ Verifica que los motores puedan girar en sentido inverso

**Problema**: Los movimientos no son precisos
- ✓ Calibra los valores de `TICKS_CELL` y `TICKS_TURN_90`
- ✓ Verifica que los encoders estén funcionando correctamente
- ✓ Ajusta los parámetros PID si es necesario
