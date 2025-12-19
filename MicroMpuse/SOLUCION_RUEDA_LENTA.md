# 🔧 Solución: Rueda Izquierda Muy Lenta / Rueda Derecha Muy Rápida

## 🔴 Problema Identificado

La rueda izquierda gira muy lento mientras la rueda derecha gira muy rápido. Esto indica que:

**El PID está corrigiendo en la dirección equivocada** porque uno de los componentes de hardware está mal configurado.

## 🎯 Causa Principal

Existen 3 causas posibles (en orden de probabilidad):

### 1. ⚡ Polaridad del Motor Invertida (MÁS COMÚN)
El motor izquierdo está conectado al revés, por lo que cuando el código envía "avanzar", el motor intenta ir hacia atrás, creando resistencia. El PID detecta que va más lento y aplica MÁS potencia, lo que empeora el problema.

### 2. 🔄 Encoder Contando al Revés
El encoder izquierdo está conectado con los canales A y B invertidos, por lo que cuenta negativo cuando debería contar positivo. El PID piensa que la rueda no se mueve y reduce su velocidad.

### 3. 🔩 Fricción Mecánica Excesiva
Hay algo trabando la rueda izquierda físicamente (tuerca apretada, eje desalineado, etc.)

---

## ✅ SOLUCIÓN RÁPIDA

### Opción 1: Invertir Polaridad del Motor Izquierdo

En el archivo `src/main.cpp`, busca la línea:

```cpp
SimpleMotor motorLeft(AIN1, AIN2, PWMA);
```

Y cámbiala por:

```cpp
SimpleMotor motorLeft(AIN2, AIN1, PWMA);  // ← Pines intercambiados
```

Esto invierte la polaridad del motor izquierdo.

---

### Opción 2: Invertir Encoder Izquierdo

Si la Opción 1 no funciona, busca la línea:

```cpp
encoderLeft.attachHalfQuad(ENC_L_A, ENC_L_B);
```

Y cámbiala por:

```cpp
encoderLeft.attachHalfQuad(ENC_L_B, ENC_L_A);  // ← Pines intercambiados
```

Esto invierte la dirección de conteo del encoder.

---

### Opción 3: Verificar Hardware

1. **Desconecta todo el código** y alimenta los motores directamente
2. **Aplica voltaje positivo** a ambos motores por separado
3. **Verifica que ambos giren en la misma dirección** (hacia adelante)
4. Si uno gira al revés, intercambia sus cables físicamente

---

## 🧪 Cómo Usar el Test de Diagnóstico

1. **Ejecuta la interfaz gráfica**
2. **Conecta el robot**
3. **Presiona el botón "🧪 Test Motores (5 seg)"**
4. **Observa visualmente** si ambas ruedas giran:
   - A la misma velocidad ✓
   - Una mucho más lenta que la otra ✗

El test te dará instrucciones específicas en el log.

---

## 📊 Explicación Técnica

### ¿Por qué pasa esto?

El algoritmo de control funciona así:

```
1. Mide encoders: Izq = 100 ticks, Der = 500 ticks
2. Calcula error: error = 100 - 500 = -400
3. Aplica corrección: 
   - Motor Izq = BASE_SPEED - (-400 * 1.2) = 130 + 480 = 610 (limitado a 255)
   - Motor Der = BASE_SPEED + (-400 * 1.2) = 130 - 480 = -350 (limitado a -255)
```

**Problema**: Si el motor izquierdo está al revés, cuando le das 255 de velocidad "adelante", en realidad va hacia atrás. El PID detecta que va lento y le da AÚN MÁS potencia, creando un ciclo vicioso.

### ¿Por qué no se detecta automáticamente?

El ESP32 no puede saber si los motores están bien conectados. Solo puede:
- Enviar PWM a los pines
- Leer pulsos de los encoders

No hay forma de que el código sepa si "adelante" es realmente adelante.

---

## 🔍 Verificación Post-Solución

Después de aplicar la solución:

1. **Sube el código** al ESP32
2. **Ejecuta el test** nuevamente
3. **Verifica que**:
   - Ambas ruedas giran a velocidad similar
   - El robot avanza en línea recta
   - No hay deriva excesiva

---

## 💡 Tips Adicionales

### Si ambas ruedas van bien pero el robot gira:
- Ajusta `Kp_enc` en el código (actualmente 1.2)
- Valores más altos = corrección más agresiva
- Valores más bajos = corrección más suave

### Si el robot vibra o se sacude:
- Reduce `Kp_enc` a 0.8 o 0.5
- Aumenta el delay en el loop a 10ms

### Si el robot va en línea recta pero no llega a la distancia correcta:
- Ajusta `CELL_SIZE` (actualmente 160.0mm)
- Mide la distancia real y ajusta proporcionalmente

---

## 📝 Checklist de Diagnóstico

- [ ] Ejecuté el test de diagnóstico
- [ ] Identifiqué cuál rueda va lenta
- [ ] Invertí la polaridad del motor correspondiente
- [ ] Volví a subir el código al ESP32
- [ ] Volví a ejecutar el test
- [ ] Ambas ruedas giran a velocidad similar
- [ ] El robot avanza recto sin deriva excesiva

---

## 🆘 Si Nada Funciona

Si después de invertir polaridades y encoders el problema persiste:

1. **Verifica las conexiones físicas**:
   - Cables sueltos
   - Pines doblados
   - Soldaduras frías

2. **Verifica los pines en el código**:
   ```cpp
   #define ENC_L_A 25  // ¿Correcto?
   #define ENC_L_B 26  // ¿Correcto?
   const int AIN1 = 32; // ¿Correcto?
   const int AIN2 = 33; // ¿Correcto?
   ```

3. **Prueba intercambiar completamente** motor izquierdo con derecho:
   - Si el problema cambia de lado, es el motor
   - Si el problema se queda en el mismo lado, es el código o los pines

---

## ✨ Resultado Esperado

Después de la corrección:
- ✅ Ambas ruedas giran a la misma velocidad
- ✅ El robot avanza en línea recta
- ✅ Los encoders cuentan valores similares
- ✅ El PID hace correcciones pequeñas (<30 de diferencia)
