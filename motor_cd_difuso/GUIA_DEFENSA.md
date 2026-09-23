# Guía de defensa

El profesor pregunta a cualquiera **al azar**: si uno falla, fallamos todos. Cada integrante debe poder responder **todas** estas preguntas sin leer. Practiquen haciéndose las preguntas entre ustedes.

---

## 1. Conceptos de lógica difusa

**¿Qué es un conjunto difuso?**
Es un conjunto en el que cada elemento pertenece con un **grado** entre 0 y 1, no solo "sí o no". Por ejemplo, un error de 0.3 es "Cero" con grado 0.4 y "Positivo Pequeño" con grado 0.6.

**¿Cuáles son los pasos de un sistema de inferencia difusa?**
1. **Fuzzificación:** convertir el número de entrada en grados de pertenencia.
2. **Evaluación de reglas:** calcular la fuerza de cada regla con el AND (usamos `min`).
3. **Agregación:** juntar las salidas de todas las reglas (usamos `max` en Mamdani).
4. **Defuzzificación:** convertir el resultado en un solo número (centroide en Mamdani, promedio ponderado en Sugeno).

**¿Diferencia entre Mamdani y Sugeno?**
- En **Mamdani**, la salida de cada regla es un **conjunto difuso** (un triángulo). Cada triángulo se recorta, se unen todos y se calcula el centroide del área. Es más interpretable, pero más costoso de calcular.
- En **Sugeno**, la salida de cada regla es un **número** (orden 0) o una función lineal de las entradas (orden 1). El resultado es `Σ(w·z) / Σw`. Es más rápido y se presta mejor para microcontroladores.

**¿Por qué la superficie de Mamdani nunca llega a ±1?**
Porque el centroide de un triángulo de hombro (por ejemplo, PG entre 0.5 y 1) queda en la mitad de su área, alrededor de 0.83, y nunca en 1. Sugeno usa el singleton en 1, así que sí llega.

**¿Por qué triángulos con 50 % de traslape?**
Con ese traslape los grados de pertenencia siempre suman 1 (hay una prueba que lo verifica). Además, cualquier entrada activa como máximo 2 conjuntos por variable, o sea como máximo 4 reglas. Y la salida cambia de forma suave, sin saltos.

**¿Para qué sirven los "hombros" en NG y PG?**
Para que un valor fuera del rango, como un error enorme, siga siendo "Grande" con grado 1 y no quede sin clasificar.

## 2. Nuestro controlador

**¿Cuáles son las entradas y la salida?**
Entradas: el error `e = ref − rpm` y su cambio `de = e_k − e_(k−1)`. Salida: el **cambio** de voltaje `du`. El voltaje real es `u = u_anterior + du`.

**¿Por qué incremental (du) y no el voltaje directo?**
Porque al acumular `du` el controlador se comporta como un **PI**, y la parte integral elimina el error en estado estacionario. Si el FIS diera `u` directo sería un PD: siempre quedaría un error, porque el motor necesita voltaje distinto de cero para girar aunque el error sea 0.

**¿Qué hacen Ke, Kde y Kdu?**
- `Ke` y `Kde` **normalizan** las entradas al universo [-1, 1]. Por ejemplo, `Ke = 1/455` significa que 455 rpm de error ya cuenta como "Grande".
- `Kdu` convierte la salida normalizada en volts.
- Subir `Kdu` hace el control más agresivo: más rápido, pero amplifica más el ruido.
- Subir `Ke` (dividir entre un número menor) da más peso al error: más rápido, pero con más riesgo de sobrepaso.
- Subir `Kde` da más amortiguamiento.

**¿Cómo sintonizaron las ganancias?**
Con un barrido de valores en simulación. Elegimos el menor IAE sin sobrepaso, cuidando que el voltaje no vibrara con ruido. Con Kdu = 3 era más rápido pero ruidoso; con Kdu = 2 era más lento; quedamos en 2.5.

**Explica una regla.**
"SI e es PP Y de es NP ENTONCES du es Z": estamos un poco por debajo de la referencia, pero el error ya está bajando (nos acercamos rápido). No hace falta subir más el voltaje, porque si lo subimos habría sobrepaso.

**¿Qué significa la diagonal de Z en la tabla?**
Son los casos en que el error y su tendencia se compensan. Es la "línea de equilibrio": en esas condiciones no se toca el voltaje.

**¿Cómo evitan el windup?**
Saturando `u` a ±12 V en **cada** paso. El acumulador nunca pasa del límite físico, así que al bajar la referencia responde de inmediato.

## 3. Ejemplo que deben saber hacer a mano (en pizarrón)

Entradas normalizadas: e = 0.3 y de = −0.2 (se genera con `python superficie.py`).

1. **Fuzzificación:**
   - e = 0.3 → Z = (0.5 − 0.3)/0.5 = **0.4**, PP = (0.3 − 0)/0.5 = **0.6**
   - de = −0.2 → NP = **0.4**, Z = **0.6**
2. **Reglas activas (min):**
   - (Z, NP) → NP, w = min(0.4, 0.4) = 0.4
   - (Z, Z) → Z, w = 0.4
   - (PP, NP) → Z, w = 0.4
   - (PP, Z) → PP, w = 0.6
3. **Sugeno:** (0.4·(−0.5) + 0.4·0 + 0.4·0 + 0.6·0.5) / (0.4 + 0.4 + 0.4 + 0.6) = 0.1 / 1.8 = **0.0556**
4. **Mamdani:** se recorta NP a 0.4, Z a 0.4 y PP a 0.6, se unen y se saca el centroide. Resultado: **0.061**.
5. **Voltaje:** du = Kdu · 0.0556 = 2.5 · 0.0556 ≈ **0.14 V** que se suma al voltaje anterior.

## 4. Modelo del motor

**¿Cuáles son las ecuaciones?**
- Eléctrica: `L di/dt = V − R i − K w`
- Mecánica: `J dw/dt = K i − b w − T_carga`

`K w` es la fuerza contraelectromotriz y `K i` es el par del motor.

**¿Por qué RK4 con dt = 0.1 ms, si el control es cada 10 ms?**
La parte eléctrica es muy rápida (L/R = 2.5 ms) y Euler con un paso grande se vuelve inestable. El controlador, igual que en el ESP32, solo actúa cada 10 ms y mantiene el voltaje entre muestras (retenedor de orden cero).

**¿Cómo se calcula la velocidad máxima?**
En estado estacionario las derivadas son 0, así que `w = V·K / (R·b + K²)`. Con 12 V da ≈ 2274 rpm.

## 5. Resultados

**¿Cuál es mejor, difuso o PI?**
Depende. Con el modelo nominal el PI es más rápido (0.15 s contra 0.19 s). Pero el difuso:
- pierde menos velocidad al entrar la carga (54–65 rpm contra 101 rpm);
- es **robusto**: con el doble de inercia su sobrepaso queda por debajo de 1 %, mientras el PI llega a 8.8 %.

El motor real nunca es igual al modelo, y ahí está la ventaja.

**¿Mamdani o Sugeno?**
Dan resultados casi iguales. Sugeno es más barato de calcular (no recorre 201 puntos) y rechaza un poco mejor la carga. Mamdani filtra mejor el ruido, porque cerca de cero su superficie es más plana.

**¿Qué métricas usaron?**
- Sobrepaso %.
- Tiempo de subida (10–90 %).
- Tiempo de establecimiento (banda del 2 %).
- Error final.
- IAE (integral del error absoluto).

## 6. ESP32 y HIL

**¿Cómo miden la velocidad del motor físico?**
No la medimos: no tenemos sensor. Por eso la Fase B es **HIL**. La velocidad viene del modelo en la PC, y el ESP32 calcula el voltaje con el FIS. Ese voltaje también se aplica al motor físico para verlo reaccionar, pero el motor real está en **lazo abierto**: si lo frenan con la mano, el controlador no se entera.

**Entonces, ¿qué demuestra el motor girando?**
Que la cadena completa funciona con hardware real: FIS en el ESP32 → PWM → puente H → motor. Y que el voltaje hace lo esperado: arranque fuerte, bajada en 1.5 s, más voltaje cuando entra la carga en 2.5 s.

**¿Qué necesitarían para el lazo cerrado real?**
Un sensor de velocidad. El más barato es un optoacoplador de ranura (FC-03) con un disco ranurado. Con un encoder, el firmware contaría pulsos con una interrupción: `rpm = (pulsos / PULSOS_POR_VUELTA) · (60 / 0.01)`, más un filtro pasa-bajas (α = 0.4). Ese código ya está: basta con poner `HAY_SENSOR = true`.

**¿Por qué el HIL corre a tiempo real?**
Para que el motor físico reciba cada voltaje durante 10 ms, igual que en el modelo. Si corriera más rápido, el motor no alcanzaría a reaccionar.

**¿Qué pasa si se cierra el script a mitad del HIL?**
El ESP32 tiene un "watchdog": si pasan 300 ms sin recibir datos de la PC, apaga el motor.

**¿Cómo se aplica el voltaje?**
Con PWM de 20 kHz y 10 bits: `duty = |u| / 12 · 1023`. IN1/IN2 eligen el sentido. Usamos 20 kHz para que no se oiga el zumbido.

**¿Qué es la zona muerta?**
Es el voltaje mínimo que el motor necesita para vencer la fricción estática. Por debajo de él no gira. El firmware la compensa sumando ese voltaje mínimo.

**¿Qué es HIL y qué demuestra?**
Hardware-in-the-Loop: el controlador corre en el hardware real (ESP32) y la planta se simula en la PC. Demuestra que el código embebido (C, float32, tiempo discreto) hace exactamente lo mismo que el diseño en Python antes de arriesgar el motor.

**¿Por qué las ganancias en el firmware dependen de RPM_MAX?**
Porque el motor real (con reductor) gira mucho más lento que el simulado. Al expresar Ke y Kde como fracción de la velocidad máxima, la misma sintonía se adapta a cualquier motor.


## 7. Preguntas "trampa"

- **¿Qué pasa si cambian el orden de las filas de la tabla de reglas?** El controlador empuja en la dirección equivocada y el sistema se vuelve inestable.
- **¿Qué pasa si quitan el `clip` de u?** Windup: al bajar la referencia el motor tarda en responder, porque el acumulador quedó por encima de 12 V.
- **¿Cuántas reglas se activan como máximo a la vez?** 4, porque son 2 conjuntos por entrada.
- **¿Por qué `ePrev = ref` al reiniciar en el firmware?** Para que en el primer paso `de` no tenga un salto artificial.
