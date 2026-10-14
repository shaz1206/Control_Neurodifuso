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

## 6. ESP32-S3, encoder y motor real

**¿Cómo miden la velocidad del motor?**
Con el encoder Hall del GA25-370: dos señales, A y B, desfasadas 90°. Una interrupción se dispara en **cada flanco de A y de B** y una tabla de cuadratura suma +1 o −1 según el sentido. Eso es la lectura **×4**: 11 pulsos × 4 flancos × 45 de reducción = **1980 cuentas por vuelta**. Cada 10 ms: `rpm = (cuentas / 1980) · (60 / 0.01)`, más un filtro pasa-bajas (α = 0.4).

**¿Por qué leer ×4 y no solo un canal?**
A 90 rpm, con un solo canal serían ~7 pulsos cada 10 ms: saltos de ~13 rpm entre mediciones. Con ×4 son ~30 cuentas: saltos de 3 rpm. Además, con dos canales se sabe el sentido de giro.

**¿Cómo verificaron las 1980 cuentas?**
Con `calibrar_encoder.py`: el motor gira despacio y contamos 10 vueltas a ojo con una marca en el eje. Salieron 1982 por vuelta, o sea una reducción de 1:45.0.

**¿Qué diferencia hay entre HIL y el motor real?**
En HIL la velocidad sale de un modelo en la PC: demuestra que el código en C hace lo mismo que el de Python. En el motor real la velocidad sale del encoder y el ESP32 controla solo, en **lazo cerrado**: si frenan el eje con los dedos, el controlador lo nota y sube el voltaje.

**¿Por qué las ganancias dependen de las rpm máximas?**
El motor simulado llega a 2274 rpm y el real a 113. Ke y Kde están expresadas como fracción de la velocidad máxima (20 % y 6.6 %), así que la misma sintonía sirve para los dos: un error "grande" son 455 rpm en el modelo y 23 rpm en el motor real.

**¿Qué es la zona muerta y cómo la midieron?**
Es el voltaje mínimo para vencer la fricción. Con una rampa de voltaje sin compensación vimos que el motor arranca entre 6 y 8 V y se detiene en ~5 V. Por encima, la velocidad sube casi en línea recta y esa recta cruza el cero en ~6.3 V. El firmware usa 6.0 V: el rango 0–12 V del controlador se convierte en 6–12 V reales.

**¿Por qué al bajar de velocidad no se permite voltaje negativo?**
Antes, al bajar de 90 a 60 rpm el controlador pedía ~−1 V. Con la compensación de zona muerta eso eran **~−6.5 V reales**: un frenazo que hundía la velocidad hasta 38 rpm. Ahora, si la referencia es positiva, el voltaje mínimo es 0: el motor queda libre y lo frena su propia fricción. La caída bajó a ~50 rpm. En HIL no se limita, para que siga idéntico a Python.

**¿Qué hace "Parado" (referencia 0)?**
Deja el motor libre (0 V) y reinicia el controlador. Si se intentara controlar alrededor de 0 rpm, la zona muerta daría patadas de ±6 V.

**¿Qué protecciones tiene?**
Si el voltaje está al máximo durante 1 s y el motor no gira, o si el encoder cuenta al revés, el ESP32 apaga el motor y manda un aviso. En HIL, si la PC deja de mandar datos 300 ms, también lo apaga.

**¿Cómo se aplica el voltaje?**
PWM de 20 kHz y 10 bits en ENA (`duty = |u| / 12 · 1023`); IN1/IN2 eligen el sentido. A 20 kHz no se oye el zumbido.

**¿Qué muestra el LED?**
La etiqueta con mayor pertenencia del error: rojo NG, amarillo NP, verde Z, cian PP, azul PG. Al arrancar se ve azul → cian → verde.

**¿Qué resultados dio el motor real (90 → 60 rpm)?**
Los tres controladores llegan en ~0.12 s con menos de 0.3 rpm de error. Al bajar, Mamdani y Sugeno caen a ~50 rpm y el PI a ~45. Sugeno mueve más el voltaje con el ruido del encoder, igual que en la simulación: cerca de cero su superficie es más inclinada.

**¿Qué es la "velocidad en palabras" de la interfaz?**
Una variable lingüística más, con 5 etiquetas: Parado, Lento, Medio, Rápido y Muy rápido. Describe la velocidad igual que el controlador describe el error ("Va 60 % Rápido y 40 % Medio"). Es de presentación: el que decide el voltaje sigue siendo el FIS de e y de.

**¿Por qué la S3 usa pines distintos y el canal A va en el GPIO7?**
La ESP32-S3 no tiene los GPIO 25 a 33 (los usa la memoria flash). Además, el GPIO15 de nuestra placa se quedaba en bajo: con el comando `d` vimos que B cambiaba y A no, y movimos A al GPIO7.

## 7. Preguntas "trampa"

- **¿Qué pasa si cambian el orden de las filas de la tabla de reglas?** El controlador empuja en la dirección equivocada y el sistema se vuelve inestable.
- **¿Qué pasa si quitan el `clip` de u?** Windup: al bajar la referencia el motor tarda en responder, porque el acumulador quedó por encima de 12 V.
- **¿Cuántas reglas se activan como máximo a la vez?** 4, porque son 2 conjuntos por entrada.
- **¿Por qué `ePrev = ref` al reiniciar en el firmware?** Para que en el primer paso `de` no tenga un salto artificial.
- **¿Qué pasa si conectan el VCC del encoder (azul) a 5 V?** Las señales A y B saldrían a 5 V y la S3 solo aguanta 3.3 V en sus pines. Por eso va a 3V3.
- **¿Por qué el contador se quedaba en 0 aunque el motor giraba?** Solo cambiaba el canal B; sin A, la tabla de cuadratura suma +1 y −1 alternados y se cancela.
- **¿El suavizado de la interfaz afecta al control?** No. Solo suaviza el dibujo (0.12 s); el ESP32 controla con los datos reales cada 10 ms.
