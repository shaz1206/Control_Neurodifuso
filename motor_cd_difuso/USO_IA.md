# Declaración de uso de IA

**Herramienta:** Claude (Anthropic), usada mediante Claude Code.

## Para qué la usamos

- Generar la primera versión del código: motor de inferencia difusa, modelo del motor, simulación, firmware del ESP32 y scripts de HIL.
- Proponer la estructura del proyecto y la guía de defensa.

## Qué hicimos nosotros (criterio propio)

> ✏️ Completen esta sección con lo que realmente hicieron. Algunos ejemplos:

- Revisamos cada archivo y comprobamos a mano el ejemplo de inferencia (sección 3 de `GUIA_DEFENSA.md`).
- Entendimos por qué el controlador es incremental (PI) y no PD.
- Ajustamos pines, `PULSOS_POR_VUELTA`, `RPM_MAX` y `ZONA_MUERTA_V` según nuestro motor real.
- Comparamos con un PI clásico y reportamos con honestidad que el PI es más rápido en el caso nominal.
- Detectamos / corregimos: _(escriban aquí cualquier error de la IA que hayan encontrado o cambio que hayan hecho)_.

## Limitaciones que identificamos

- No tenemos sensor de velocidad. La Fase B es HIL: el motor físico recibe el voltaje del controlador, pero su velocidad real no se retroalimenta.
- Los parámetros del modelo son típicos, no los de nuestro motor.
- El FIS del ESP32 se validó contra Python con una emulación. La prueba HIL real confirma esa validación en el hardware.
