"""
Una neurona "pelona": solo Python, sin librerías.

    z = w1*x1 + w2*x2 + b             suma ponderada
    y = 1 si z >= 0, si no 0          función de activación (escalón)

Se entrena para que aprenda la compuerta AND.
Regla de aprendizaje (perceptrón):
    error = y_deseada - y
    w = w + tasa * error * x
    b = b + tasa * error

Uso:  python neurona_pelona.py
"""

# Datos de entrenamiento: entradas (x1, x2) y salida deseada de AND
datos = [
    ([0, 0], 0),
    ([0, 1], 0),
    ([1, 0], 0),
    ([1, 1], 1),
]

# Pesos y sesgo iniciales (pueden ser cualquier número pequeño)
w1, w2, b = 0.3, -0.2, 0.0
tasa = 0.1  # tasa de aprendizaje: qué tan grande es cada corrección


def neurona(x1, x2):
    z = w1 * x1 + w2 * x2 + b
    return 1 if z >= 0 else 0


print(f"Inicio:  w1={w1:+.2f}  w2={w2:+.2f}  b={b:+.2f}\n")

for epoca in range(1, 101):
    print(f"===== Época {epoca} =====")
    errores = 0
    for (x1, x2), deseada in datos:
        z = w1 * x1 + w2 * x2 + b
        y = neurona(x1, x2)
        error = deseada - y
        print(f"  Entrada ({x1},{x2}):  z = {w1:+.2f}*{x1} {w2:+.2f}*{x2} {b:+.2f} = {z:+.2f}"
              f"  -> dice {y}, debía ser {deseada}", end="")
        if error != 0:  # si se equivocó, corrige pesos y sesgo
            w1 += tasa * error * x1
            w2 += tasa * error * x2
            b += tasa * error
            errores += 1
            print(f"  ✗ error={error:+d}  -> corrige: w1={w1:+.2f}  w2={w2:+.2f}  b={b:+.2f}")
        else:
            print("  ✓ bien, no cambia nada")
    print(f"  Fin de la época {epoca}: {errores} errores\n")
    if errores == 0:  # ya acertó todos: terminó de aprender
        break

print("\nPrueba de la neurona entrenada:")
for (x1, x2), deseada in datos:
    print(f"  {x1} AND {x2} = {neurona(x1, x2)}   (deseado {deseada})")
