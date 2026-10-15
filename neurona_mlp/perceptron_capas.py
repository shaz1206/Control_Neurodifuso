"""
Perceptrón multicapa con 1, 2 y 3 capas ocultas: ¿qué cambia al agregar capas?

Cada capa es un grupo de neuronas iguales a la "neurona pelona", pero:
  - en vez del escalón usan tanh (una curva suave, necesaria para poder calcular
    cuánto culpa tiene cada peso del error: el escalón no tiene derivada útil);
  - la salida usa sigmoide, que da un número entre 0 y 1 (>= 0.5 se toma como 1).

Las tres redes tienen 6 neuronas por capa oculta y se entrenan igual
(misma tasa, mismas épocas). Solo cambia el número de capas:

    1 capa :  2 entradas -> [6] -> 1 salida
    2 capas:  2 entradas -> [6] -> [6] -> 1 salida
    3 capas:  2 entradas -> [6] -> [6] -> [6] -> 1 salida

Problemas:
  1. XOR (4 puntos): fácil, lo resuelven las tres.
  2. Espiral (200 puntos, dos brazos enrollados): difícil, ahí se ven las diferencias.

Uso:  python perceptron_capas.py    -> imprime tablas y guarda resultados/capas_*.png
"""
import os

import numpy as np

CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultados")
NEURONAS = 6          # neuronas en cada capa oculta
TASA = 0.3            # tasa de aprendizaje
EPOCAS = 5000


def sigmoide(z):
    return 1.0 / (1.0 + np.exp(-z))


class RedMulticapa:
    def __init__(self, n_capas_ocultas, neuronas=NEURONAS, semilla=0):
        rng = np.random.default_rng(semilla)
        tamanos = [2] + [neuronas] * n_capas_ocultas + [1]   # p. ej. [2, 6, 6, 1]
        # Una matriz de pesos W y un vector de sesgos b por cada capa.
        # W[k][i, j] = peso de la neurona i de la capa anterior hacia la neurona j.
        self.W = [rng.normal(0, np.sqrt(1 / n), (n, m)) for n, m in zip(tamanos[:-1], tamanos[1:])]
        self.b = [np.zeros(m) for m in tamanos[1:]]
        self.vW = [np.zeros_like(w) for w in self.W]  # "inercia" del momento
        self.vb = [np.zeros_like(b) for b in self.b]

    def n_pesos(self):
        return sum(w.size + b.size for w, b in zip(self.W, self.b))

    def adelante(self, X):
        """Pasa las entradas capa por capa. Cada capa hace lo mismo que la neurona
        pelona (suma ponderada + sesgo) pero con muchas neuronas a la vez."""
        self.a = [X]
        for k in range(len(self.W)):
            z = self.a[-1] @ self.W[k] + self.b[k]
            es_salida = k == len(self.W) - 1
            self.a.append(sigmoide(z) if es_salida else np.tanh(z))
        return self.a[-1]

    def entrenar_una_epoca(self, X, Y, tasa=TASA, momento=0.9):
        """Retropropagación: el error de la salida se reparte hacia atrás, capa por
        capa, para saber cuánto corregir cada peso."""
        y = self.adelante(X)
        perdida = np.mean((y - Y) ** 2)
        delta = 2 * (y - Y) / len(Y) * y * (1 - y)        # culpa en la salida
        for k in reversed(range(len(self.W))):
            gW = self.a[k].T @ delta                       # cuánto corregir cada peso
            gb = delta.sum(axis=0)
            if k > 0:                                      # culpa de la capa anterior
                delta = (delta @ self.W[k].T) * (1 - self.a[k] ** 2)
            self.vW[k] = momento * self.vW[k] - tasa * gW
            self.vb[k] = momento * self.vb[k] - tasa * gb
            self.W[k] += self.vW[k]
            self.b[k] += self.vb[k]
        return perdida

    def predecir(self, X):
        return (self.adelante(X) >= 0.5).astype(int)


# ------------------------------------------------------------------ datos
def datos_xor():
    X = np.array([[0, 0], [0, 1], [1, 0], [1, 1]], dtype=float)
    Y = np.array([[0], [1], [1], [0]], dtype=float)
    return X, Y


def datos_espiral(n=100, vueltas=2, semilla=0):
    """Dos espirales enrolladas una dentro de la otra: clase 0 y clase 1."""
    rng = np.random.default_rng(semilla)
    t = np.sqrt(rng.uniform(0, 1, n)) * vueltas * 2 * np.pi
    brazo = np.c_[t * np.cos(t), t * np.sin(t)] / (vueltas * 2 * np.pi)
    X = np.r_[brazo, -brazo]
    Y = np.r_[np.zeros(n), np.ones(n)][:, None]
    return X, Y


def comparar(nombre, X, Y, umbral=0.02):
    """Entrena las redes de 1, 2 y 3 capas con los mismos datos y las compara."""
    print(f"\n=== {nombre} ({len(X)} puntos) ===")
    print(f"{'capas':>5} {'pesos':>6} {'aciertos':>9} {'pérdida final':>14} {'épocas p/ aprender':>19}")
    resultados = []
    for capas in (1, 2, 3):
        red = RedMulticapa(capas)
        historial = [red.entrenar_una_epoca(X, Y) for _ in range(EPOCAS)]
        aciertos = (red.predecir(X) == Y).mean() * 100
        aprendio = next((i + 1 for i, p in enumerate(historial) if p < umbral), None)
        print(f"{capas:>5} {red.n_pesos():>6} {aciertos:>8.1f}% {historial[-1]:>14.4f} "
              f"{aprendio if aprendio else 'no llegó':>19}")
        resultados.append((capas, red, historial, aciertos))
    return resultados


def graficar(nombre, archivo, X, Y, resultados, limite):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ejes = plt.subplots(1, 4, figsize=(18, 4.6))
    malla = np.linspace(-limite, limite, 250) if limite else np.linspace(-0.5, 1.5, 250)
    M1, M2 = np.meshgrid(malla, malla)
    for ax, (capas, red, historial, aciertos) in zip(ejes, resultados):
        Z = red.adelante(np.c_[M1.ravel(), M2.ravel()]).reshape(M1.shape)
        ax.contourf(M1, M2, Z, levels=20, cmap="RdYlGn", vmin=0, vmax=1)
        ax.contour(M1, M2, Z, levels=[0.5], colors="k", linewidths=1)
        ax.scatter(*X[Y[:, 0] == 0].T, c="#e5484d", s=18, edgecolors="k", linewidths=0.4)
        ax.scatter(*X[Y[:, 0] == 1].T, c="#30a46c", s=18, edgecolors="k", linewidths=0.4)
        ax.set(title=f"{capas} capa{'s' if capas > 1 else ''} oculta{'s' if capas > 1 else ''}"
                     f" ({red.n_pesos()} pesos): {aciertos:.0f} %", xticks=[], yticks=[])
        ejes[3].semilogy(historial, label=f"{capas} capa{'s' if capas > 1 else ''}")
    ejes[3].set(title="Pérdida durante el entrenamiento", xlabel="Época", ylabel="Pérdida")
    ejes[3].legend()
    ejes[3].grid(alpha=0.3, which="both")
    fig.suptitle(f"{nombre}: misma red, mismo entrenamiento, distinto número de capas")
    fig.tight_layout()
    fig.savefig(os.path.join(CARPETA, archivo), dpi=110)


def main():
    os.makedirs(CARPETA, exist_ok=True)
    X, Y = datos_xor()
    res = comparar("XOR", X, Y)
    graficar("XOR", "capas_xor.png", X, Y, res, None)

    X, Y = datos_espiral()
    res = comparar("Espiral", X, Y)
    graficar("Espiral", "capas_espiral.png", X, Y, res, 1.1)
    print(f"\nGráficas guardadas en {CARPETA}")


if __name__ == "__main__":
    main()
