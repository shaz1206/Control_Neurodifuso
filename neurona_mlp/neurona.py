"""
Una sola neurona (perceptrón de Rosenblatt) escrita desde cero con numpy.

    salida = escalon(w1*x1 + w2*x2 + b)          escalon(z) = 1 si z >= 0, si no 0

Regla de aprendizaje del perceptrón, para cada ejemplo:
    error = y_deseada - y_obtenida               (vale -1, 0 o 1)
    w  <- w + tasa * error * x
    b  <- b + tasa * error

Se entrena con las compuertas AND, OR y XOR. AND y OR se aprenden en pocas
épocas porque son LINEALMENTE SEPARABLES (una recta separa los 1 de los 0).
XOR no lo es: ninguna recta lo separa, así que una sola neurona nunca lo
aprende. Para eso hace falta el perceptrón multicapa (mlp.py).

Uso:  python neurona.py      -> imprime el entrenamiento y guarda resultados/perceptron.png
"""
import os

import numpy as np

CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultados")

# Las 4 combinaciones de entrada y la salida deseada de cada compuerta
X = np.array([[0, 0], [0, 1], [1, 0], [1, 1]], dtype=float)
COMPUERTAS = {
    "AND": np.array([0, 0, 0, 1]),
    "OR": np.array([0, 1, 1, 1]),
    "XOR": np.array([0, 1, 1, 0]),
}


def escalon(z):
    return (np.asarray(z) >= 0).astype(int)


class Perceptron:
    def __init__(self, n_entradas=2, tasa=0.1, semilla=0):
        rng = np.random.default_rng(semilla)
        self.w = rng.uniform(-0.5, 0.5, n_entradas)  # pesos iniciales pequeños al azar
        self.b = 0.0
        self.tasa = tasa

    def predecir(self, X):
        return escalon(X @ self.w + self.b)

    def entrenar(self, X, y, max_epocas=100):
        """Recorre los ejemplos hasta que no haya errores o se acaben las épocas.
        Devuelve la lista de errores por época (ejemplos mal clasificados)."""
        historial = []
        for _ in range(max_epocas):
            errores = 0
            for xi, yi in zip(X, y):
                error = yi - self.predecir(xi)
                if error != 0:
                    self.w += self.tasa * error * xi
                    self.b += self.tasa * error
                    errores += 1
            historial.append(errores)
            if errores == 0:  # convergió: ya clasifica todo bien
                break
        return historial


def main():
    resultados = {}
    for nombre, y in COMPUERTAS.items():
        neurona = Perceptron()
        historial = neurona.entrenar(X, y)
        aciertos = (neurona.predecir(X) == y).mean() * 100
        resultados[nombre] = (neurona, historial)
        estado = f"convergió en {len(historial)} épocas" if historial[-1] == 0 \
            else f"NO convergió en {len(historial)} épocas"
        print(f"{nombre:<4} {estado:<28} aciertos {aciertos:5.1f} %   "
              f"w = [{neurona.w[0]:+.2f}, {neurona.w[1]:+.2f}]  b = {neurona.b:+.2f}")
    print("\nXOR no se puede: ninguna recta separa (0,1) y (1,0) de (0,0) y (1,1).")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(CARPETA, exist_ok=True)
    fig, ejes = plt.subplots(2, 3, figsize=(13, 8))
    malla = np.linspace(-0.5, 1.5, 300)
    M1, M2 = np.meshgrid(malla, malla)
    for k, (nombre, (neurona, historial)) in enumerate(resultados.items()):
        ax = ejes[0, k]
        zona = neurona.predecir(np.c_[M1.ravel(), M2.ravel()]).reshape(M1.shape)
        ax.contourf(M1, M2, zona, levels=[-0.5, 0.5, 1.5], colors=["#fde2e2", "#dff3e4"])
        y = COMPUERTAS[nombre]
        ax.scatter(X[y == 0, 0], X[y == 0, 1], s=140, c="#e5484d", label="salida 0", zorder=3)
        ax.scatter(X[y == 1, 0], X[y == 1, 1], s=140, c="#30a46c", label="salida 1", zorder=3)
        if abs(neurona.w[1]) > 1e-9:  # recta de decisión: w1*x1 + w2*x2 + b = 0
            ax.plot(malla, -(neurona.w[0] * malla + neurona.b) / neurona.w[1], "k--", lw=1.5)
        ax.set(xlim=(-0.5, 1.5), ylim=(-0.5, 1.5), xlabel="x1", ylabel="x2",
               title=f"{nombre}: {'aprendido' if historial[-1] == 0 else 'NO se puede con una recta'}")
        ax.legend(loc="upper right", fontsize=8)
        ax2 = ejes[1, k]
        ax2.plot(range(1, len(historial) + 1), historial, "o-", ms=3)
        ax2.set(xlabel="Época", ylabel="Ejemplos mal clasificados", title=f"Entrenamiento {nombre}",
                ylim=(-0.2, 4.2))
        ax2.grid(alpha=0.3)
    fig.suptitle("Una neurona (perceptrón): aprende AND y OR, pero no XOR")
    fig.tight_layout()
    ruta = os.path.join(CARPETA, "perceptron.png")
    fig.savefig(ruta, dpi=120)
    print(f"Gráfica guardada en {ruta}")


if __name__ == "__main__":
    main()
