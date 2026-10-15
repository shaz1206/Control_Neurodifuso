"""
Perceptrón multicapa (MLP) escrito desde cero con numpy y entrenado con
retropropagación (backpropagation) y descenso de gradiente con momento.

Arquitectura: capas totalmente conectadas.
    capa oculta:  a = tanh(a_anterior @ W + b)
    capa salida:  y = sigmoide(z)  (clasificación, 0..1)   o   y = z  (regresión)
Pérdida: error cuadrático medio  L = mean((y - y_deseada)^2)

Dos experimentos:
  1. XOR: lo que una sola neurona NO pudo (neurona.py), el MLP sí lo aprende.
  2. Imitar al controlador difuso de la práctica del motor: el MLP aprende la
     superficie du = FIS(e, de) de Mamdani y se compara con el original.

Uso:  python mlp.py      -> imprime el entrenamiento y guarda gráficas en resultados/
"""
import os
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
CARPETA = os.path.join(AQUI, "resultados")


def sigmoide(z):
    return 1.0 / (1.0 + np.exp(-z))


class MLP:
    def __init__(self, capas, salida="sigmoide", semilla=0):
        """capas: neuronas por capa, p. ej. [2, 4, 1] = 2 entradas, 4 ocultas, 1 salida.
        salida: "sigmoide" (clasificación) o "lineal" (regresión)."""
        rng = np.random.default_rng(semilla)
        # Inicialización de Xavier: pesos ~ N(0, 1/n_entradas) para que tanh no se sature
        self.W = [rng.normal(0, np.sqrt(1.0 / n_in), (n_in, n_out))
                  for n_in, n_out in zip(capas[:-1], capas[1:])]
        self.b = [np.zeros(n_out) for n_out in capas[1:]]
        self.salida = salida
        self.vW = [np.zeros_like(w) for w in self.W]  # "velocidades" del momento
        self.vb = [np.zeros_like(b) for b in self.b]

    def adelante(self, X):
        """Propagación hacia adelante. Guarda las activaciones de cada capa."""
        self.a = [X]
        for k, (W, b) in enumerate(zip(self.W, self.b)):
            z = self.a[-1] @ W + b
            ultima = k == len(self.W) - 1
            if not ultima:
                self.a.append(np.tanh(z))
            else:
                self.a.append(sigmoide(z) if self.salida == "sigmoide" else z)
        return self.a[-1]

    def perdida(self, Y):
        return float(np.mean((self.a[-1] - Y) ** 2))

    def gradientes(self, Y):
        """Retropropagación: derivadas de la pérdida respecto a cada W y b.

        delta = dL/dz de la capa. En la salida:
            lineal:    delta = 2 (y - y_d) / N
            sigmoide:  delta = 2 (y - y_d) / N * y (1 - y)
        Hacia atrás, por la regla de la cadena:
            delta_anterior = (delta @ W^T) * (1 - a^2)      (derivada de tanh)
            dL/dW = a_anterior^T @ delta,   dL/db = suma(delta)
        """
        y = self.a[-1]
        delta = 2.0 * (y - Y) / Y.size
        if self.salida == "sigmoide":
            delta = delta * y * (1.0 - y)
        gW, gb = [None] * len(self.W), [None] * len(self.b)
        for k in reversed(range(len(self.W))):
            gW[k] = self.a[k].T @ delta
            gb[k] = delta.sum(axis=0)
            if k > 0:
                delta = (delta @ self.W[k].T) * (1.0 - self.a[k] ** 2)
        return gW, gb

    def entrenar(self, X, Y, tasa=0.1, epocas=1000, momento=0.9, cada=None):
        """Descenso de gradiente con momento (lote completo).
        v <- momento * v - tasa * gradiente ;  w <- w + v
        Devuelve la pérdida de cada época."""
        historial = []
        for epoca in range(epocas):
            self.adelante(X)
            historial.append(self.perdida(Y))
            gW, gb = self.gradientes(Y)
            for k in range(len(self.W)):
                self.vW[k] = momento * self.vW[k] - tasa * gW[k]
                self.vb[k] = momento * self.vb[k] - tasa * gb[k]
                self.W[k] += self.vW[k]
                self.b[k] += self.vb[k]
            if cada and (epoca % cada == 0 or epoca == epocas - 1):
                print(f"  época {epoca:6d}   pérdida {historial[-1]:.6f}")
        return historial

    def predecir(self, X):
        return self.adelante(X)


# ------------------------------------------------------------------ experimentos
X_XOR = np.array([[0, 0], [0, 1], [1, 0], [1, 1]], dtype=float)
Y_XOR = np.array([[0], [1], [1], [0]], dtype=float)


def experimento_xor(semilla=0, verbose=True):
    red = MLP([2, 4, 1], salida="sigmoide", semilla=semilla)
    historial = red.entrenar(X_XOR, Y_XOR, tasa=0.5, epocas=3000, momento=0.9,
                             cada=500 if verbose else None)
    return red, historial


def datos_fuzzy(n=21, metodo="mamdani"):
    """Muestrea la superficie del controlador difuso de la práctica del motor."""
    sys.path.insert(0, os.path.join(AQUI, "..", "motor_cd_difuso", "fase_a"))
    from controlador import REGLAS
    from fuzzy import SistemaDifuso

    fis = SistemaDifuso(REGLAS, metodo)
    malla = np.linspace(-1, 1, n)
    E, DE = np.meshgrid(malla, malla)
    X = np.c_[E.ravel(), DE.ravel()]
    Y = np.array([[fis.evaluar(e, de)] for e, de in X])
    return X, Y, fis


def experimento_fuzzy(semilla=0, verbose=True):
    X, Y, fis = datos_fuzzy(21)  # 441 puntos de entrenamiento
    red = MLP([2, 12, 12, 1], salida="lineal", semilla=semilla)
    historial = red.entrenar(X, Y, tasa=0.05, epocas=6000, momento=0.9,
                             cada=1000 if verbose else None)
    # Prueba con puntos que la red NUNCA vio (al azar dentro del universo)
    rng = np.random.default_rng(123)
    Xp = rng.uniform(-1, 1, (500, 2))
    Yp = np.array([[fis.evaluar(e, de)] for e, de in Xp])
    rmse = float(np.sqrt(np.mean((red.predecir(Xp) - Yp) ** 2)))
    return red, historial, rmse, fis


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(CARPETA, exist_ok=True)

    # 1) XOR
    print("1) XOR con un MLP 2-4-1 (tanh en la oculta, sigmoide en la salida)")
    red, hist = experimento_xor()
    salidas = red.predecir(X_XOR).ravel()
    for x, s, d in zip(X_XOR, salidas, Y_XOR.ravel()):
        print(f"   x = {x.astype(int)}  ->  {s:.3f}  (redondeado {int(s >= 0.5)}, deseado {int(d)})")
    aciertos = (np.round(salidas) == Y_XOR.ravel()).mean() * 100
    print(f"   aciertos: {aciertos:.0f} %\n")

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.8))
    malla = np.linspace(-0.5, 1.5, 300)
    M1, M2 = np.meshgrid(malla, malla)
    Z = red.predecir(np.c_[M1.ravel(), M2.ravel()]).reshape(M1.shape)
    cf = a1.contourf(M1, M2, Z, levels=20, cmap="RdYlGn", vmin=0, vmax=1)
    a1.contour(M1, M2, Z, levels=[0.5], colors="k", linestyles="--")
    for clase, color in ((0, "#e5484d"), (1, "#30a46c")):
        m = Y_XOR.ravel() == clase
        a1.scatter(X_XOR[m, 0], X_XOR[m, 1], s=160, c=color, edgecolors="k", label=f"salida {clase}", zorder=3)
    fig.colorbar(cf, ax=a1, label="salida de la red")
    a1.set(title="XOR aprendido: la frontera ya no es una recta", xlabel="x1", ylabel="x2")
    a1.legend(loc="upper right")
    a2.semilogy(hist)
    a2.set(title="Pérdida (error cuadrático medio)", xlabel="Época", ylabel="Pérdida")
    a2.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(os.path.join(CARPETA, "mlp_xor.png"), dpi=120)

    # 2) Imitar al controlador difuso
    print("2) MLP 2-12-12-1 que imita al controlador difuso Mamdani de la práctica del motor")
    red, hist, rmse, fis = experimento_fuzzy()
    print(f"   error RMS en 500 puntos nuevos: {rmse:.4f}  (la salida va de -0.83 a 0.83)\n")

    malla = np.linspace(-1, 1, 41)
    E, DE = np.meshgrid(malla, malla)
    Zf = np.vectorize(fis.evaluar)(E, DE)
    Zr = red.predecir(np.c_[E.ravel(), DE.ravel()]).reshape(E.shape)
    fig = plt.figure(figsize=(15, 4.8))
    for i, (Z, titulo) in enumerate(((Zf, "Controlador difuso (Mamdani)"),
                                     (Zr, "Perceptrón multicapa (aprendido)"))):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        ax.plot_surface(E, DE, Z, cmap="viridis", edgecolor="none")
        ax.set(title=titulo, xlabel="e", ylabel="de", zlabel="du")
    ax = fig.add_subplot(1, 3, 3)
    im = ax.imshow(np.abs(Zr - Zf), extent=(-1, 1, -1, 1), origin="lower", cmap="magma")
    fig.colorbar(im, ax=ax, label="|error|")
    ax.set(title=f"Diferencia (RMS = {rmse:.3f})", xlabel="e", ylabel="de")
    fig.tight_layout()
    fig.savefig(os.path.join(CARPETA, "mlp_imita_fuzzy.png"), dpi=120)

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.semilogy(hist)
    ax.set(title="Pérdida al imitar al controlador difuso", xlabel="Época", ylabel="Pérdida")
    ax.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(os.path.join(CARPETA, "mlp_fuzzy_perdida.png"), dpi=120)
    print(f"Gráficas guardadas en {CARPETA}")


if __name__ == "__main__":
    main()
