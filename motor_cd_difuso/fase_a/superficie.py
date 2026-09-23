"""
Visualiza el sistema de inferencia difusa:
  - funciones de pertenencia (iguales para e, de y du)
  - superficie de control du = f(e, de) para Mamdani y Sugeno
  - un ejemplo de inferencia paso a paso impreso en consola

Uso: python superficie.py
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from controlador import REGLAS
from fuzzy import CONJUNTOS_ESTANDAR, SistemaDifuso, trimf

CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultados")


def ejemplo_paso_a_paso(e=0.3, de=-0.2):
    """Imprime cada etapa de la inferencia para un punto: útil para la defensa."""
    fis = SistemaDifuso(REGLAS, "mamdani")
    print(f"Entradas normalizadas: e = {e}, de = {de}\n")
    print("1) Fuzzificación")
    for nombre, var, x in (("e ", fis.e, e), ("de", fis.de, de)):
        mu = {k: round(v, 3) for k, v in var.fuzzificar(x).items() if v > 0}
        print(f"   {nombre}: {mu}")
    print("\n2) Reglas activas (AND = min)")
    for (et_e, et_de), et_out in REGLAS.items():
        w = min(fis.e.fuzzificar(e)[et_e], fis.de.fuzzificar(de)[et_de])
        if w > 0:
            print(f"   SI e es {et_e:<2} Y de es {et_de:<2} ENTONCES du es {et_out:<2}  -> w = {w:.3f}")
    print("\n3) Defuzzificación")
    print(f"   Mamdani (centroide):          du = {fis.evaluar(e, de):+.4f}")
    fis.metodo = "sugeno"
    print(f"   Sugeno (promedio ponderado):  du = {fis.evaluar(e, de):+.4f}")


def main():
    os.makedirs(CARPETA, exist_ok=True)

    # Funciones de pertenencia
    x = np.linspace(-1.2, 1.2, 500)
    fig, ax = plt.subplots(figsize=(8, 3))
    for et, abc in CONJUNTOS_ESTANDAR.items():
        ax.plot(x, trimf(x, *abc), label=et, lw=2)
    ax.set_title("Funciones de pertenencia (e, de y du normalizados)")
    ax.set_xlabel("Universo normalizado")
    ax.set_ylabel("μ")
    ax.legend(ncol=5, loc="upper center", bbox_to_anchor=(0.5, -0.25))
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(CARPETA, "funciones_pertenencia.png"), dpi=120)

    # Superficies de control
    malla = np.linspace(-1, 1, 41)
    E, DE = np.meshgrid(malla, malla)
    fig = plt.figure(figsize=(12, 5))
    for n, metodo in enumerate(("mamdani", "sugeno"), start=1):
        fis = SistemaDifuso(REGLAS, metodo)
        Z = np.vectorize(fis.evaluar)(E, DE)
        ax = fig.add_subplot(1, 2, n, projection="3d")
        ax.plot_surface(E, DE, Z, cmap="viridis", edgecolor="none")
        ax.set_title(f"Superficie de control — {metodo.capitalize()}")
        ax.set_xlabel("e")
        ax.set_ylabel("de")
        ax.set_zlabel("du")
    fig.tight_layout()
    fig.savefig(os.path.join(CARPETA, "superficies_control.png"), dpi=120)
    print(f"Figuras guardadas en {CARPETA}\n")

    ejemplo_paso_a_paso()


if __name__ == "__main__":
    main()
