"""
Simulación en lazo cerrado: motor CD + controlador (Mamdani, Sugeno o PI).

Escenario de prueba (4 s):
  t = 0.0 s  referencia 0 -> 1500 rpm      (arranque)
  t = 1.5 s  referencia 1500 -> 800 rpm    (cambio de consigna)
  t = 2.5 s  se aplica un par de carga     (perturbación)

Uso:
  python simular.py                -> gráficas en resultados/ y tabla de métricas
  python simular.py --sin-graficas -> solo la tabla
"""
import argparse
import os

import numpy as np

from controlador import TS, ControladorDifuso, ControladorPI
from motor_dc import MotorDC

DT_PLANTA = 1e-4           # paso de integración del motor (100 veces menor que TS)
T_FINAL = 4.0
T_CARGA = 0.03             # par de carga [N·m] (~10 % del par de arranque)
CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultados")


def referencia(t):
    return 1500.0 if t < 1.5 else 800.0


def carga(t):
    return T_CARGA if t >= 2.5 else 0.0


def simular(ctrl, motor, ruido_rpm=0.0, semilla=0):
    """Corre el lazo cerrado. El controlador solo actúa cada TS segundos
    (como en el microcontrolador); entre muestras el voltaje se mantiene."""
    rng = np.random.default_rng(semilla)
    ctrl.reiniciar()
    motor.reiniciar()
    sub = int(round(TS / DT_PLANTA))
    n = int(round(T_FINAL / TS))
    t = np.arange(n) * TS
    r, y, u = np.zeros(n), np.zeros(n), np.zeros(n)
    for k in range(n):
        medida = motor.rpm + ruido_rpm * rng.standard_normal()
        r[k] = referencia(t[k])
        u[k] = ctrl.calcular(r[k], medida)
        y[k] = motor.rpm
        for _ in range(sub):
            motor.paso(u[k], DT_PLANTA, carga(t[k]))
    return t, r, y, u


def metricas(t, r, y, t0=0.0, t1=1.5, banda=0.02):
    """Métricas del escalón que empieza en t0 (hasta t1)."""
    m = (t >= t0) & (t < t1)
    tt, yy = t[m] - t0, y[m]
    ref = r[m][0]
    inicial = yy[0]
    salto = ref - inicial
    sobrepaso = max(0.0, (np.max(yy) - ref) / salto * 100.0) if salto > 0 else 0.0
    fuera = np.abs(yy - ref) > banda * abs(salto)
    if fuera[-1]:
        t_est = float("nan")  # nunca entró a la banda del 2 %
    elif not fuera.any():
        t_est = 0.0
    else:  # instante justo después de la última vez que estuvo fuera
        t_est = tt[np.where(fuera)[0][-1] + 1]
    # Tiempo de subida 10 %-90 %
    i10 = np.argmax(yy >= inicial + 0.1 * salto)
    i90 = np.argmax(yy >= inicial + 0.9 * salto)
    return {
        "sobrepaso_%": sobrepaso,
        "t_subida_s": tt[i90] - tt[i10],
        "t_establ_s": t_est,
        "error_final_rpm": abs(ref - np.mean(yy[-10:])),
        "IAE": float(np.sum(np.abs(r[m] - yy)) * TS),
    }


def caida_por_carga(t, r, y):
    """Máxima caída de velocidad tras la perturbación y error al final."""
    m = t >= 2.5
    return float(np.max(r[m] - y[m])), float(abs(r[-1] - np.mean(y[-10:])))


def controladores():
    return {
        "Mamdani": ControladorDifuso("mamdani"),
        "Sugeno": ControladorDifuso("sugeno"),
        "PI clásico": ControladorPI(),
    }


def imprimir_tabla(filas, titulo):
    print(f"\n{titulo}")
    cols = list(next(iter(filas.values())).keys())
    print(f"{'':<12}" + "".join(f"{c:>17}" for c in cols))
    for nombre, fila in filas.items():
        print(f"{nombre:<12}" + "".join(f"{v:>17.3f}" for v in fila.values()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sin-graficas", action="store_true")
    args = ap.parse_args()

    # 1) Motor nominal
    resultados, filas = {}, {}
    for nombre, ctrl in controladores().items():
        t, r, y, u = simular(ctrl, MotorDC())
        resultados[nombre] = (t, r, y, u)
        fila = metricas(t, r, y)
        fila["caida_carga_rpm"], fila["err_con_carga"] = caida_por_carga(t, r, y)
        filas[nombre] = fila
    imprimir_tabla(filas, "MOTOR NOMINAL (escalón 0 -> 1500 rpm + carga en t = 2.5 s)")

    # 2) Robustez: el controlador NO se vuelve a sintonizar
    casos = {
        "J x2": dict(J=1e-4),
        "R +50%": dict(R=3.0),
        "K -20%": dict(K=0.04),
        "ruido 15rpm": {},
    }
    robustez = {}
    for caso, params in casos.items():
        ruido = 15.0 if "ruido" in caso else 0.0
        fila = {}
        for nombre, ctrl in controladores().items():
            t, r, y, u = simular(ctrl, MotorDC(**params), ruido_rpm=ruido)
            met = metricas(t, r, y)
            fila[f"{nombre} SP%"] = met["sobrepaso_%"]
            fila[f"{nombre} ts"] = met["t_establ_s"]
        robustez[caso] = fila
    imprimir_tabla(robustez, "ROBUSTEZ (sobrepaso % y tiempo de establecimiento s)")

    if args.sin_graficas:
        return

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(CARPETA, exist_ok=True)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    t, r, _, _ = next(iter(resultados.values()))
    ax1.plot(t, r, "k--", label="Referencia")
    for nombre, (t, r, y, u) in resultados.items():
        ax1.plot(t, y, label=nombre)
        ax2.step(t, u, where="post", label=nombre)
    ax1.axvline(2.5, color="gray", alpha=0.4)
    ax1.text(2.52, 200, "carga", color="gray")
    ax1.set_ylabel("Velocidad [rpm]")
    ax1.set_title("Control de velocidad del motor CD")
    ax1.legend()
    ax1.grid(alpha=0.3)
    ax2.set_ylabel("Voltaje [V]")
    ax2.set_xlabel("Tiempo [s]")
    ax2.legend()
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    ruta = os.path.join(CARPETA, "respuesta_nominal.png")
    fig.savefig(ruta, dpi=120)
    print(f"\nGráfica guardada en {ruta}")

    # Robustez visual: Mamdani con variaciones de la planta
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(t, r, "k--", label="Referencia")
    for caso, params in {"Nominal": {}, **casos}.items():
        ruido = 15.0 if "ruido" in caso else 0.0
        t, r, y, _ = simular(ControladorDifuso("mamdani"), MotorDC(**params), ruido)
        ax.plot(t, y, label=caso)
    ax.set_title("Robustez del controlador Mamdani (sin re-sintonizar)")
    ax.set_xlabel("Tiempo [s]")
    ax.set_ylabel("Velocidad [rpm]")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    ruta = os.path.join(CARPETA, "robustez_mamdani.png")
    fig.savefig(ruta, dpi=120)
    print(f"Gráfica guardada en {ruta}")


if __name__ == "__main__":
    main()
