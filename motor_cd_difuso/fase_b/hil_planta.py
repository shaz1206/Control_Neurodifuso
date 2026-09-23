"""
Hardware-in-the-Loop: el CONTROLADOR corre en el ESP32 y el MOTOR se simula
en la PC con el mismo modelo de la Fase A. En cada periodo:

    PC  --"y<rpm>"-->  ESP32   (velocidad simulada)
    PC  <--"u<V>"---   ESP32   (voltaje que calculó el FIS embebido)

Al final compara contra la simulación 100 % Python: si las curvas coinciden,
el FIS del ESP32 está bien implementado.

Si en el firmware MOTOR_SIGUE_HIL = true, el voltaje también se aplica al
motor real: se ve arrancar, bajar de velocidad y "empujar" cuando entra la
carga simulada en t = 2.5 s. Por eso corre a tiempo real (10 ms por paso).

El firmware debe tener RPM_MAX = 2274.0 (las rpm máximas del modelo simulado).

Uso:  python hil_planta.py --puerto COM7 --metodo mamdani
      python hil_planta.py --puerto COM7 --metodo sugeno --rapido   (sin esperar)
"""
import argparse
import csv
import os
import sys
import time

import numpy as np
import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fase_a"))
from controlador import TS, ControladorDifuso, ControladorPI  # noqa: E402
from motor_dc import MotorDC  # noqa: E402
from simular import DT_PLANTA, T_FINAL, carga, metricas, referencia, simular  # noqa: E402

CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultados")


def esperar_linea(ser, prefijo, timeout=3.0):
    limite = time.time() + timeout
    while time.time() < limite:
        linea = ser.readline().decode(errors="ignore").strip()
        if linea.startswith(prefijo):
            return linea
    raise TimeoutError(f"El ESP32 no respondió con '{prefijo}'")


def correr_hil(ser, metodo, tiempo_real=True):
    ser.write(b"h\n")
    esperar_linea(ser, "# HIL ON")
    ser.write({"mamdani": b"m\n", "sugeno": b"s\n", "pi": b"i\n"}[metodo])

    motor = MotorDC()
    sub = int(round(TS / DT_PLANTA))
    n = int(round(T_FINAL / TS))
    t = np.arange(n) * TS
    r, y, u = np.zeros(n), np.zeros(n), np.zeros(n)
    ref_enviada = None
    inicio = time.perf_counter()
    try:
        for k in range(n):
            if tiempo_real:  # esperar al instante k*TS para ir al ritmo del motor real
                while time.perf_counter() - inicio < t[k]:
                    pass
            r[k] = referencia(t[k])
            if r[k] != ref_enviada:
                ser.write(f"r{r[k]:.1f}\n".encode())
                ref_enviada = r[k]
            y[k] = motor.rpm
            ser.write(f"y{y[k]:.3f}\n".encode())
            u[k] = float(esperar_linea(ser, "u")[1:])
            for _ in range(sub):
                motor.paso(u[k], DT_PLANTA, carga(t[k]))
    finally:
        ser.write(b"h\n")  # salir de HIL aunque algo falle
    return t, r, y, u


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", required=True, help="COM5, /dev/ttyUSB0, ...")
    ap.add_argument("--metodo", choices=["mamdani", "sugeno", "pi"], default="mamdani")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rapido", action="store_true",
                    help="no esperar a tiempo real (sin demo visual del motor)")
    args = ap.parse_args()

    with serial.Serial(args.puerto, args.baud, timeout=1) as ser:
        time.sleep(2.0)  # el ESP32 se reinicia al abrir el puerto
        ser.reset_input_buffer()
        inicio = time.time()
        t, r, y, u = correr_hil(ser, args.metodo, tiempo_real=not args.rapido)
        print(f"HIL terminado en {time.time() - inicio:.1f} s ({len(t)} pasos)")

    ctrl = ControladorPI() if args.metodo == "pi" else ControladorDifuso(args.metodo)
    ts, rs, ys, us = simular(ctrl, MotorDC())
    print(f"Diferencia máx. HIL vs simulación: {np.max(np.abs(y - ys)):.2f} rpm, "
          f"{np.max(np.abs(u - us)):.4f} V")
    for nombre, yy in (("HIL", y), ("Simulación", ys)):
        met = metricas(t, r, yy)
        print(f"{nombre:<11} " + "  ".join(f"{k}={v:.3f}" for k, v in met.items()))

    os.makedirs(CARPETA, exist_ok=True)
    ruta_csv = os.path.join(CARPETA, f"hil_{args.metodo}.csv")
    with open(ruta_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "ref_rpm", "rpm_hil", "u_hil", "rpm_sim", "u_sim"])
        w.writerows(zip(t, r, y, u, ys, us))

    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax1.plot(t, r, "k--", label="Referencia")
    ax1.plot(t, y, label="HIL (FIS en ESP32)", lw=2)
    ax1.plot(ts, ys, ":", label="Simulación Python", lw=2)
    ax1.set_ylabel("Velocidad [rpm]")
    ax1.set_title(f"HIL — {args.metodo.capitalize()}")
    ax1.legend()
    ax1.grid(alpha=0.3)
    ax2.step(t, u, where="post", label="HIL")
    ax2.step(ts, us, ":", where="post", label="Simulación")
    ax2.set_ylabel("Voltaje [V]")
    ax2.set_xlabel("Tiempo [s]")
    ax2.legend()
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(CARPETA, f"hil_{args.metodo}.png"), dpi=120)
    print(f"Resultados en {CARPETA}")
    plt.show()


if __name__ == "__main__":
    main()
