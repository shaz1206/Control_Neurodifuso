"""
Pruebas con el motor FÍSICO conectado al ESP32.

1) Identificar el motor (lazo abierto): aplica un escalón de voltaje y mide
   la ganancia (rpm/V) y la constante de tiempo tau.
       python prueba_motor.py identificar --puerto COM5 --voltaje 12

2) Lazo cerrado con el controlador difuso: dos escalones de referencia.
       python prueba_motor.py lazo --puerto COM5 --metodo mamdani --ref1 200 --ref2 120
   Durante el segundo escalón pueden frenar el eje con los dedos para
   mostrar el rechazo a perturbaciones.

Guarda CSV y gráfica en resultados/.
"""
import argparse
import csv
import os
import sys
import time

import numpy as np
import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fase_a"))
from simular import metricas  # noqa: E402

CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultados")


def leer_durante(ser, segundos, datos, eventos=()):
    """Lee líneas CSV del ESP32 durante `segundos`. `eventos` es una lista de
    (instante_s, comando) que se envían al llegar ese instante."""
    pendientes = sorted(eventos)
    inicio = time.time()
    while (ahora := time.time() - inicio) < segundos:
        while pendientes and ahora >= pendientes[0][0]:
            ser.write((pendientes.pop(0)[1] + "\n").encode())
        linea = ser.readline().decode(errors="ignore").strip()
        partes = linea.split(",")
        if linea.startswith("#") or len(partes) != 5:
            continue
        try:
            datos.append([float(p) for p in partes[:4]])
        except ValueError:
            pass  # línea cortada al abrir el puerto


def abrir(puerto, baud):
    ser = serial.Serial(puerto, baud, timeout=0.5)
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.write(b"p\n")
    return ser


def guardar(nombre, datos):
    os.makedirs(CARPETA, exist_ok=True)
    ruta = os.path.join(CARPETA, f"{nombre}.csv")
    with open(ruta, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "ref_rpm", "rpm", "u_V"])
        w.writerows(datos)
    return ruta


def graficar(nombre, t, r, y, u, titulo):
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax1.plot(t, r, "k--", label="Referencia")
    ax1.plot(t, y, label="Medida")
    ax1.set_ylabel("Velocidad [rpm]")
    ax1.set_title(titulo)
    ax1.legend()
    ax1.grid(alpha=0.3)
    ax2.step(t, u, where="post")
    ax2.set_ylabel("Voltaje [V]")
    ax2.set_xlabel("Tiempo [s]")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(CARPETA, f"{nombre}.png"), dpi=120)
    plt.show()


def identificar(args):
    datos = []
    with abrir(args.puerto, args.baud) as ser:
        leer_durante(ser, 3.5, datos, [(0.5, f"v{args.voltaje}"), (3.0, "p")])
    d = np.array(datos)
    t, y, u = (d[:, 0] - d[0, 0]) / 1000.0, d[:, 2], d[:, 3]
    guardar("identificacion", np.column_stack([t, d[:, 1], y, u]))

    en_escalon = u > 0.5 * args.voltaje
    t0 = t[np.argmax(en_escalon)]
    ventana = en_escalon & (t > t0 + 1.5)  # último segundo del escalón (ya estable)
    y_final = float(np.mean(y[ventana]))
    ganancia = y_final / args.voltaje
    i63 = np.argmax(en_escalon & (y >= 0.632 * y_final))
    tau = t[i63] - t0
    print(f"Velocidad final: {y_final:.1f} rpm con {args.voltaje} V")
    print(f"Ganancia: {ganancia:.2f} rpm/V    Constante de tiempo tau: {tau * 1000:.0f} ms")
    print(f"-> En el firmware ponga RPM_MAX = {ganancia * args.v_fuente:.0f}")
    print("   (mejor aún: mida directamente con --voltaje igual a la fuente)")
    if tau < 0.03:
        print("   tau es corta: el periodo de 10 ms es adecuado.")
    graficar("identificacion", t, d[:, 1], y, u, f"Escalón de {args.voltaje} V en lazo abierto")


def lazo(args):
    datos = []
    cmd = "m" if args.metodo == "mamdani" else "s"
    mitad = args.duracion / 2
    with abrir(args.puerto, args.baud) as ser:
        leer_durante(ser, args.duracion + 0.5, datos, [
            (0.2, cmd), (0.5, f"r{args.ref1}"), (0.5 + mitad, f"r{args.ref2}"),
            (args.duracion + 0.3, "p"),
        ])
    d = np.array(datos)
    t, r, y, u = (d[:, 0] - d[0, 0]) / 1000.0, d[:, 1], d[:, 2], d[:, 3]
    nombre = f"lazo_{args.metodo}"
    print(f"CSV: {guardar(nombre, np.column_stack([t, r, y, u]))}")

    t0 = t[np.argmax(r == args.ref1)]
    met = metricas(t, r, y, t0=t0, t1=t0 + mitad)
    print("Escalón 1: " + "  ".join(f"{k}={v:.3f}" for k, v in met.items()))
    graficar(nombre, t, r, y, u, f"Motor real — {args.metodo.capitalize()}")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="modo", required=True)
    for nombre in ("identificar", "lazo"):
        p = sub.add_parser(nombre)
        p.add_argument("--puerto", required=True)
        p.add_argument("--baud", type=int, default=115200)
    sub.choices["identificar"].add_argument("--voltaje", type=float, default=12.0)
    sub.choices["identificar"].add_argument("--v-fuente", type=float, default=12.0)
    sub.choices["lazo"].add_argument("--metodo", choices=["mamdani", "sugeno"], default="mamdani")
    sub.choices["lazo"].add_argument("--ref1", type=float, default=200.0)
    sub.choices["lazo"].add_argument("--ref2", type=float, default=120.0)
    sub.choices["lazo"].add_argument("--duracion", type=float, default=6.0)
    args = ap.parse_args()
    identificar(args) if args.modo == "identificar" else lazo(args)


if __name__ == "__main__":
    main()
