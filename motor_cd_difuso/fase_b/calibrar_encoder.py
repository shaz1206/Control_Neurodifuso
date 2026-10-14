"""
Calibra CUENTAS_POR_VUELTA del encoder contando vueltas del eje de salida.

El motor gira solo, despacio, y ustedes cuentan las vueltas con una marca:
1. Marquen el eje de salida (o la rueda) con plumón.
2. Corran:  python calibrar_encoder.py --puerto COM6
3. Presionen Enter justo cuando la marca pase por un punto fijo, cuenten
   10 vueltas y presionen Enter otra vez cuando la marca vuelva a pasar.

Con --a-mano el motor no se mueve y el eje se gira a mano (cuesta trabajo
por los engranes).

Al final dice qué número poner en CUENTAS_POR_VUELTA del firmware y si hay
que invertir el encoder.
"""
import argparse
import threading
import time

from puerto import abrir_esp32


def leer_cuentas(ser):
    ser.reset_input_buffer()
    ser.write(b"c\n")
    limite = time.time() + 1.0
    while time.time() < limite:
        linea = ser.readline().decode(errors="ignore").strip()
        if linea.startswith("# cuentas"):
            return int(linea.split()[2])
    raise TimeoutError("El ESP32 no respondió al comando 'c'. ¿Tiene el firmware con encoder?")


def esperar_enter_mostrando(ser, origen, esperado):
    """Muestra las cuentas en vivo hasta que se presione Enter; devuelve las cuentas."""
    listo = threading.Event()
    threading.Thread(target=lambda: (input(), listo.set()), daemon=True).start()
    while not listo.is_set():
        ahora = leer_cuentas(ser) - origen
        print(f"\r  cuentas: {ahora:+8d}   (~{ahora / esperado:+.2f} vueltas)   ",
              end="", flush=True)
        time.sleep(0.15)
    print()
    return leer_cuentas(ser)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", required=True)
    ap.add_argument("--vueltas", type=int, default=10)
    ap.add_argument("--voltaje", type=float, default=1.0,
                    help="voltaje para girar despacio (con la zona muerta, 1 V ~ 7 V reales)")
    ap.add_argument("--a-mano", action="store_true", help="no mover el motor; girar a mano")
    ap.add_argument("--esperado", type=float, default=1980.0,
                    help="valor actual de CUENTAS_POR_VUELTA en el firmware")
    args = ap.parse_args()

    ser = abrir_esp32(args.puerto)
    ser.write(b"p\n")
    ser.write(b"q\n")  # silenciar el CSV para leer solo las cuentas
    time.sleep(0.2)
    try:
        if not args.a_mano:
            ser.write(f"v{args.voltaje}\n".encode())
            print(f"El motor gira con v{args.voltaje}. Si no se mueve, ciérralo (Ctrl+C) y "
                  "prueba con --voltaje 2.\n")
        print("Presiona Enter justo cuando la marca pase por un punto fijo...")
        inicio = esperar_enter_mostrando(ser, leer_cuentas(ser), args.esperado)
        print(f"Cuenta {args.vueltas} vueltas y presiona Enter cuando la marca vuelva "
              "a pasar por el mismo punto.")
        fin = esperar_enter_mostrando(ser, inicio, args.esperado)
    finally:
        ser.write(b"p\n")  # detener el motor
        ser.write(b"q\n")  # reactivar el CSV
        ser.close()

    total = fin - inicio
    por_vuelta = abs(total) / args.vueltas
    print(f"\nTotal: {total} cuentas en {args.vueltas} vueltas -> {por_vuelta:.0f} por vuelta")
    if abs(total) < 20 * args.vueltas:  # una vuelta real son cientos de cuentas
        print("El encoder casi no contó: o el motor no giró, o el encoder no manda pulsos.\n"
              "Revisa amarillo -> GPIO15, verde -> GPIO16, azul -> 3V3 y negro -> GND.\n"
              "No cambies CUENTAS_POR_VUELTA con este resultado.")
        return
    print(f"Reducción de los engranes: 1:{por_vuelta / (11 * 4):.1f}")
    if abs(por_vuelta - args.esperado) / args.esperado < 0.03:
        print(f"Coincide con CUENTAS_POR_VUELTA = {args.esperado:.0f}. No hay que cambiar nada.")
    else:
        print(f"Pon CUENTAS_POR_VUELTA = {por_vuelta:.0f}.0 en el firmware y vuelve a subirlo.")
    if total < 0:
        print("Contó en negativo: pon INVERTIR_ENCODER = true en el firmware.")


if __name__ == "__main__":
    main()
