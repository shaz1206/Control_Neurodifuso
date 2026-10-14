"""
Abrir el puerto serie del ESP32 sin dejarlo atorado.

Al abrir un puerto, pyserial activa las líneas DTR y RTS. En la ESP32-S3
(chip USB CH343) esa combinación la reinicia en MODO DE DESCARGA
("waiting for download") y el programa nunca arranca. Por eso se abre con
DTR/RTS apagados y se da un pulso en RTS (= botón RST) para un arranque limpio.
"""
import time

import serial


def abrir_esp32(puerto, baud=115200, timeout=1.0):
    ser = serial.Serial()
    ser.port = puerto
    ser.baudrate = baud
    ser.timeout = timeout
    ser.dtr = False  # IO0 en alto: arranca el programa, no el modo descarga
    ser.rts = False
    ser.open()
    ser.rts = True   # EN en bajo: reinicio
    time.sleep(0.1)
    ser.rts = False  # EN en alto: arranca
    time.sleep(1.5)  # el ESP32 tarda ~1 s en arrancar
    ser.reset_input_buffer()
    return ser
