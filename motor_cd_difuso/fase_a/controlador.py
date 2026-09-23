"""
Controlador difuso tipo PI incremental para velocidad del motor CD.

    e_k   = ref - rpm                (error)
    de_k  = e_k - e_(k-1)            (cambio del error en un periodo)
    du_k  = Kdu * FIS(Ke*e_k, Kde*de_k)
    u_k   = sat(u_(k-1) + du_k)      (voltaje al puente H)

Por qué incremental: el FIS entrega un *cambio* de voltaje; al acumularlo se
comporta como un PI, por lo que el error en estado estacionario tiende a 0.
Saturar u en cada paso evita el "windup" del acumulador.

Las mismas ganancias y la misma tabla de reglas están en el firmware del ESP32
(fase_b/esp32_fuzzy/esp32_fuzzy.ino).
"""
import numpy as np

from fuzzy import ETIQUETAS, SistemaDifuso

# Tabla de reglas (filas = error, columnas = cambio del error) -> du.
#   e > 0  : la velocidad está por DEBAJO de la referencia -> subir voltaje.
#   de > 0 : el error está creciendo -> empujar más.
# La diagonal Z es la "línea de equilibrio": cuando e y de se compensan
# (p. ej. estamos abajo pero subiendo rápido) no se cambia el voltaje.
TABLA_REGLAS = [
    #  de:  NG    NP    Z     PP    PG
    ["NG", "NG", "NG", "NP", "Z"],   # e = NG
    ["NG", "NG", "NP", "Z", "PP"],   # e = NP
    ["NG", "NP", "Z", "PP", "PG"],   # e = Z
    ["NP", "Z", "PP", "PG", "PG"],   # e = PP
    ["Z", "PP", "PG", "PG", "PG"],   # e = PG
]

REGLAS = {
    (et_e, et_de): TABLA_REGLAS[i][j]
    for i, et_e in enumerate(ETIQUETAS)
    for j, et_de in enumerate(ETIQUETAS)
}

# Ganancias de escala, sintonizadas con un barrido sobre el modelo por defecto.
# Criterio: mínimo IAE sin sobrepaso, también con J x2, y sin que el voltaje
# "vibre" demasiado cuando la medición tiene ruido (Kdu = 3 era más rápido
# pero amplificaba el ruido; Kdu = 2 era más lento).
# Se expresan relativas a las rpm máximas para poder llevarlas a otro motor
# (el firmware usa exactamente las mismas fórmulas).
RPM_MAX = 2274.0              # = MotorDC().rpm_estacionaria(12.0)
KE = 1.0 / (0.20 * RPM_MAX)   # ~455 rpm de error ya es "Positivo Grande"
KDE = 1.0 / (0.066 * RPM_MAX)  # ~150 rpm de cambio por periodo ya es "Grande"
KDU = 2.5                     # cambio máximo de voltaje por periodo [V]
TS = 0.01          # periodo de muestreo [s] (igual que en el ESP32)


class ControladorDifuso:
    def __init__(self, metodo="mamdani", ke=KE, kde=KDE, kdu=KDU, u_max=12.0):
        self.fis = SistemaDifuso(REGLAS, metodo=metodo)
        self.ke, self.kde, self.kdu = ke, kde, kdu
        self.u_max = u_max
        self.reiniciar()

    def reiniciar(self):
        self.e_prev = 0.0
        self.u = 0.0

    def calcular(self, ref, medida):
        e = ref - medida
        de = e - self.e_prev
        self.e_prev = e
        du = self.kdu * self.fis.evaluar(self.ke * e, self.kde * de)
        self.u = float(np.clip(self.u + du, -self.u_max, self.u_max))
        return self.u


class ControladorPI:
    """PI clásico discreto (con anti-windup) solo para comparar."""

    def __init__(self, kp=0.004, ki=0.12, ts=TS, u_max=12.0):
        self.kp, self.ki, self.ts, self.u_max = kp, ki, ts, u_max
        self.reiniciar()

    def reiniciar(self):
        self.integral = 0.0

    def calcular(self, ref, medida):
        e = ref - medida
        u_libre = self.kp * e + self.ki * (self.integral + e * self.ts)
        u = float(np.clip(u_libre, -self.u_max, self.u_max))
        if u == u_libre:  # solo integra si no está saturado (anti-windup)
            self.integral += e * self.ts
        return u
