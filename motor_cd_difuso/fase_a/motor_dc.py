"""
Modelo físico de un motor CD de imán permanente.

Ecuaciones (estado x = [i, w]):
    Eléctrica:  L di/dt = V - R i - Ke w
    Mecánica:   J dw/dt = Kt i - b w - T_carga

Con unidades SI, Kt = Ke = K (constante del motor).
Los valores por defecto son de un motor pequeño de 12 V (~2200 rpm en vacío);
para el motor real se reemplazan con los valores identificados en la Fase B.
"""
import numpy as np

RAD_S_A_RPM = 60.0 / (2.0 * np.pi)


class MotorDC:
    def __init__(self, R=2.0, L=0.005, K=0.05, J=5e-5, b=1e-5, v_max=12.0):
        self.R, self.L, self.K, self.J, self.b = R, L, K, J, b
        self.v_max = v_max
        self.reiniciar()

    def reiniciar(self):
        self.i = 0.0  # corriente [A]
        self.w = 0.0  # velocidad angular [rad/s]

    def _derivadas(self, i, w, V, T_carga):
        di = (V - self.R * i - self.K * w) / self.L
        dw = (self.K * i - self.b * w - T_carga) / self.J
        return di, dw

    def paso(self, V, dt, T_carga=0.0):
        """Avanza dt segundos con Runge-Kutta de 4.º orden. Devuelve rpm."""
        V = float(np.clip(V, -self.v_max, self.v_max))  # el puente H satura
        i, w = self.i, self.w
        k1 = self._derivadas(i, w, V, T_carga)
        k2 = self._derivadas(i + dt / 2 * k1[0], w + dt / 2 * k1[1], V, T_carga)
        k3 = self._derivadas(i + dt / 2 * k2[0], w + dt / 2 * k2[1], V, T_carga)
        k4 = self._derivadas(i + dt * k3[0], w + dt * k3[1], V, T_carga)
        self.i += dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        self.w += dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        return self.rpm

    @property
    def rpm(self):
        return self.w * RAD_S_A_RPM

    def rpm_estacionaria(self, V):
        """Velocidad final con voltaje V constante y sin carga (dw/dt = di/dt = 0)."""
        w = V * self.K / (self.R * self.b + self.K ** 2)
        return w * RAD_S_A_RPM
