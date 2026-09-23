"""
Pruebas automáticas. Ejecutar:  python -m unittest -v
"""
import unittest

import numpy as np

from controlador import REGLAS, RPM_MAX, ControladorDifuso
from fuzzy import SistemaDifuso, trimf
from motor_dc import MotorDC
from simular import metricas, simular


class TestPertenencia(unittest.TestCase):
    def test_triangulo(self):
        self.assertEqual(trimf(0.0, -0.5, 0.0, 0.5), 1.0)
        self.assertAlmostEqual(float(trimf(0.25, -0.5, 0.0, 0.5)), 0.5)
        self.assertEqual(trimf(0.6, -0.5, 0.0, 0.5), 0.0)

    def test_hombros_cubren_fuera_de_rango(self):
        self.assertEqual(trimf(-5.0, -1.0, -1.0, -0.5), 1.0)
        self.assertEqual(trimf(5.0, 0.5, 1.0, 1.0), 1.0)

    def test_particion_suma_uno(self):
        # Con triángulos al 50 % de traslape, la suma de grados siempre es 1.
        fis = SistemaDifuso(REGLAS)
        for x in np.linspace(-1, 1, 37):
            self.assertAlmostEqual(sum(fis.e.fuzzificar(x).values()), 1.0)


class TestInferencia(unittest.TestCase):
    def test_equilibrio_da_cero(self):
        for metodo in ("mamdani", "sugeno"):
            self.assertAlmostEqual(SistemaDifuso(REGLAS, metodo).evaluar(0, 0), 0.0)

    def test_simetria(self):
        # Reglas simétricas: f(-e, -de) = -f(e, de)
        for metodo in ("mamdani", "sugeno"):
            fis = SistemaDifuso(REGLAS, metodo)
            for e, de in [(0.3, -0.2), (0.8, 0.1), (-0.4, 0.9)]:
                self.assertAlmostEqual(fis.evaluar(e, de), -fis.evaluar(-e, -de), places=6)

    def test_monotono_en_error(self):
        fis = SistemaDifuso(REGLAS, "sugeno")
        salidas = [fis.evaluar(e, 0.0) for e in np.linspace(-1, 1, 21)]
        self.assertTrue(all(b >= a - 1e-9 for a, b in zip(salidas, salidas[1:])))

    def test_sugeno_a_mano(self):
        # e = 0.25 -> Z 0.5, PP 0.5 ; de = 0 -> Z 1
        # reglas: (Z,Z)->Z con w=0.5, (PP,Z)->PP con w=0.5
        # du = (0.5*0 + 0.5*0.5) / (0.5 + 0.5) = 0.25
        self.assertAlmostEqual(SistemaDifuso(REGLAS, "sugeno").evaluar(0.25, 0.0), 0.25)


class TestLazoCerrado(unittest.TestCase):
    def test_motor_estacionario(self):
        m = MotorDC()
        for _ in range(20000):
            m.paso(12.0, 1e-4)
        self.assertAlmostEqual(m.rpm, m.rpm_estacionaria(12.0), delta=1.0)
        self.assertAlmostEqual(RPM_MAX, m.rpm_estacionaria(12.0), delta=1.0)

    def test_sigue_referencia_sin_error(self):
        for metodo in ("mamdani", "sugeno"):
            t, r, y, u = simular(ControladorDifuso(metodo), MotorDC())
            met = metricas(t, r, y)
            self.assertLess(met["error_final_rpm"], 5.0)
            self.assertLess(met["sobrepaso_%"], 5.0)
            self.assertLess(met["t_establ_s"], 0.5)
            self.assertLess(abs(r[-1] - y[-1]), 5.0, "debe rechazar la carga")
            self.assertTrue(np.all(np.abs(u) <= 12.0))

    def test_robusto_a_inercia_doble(self):
        t, r, y, _ = simular(ControladorDifuso("mamdani"), MotorDC(J=1e-4))
        self.assertLess(metricas(t, r, y)["sobrepaso_%"], 5.0)


if __name__ == "__main__":
    unittest.main()
