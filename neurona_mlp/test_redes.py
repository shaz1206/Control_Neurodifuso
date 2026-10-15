"""Pruebas de la neurona y del perceptrón multicapa.   Uso:  python -m unittest -v"""
import unittest

import numpy as np

from mlp import MLP, X_XOR, Y_XOR, experimento_fuzzy, experimento_xor
from neurona import COMPUERTAS, Perceptron, X


class TestPerceptron(unittest.TestCase):
    def test_aprende_and_y_or(self):
        for nombre in ("AND", "OR"):
            neurona = Perceptron()
            historial = neurona.entrenar(X, COMPUERTAS[nombre])
            self.assertEqual(historial[-1], 0, nombre)
            np.testing.assert_array_equal(neurona.predecir(X), COMPUERTAS[nombre])

    def test_no_puede_con_xor(self):
        neurona = Perceptron()
        historial = neurona.entrenar(X, COMPUERTAS["XOR"], max_epocas=500)
        self.assertGreater(historial[-1], 0)  # nunca llega a 0 errores
        self.assertLess((neurona.predecir(X) == COMPUERTAS["XOR"]).mean(), 1.0)


class TestMLP(unittest.TestCase):
    def test_gradientes_coinciden_con_derivada_numerica(self):
        """La retropropagación debe dar lo mismo que derivar 'a mano' con diferencias finitas."""
        rng = np.random.default_rng(1)
        Xr, Yr = rng.normal(size=(6, 2)), rng.normal(size=(6, 1))
        for salida in ("sigmoide", "lineal"):
            red = MLP([2, 3, 3, 1], salida=salida, semilla=2)
            red.adelante(Xr)
            gW, _ = red.gradientes(Yr)
            h = 1e-6
            for k in range(len(red.W)):
                for idx in np.ndindex(red.W[k].shape):
                    original = red.W[k][idx]
                    red.W[k][idx] = original + h
                    red.adelante(Xr); mas = red.perdida(Yr)
                    red.W[k][idx] = original - h
                    red.adelante(Xr); menos = red.perdida(Yr)
                    red.W[k][idx] = original
                    self.assertAlmostEqual(gW[k][idx], (mas - menos) / (2 * h), places=6)

    def test_aprende_xor(self):
        for semilla in (0, 1, 2):
            red, historial = experimento_xor(semilla=semilla, verbose=False)
            np.testing.assert_array_equal(np.round(red.predecir(X_XOR)), Y_XOR)
            self.assertLess(historial[-1], historial[0] / 100)

    def test_imita_al_controlador_difuso(self):
        _, _, rmse, _ = experimento_fuzzy(verbose=False)
        self.assertLess(rmse, 0.07)


if __name__ == "__main__":
    unittest.main()
