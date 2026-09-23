"""
Motor de inferencia difusa escrito desde cero (sin librerías de lógica difusa).

Contiene:
  - trimf: función de pertenencia triangular (con "hombros" en los extremos).
  - VariableDifusa: un universo normalizado [-1, 1] con sus conjuntos.
  - SistemaDifuso: base de reglas 2 entradas -> 1 salida con inferencia
    Mamdani (min-max + centroide) o Sugeno de orden cero (promedio ponderado).

Todo trabaja en el universo normalizado [-1, 1]. Las ganancias de escala
(Ke, Kde, Kdu) viven en el controlador, no aquí.
"""
import numpy as np


def trimf(x, a, b, c):
    """Pertenencia triangular: vale 0 en a, 1 en b y 0 en c.

    Si a == b (o b == c) el triángulo se vuelve un "hombro": se queda en 1
    hacia la izquierda (o derecha). Así los conjuntos de los extremos
    cubren cualquier valor que se salga del universo.
    """
    x = np.asarray(x, dtype=float)
    subida = np.ones_like(x) if b == a else (x - a) / (b - a)
    bajada = np.ones_like(x) if c == b else (c - x) / (c - b)
    return np.clip(np.minimum(subida, bajada), 0.0, 1.0)


# Cinco etiquetas lingüísticas, igual para las tres variables.
ETIQUETAS = ["NG", "NP", "Z", "PP", "PG"]  # Negativo Grande ... Positivo Grande

# Triángulos equiespaciados en [-1, 1] con 50 % de traslape entre vecinos.
CONJUNTOS_ESTANDAR = {
    "NG": (-1.0, -1.0, -0.5),
    "NP": (-1.0, -0.5, 0.0),
    "Z": (-0.5, 0.0, 0.5),
    "PP": (0.0, 0.5, 1.0),
    "PG": (0.5, 1.0, 1.0),
}

# Para Sugeno de orden cero cada etiqueta de salida es un número (singleton).
# Se usa el pico de cada triángulo, así Mamdani y Sugeno son comparables.
SINGLETONS = {"NG": -1.0, "NP": -0.5, "Z": 0.0, "PP": 0.5, "PG": 1.0}


class VariableDifusa:
    def __init__(self, nombre, conjuntos=None):
        self.nombre = nombre
        self.conjuntos = conjuntos or CONJUNTOS_ESTANDAR

    def fuzzificar(self, x):
        """Número -> diccionario {etiqueta: grado de pertenencia}."""
        return {et: float(trimf(x, *abc)) for et, abc in self.conjuntos.items()}


class SistemaDifuso:
    """FIS de 2 entradas (e, de) y 1 salida (du).

    reglas: diccionario {(etiqueta_e, etiqueta_de): etiqueta_salida}
    metodo: "mamdani" o "sugeno"
    """

    def __init__(self, reglas, metodo="mamdani", puntos=201):
        if metodo not in ("mamdani", "sugeno"):
            raise ValueError("metodo debe ser 'mamdani' o 'sugeno'")
        self.e = VariableDifusa("error")
        self.de = VariableDifusa("cambio del error")
        self.salida = VariableDifusa("cambio de control")
        self.reglas = reglas
        self.metodo = metodo
        # Universo discretizado de la salida (solo lo usa Mamdani).
        self.universo = np.linspace(-1.0, 1.0, puntos)
        self._mf_salida = {
            et: trimf(self.universo, *abc) for et, abc in self.salida.conjuntos.items()
        }

    def activaciones(self, e, de):
        """Paso 1 y 2: fuzzificar y evaluar cada regla con el operador AND = min."""
        mu_e = self.e.fuzzificar(e)
        mu_de = self.de.fuzzificar(de)
        fuerzas = []
        for (et_e, et_de), et_out in self.reglas.items():
            w = min(mu_e[et_e], mu_de[et_de])
            if w > 0.0:
                fuerzas.append((w, et_out))
        return fuerzas

    def evaluar(self, e, de):
        """Entradas normalizadas en [-1, 1] -> salida normalizada en [-1, 1]."""
        e = float(np.clip(e, -1.0, 1.0))
        de = float(np.clip(de, -1.0, 1.0))
        fuerzas = self.activaciones(e, de)
        if not fuerzas:  # no pasa con conjuntos que cubren todo el universo
            return 0.0

        if self.metodo == "sugeno":
            # Promedio ponderado de los singletons: sum(w*z) / sum(w)
            num = sum(w * SINGLETONS[et] for w, et in fuerzas)
            den = sum(w for w, _ in fuerzas)
            return num / den

        # Mamdani: recortar (min) cada conjunto de salida a la fuerza de su
        # regla, unir todo con max y sacar el centroide del área resultante.
        agregado = np.zeros_like(self.universo)
        for w, et in fuerzas:
            agregado = np.maximum(agregado, np.minimum(w, self._mf_salida[et]))
        area = agregado.sum()
        if area == 0.0:
            return 0.0
        return float((self.universo * agregado).sum() / area)
