/*
 * Lógica del controlador para la interfaz web: la misma de fase_a (Python),
 * traducida a JavaScript. No dibuja nada; index.html la usa y Node la prueba.
 *
 *   fis()            motor de inferencia Mamdani / Sugeno (como fuzzy.py)
 *   ControladorDifuso, ControladorPI   (como controlador.py)
 *   MotorDC          modelo físico con RK4 (como motor_dc.py)
 *   metricas()       sobrepaso, tiempos, error final, IAE (como simular.py)
 */
"use strict";

const ETIQUETAS = ["NG", "NP", "Z", "PP", "PG"];
const CENTROS = [-1, -0.5, 0, 0.5, 1];
// Tabla de reglas: REGLAS[e][de] = etiqueta de du (igual que en Python y en el ESP32)
const REGLAS = [
  [0, 0, 0, 1, 2],
  [0, 0, 1, 2, 3],
  [0, 1, 2, 3, 4],
  [1, 2, 3, 4, 4],
  [2, 3, 4, 4, 4],
];
const TS = 0.01;            // periodo de control [s]
const RPM_MAX_SIM = 2274;   // motor simulado de la Fase A (y del HIL)
const RPM_MAX_REAL = 113;   // GA25-370 medido con 12 V a través del L298N
const UNIVERSO = Array.from({ length: 201 }, (_, k) => -1 + k * 0.01);

// Triángulo de ancho 0.5 a cada lado; los extremos son hombros.
function pertenencia(x, j) {
  if (j === 0 && x <= CENTROS[0]) return 1;
  if (j === 4 && x >= CENTROS[4]) return 1;
  const mu = 1 - Math.abs(x - CENTROS[j]) / 0.5;
  return mu > 0 ? mu : 0;
}

const limitar = (x, a, b) => Math.min(Math.max(x, a), b);

/*
 * Inferencia completa con entradas normalizadas en [-1, 1]. Además del
 * resultado devuelve todos los pasos intermedios, para mostrarlos en vivo:
 *   muE, muDE   grados de pertenencia de e y de
 *   fuerzas     fuerza de cada una de las 25 reglas (AND = min)
 *   W           máxima fuerza por etiqueta de salida (unión de Mamdani)
 *   S           suma de fuerzas por etiqueta (pesos de Sugeno)
 *   agregado    conjunto de salida recortado y unido (solo Mamdani)
 */
function fis(e, de, mamdani) {
  e = limitar(e, -1, 1);
  de = limitar(de, -1, 1);
  const muE = [0, 1, 2, 3, 4].map((j) => pertenencia(e, j));
  const muDE = [0, 1, 2, 3, 4].map((j) => pertenencia(de, j));
  const fuerzas = REGLAS.map((fila, i) => fila.map((_, j) => Math.min(muE[i], muDE[j])));
  const W = [0, 0, 0, 0, 0];
  const S = [0, 0, 0, 0, 0];
  let num = 0;
  let den = 0;
  for (let i = 0; i < 5; i++) {
    for (let j = 0; j < 5; j++) {
      const w = fuerzas[i][j];
      if (w <= 0) continue;
      const o = REGLAS[i][j];
      num += w * CENTROS[o];
      den += w;
      W[o] = Math.max(W[o], w);
      S[o] += w;
    }
  }
  const base = { e, de, muE, muDE, fuerzas, W, S, agregado: null };
  if (den === 0) return { ...base, du: 0 };
  if (!mamdani) return { ...base, du: num / den };

  const agregado = UNIVERSO.map((x) => {
    let a = 0;
    for (let j = 0; j < 5; j++) a = Math.max(a, Math.min(W[j], pertenencia(x, j)));
    return a;
  });
  let sx = 0;
  let sa = 0;
  UNIVERSO.forEach((x, k) => { sx += x * agregado[k]; sa += agregado[k]; });
  return { ...base, agregado, du: sa > 0 ? sx / sa : 0 };
}

/*
 * Variable lingüística "velocidad" (para el panel "Velocidad en palabras").
 * No la usa el controlador: describe en palabras la velocidad medida y da la
 * referencia de cada botón. Centros en fracciones de las rpm máximas.
 */
const VELOCIDADES = ["Parado", "Lento", "Medio", "Rápido", "Muy rápido"];
const FRAC_VEL = [0, 0.22, 0.44, 0.66, 0.88];

function centrosVelocidad(rpmMax) {
  return FRAC_VEL.map((f) => f * rpmMax);
}

function pertenenciaVelocidad(v, j, rpmMax) {
  const c = centrosVelocidad(rpmMax);
  const ancho = 0.22 * rpmMax;
  if (j === 0 && v <= c[0]) return 1;
  if (j === 4 && v >= c[4]) return 1;
  const mu = 1 - Math.abs(v - c[j]) / ancho;
  return mu > 0 ? mu : 0;
}

function etiquetaDominante(mus) {
  let mejor = 0;
  for (let j = 1; j < 5; j++) if (mus[j] > mus[mejor]) mejor = j;
  return mejor;
}

// Controlador difuso PI incremental: u = sat(u + Kdu * FIS(Ke*e, Kde*de))
class ControladorDifuso {
  constructor(metodo, rpmMax = RPM_MAX_SIM) {
    this.mamdani = metodo === "mamdani";
    this.ke = 1 / (0.2 * rpmMax);
    this.kde = 1 / (0.066 * rpmMax);
    this.kdu = 2.5;
    this.u = 0;
    this.ePrev = 0;
  }
  calcular(ref, medida) {
    const e = ref - medida;
    const de = e - this.ePrev;
    this.ePrev = e;
    const r = fis(this.ke * e, this.kde * de, this.mamdani);
    this.u = limitar(this.u + this.kdu * r.du, -12, 12);
    return this.u;
  }
}

// PI clásico con anti-windup (solo integra si la salida no está saturada)
class ControladorPI {
  constructor(rpmMax = RPM_MAX_SIM) {
    this.kp = (0.004 * RPM_MAX_SIM) / rpmMax;
    this.ki = (0.12 * RPM_MAX_SIM) / rpmMax;
    this.integral = 0;
  }
  calcular(ref, medida) {
    const e = ref - medida;
    const uLibre = this.kp * e + this.ki * (this.integral + e * TS);
    const u = limitar(uLibre, -12, 12);
    if (u === uLibre) this.integral += e * TS;
    return u;
  }
}

function nuevoControlador(metodo, rpmMax = RPM_MAX_SIM) {
  return metodo === "pi" ? new ControladorPI(rpmMax) : new ControladorDifuso(metodo, rpmMax);
}

// Motor CD: L di/dt = V - R i - K w ;  J dw/dt = K i - b w - carga
class MotorDC {
  constructor() {
    Object.assign(this, { R: 2.0, L: 0.005, K: 0.05, J: 5e-5, b: 1e-5 });
    this.i = 0;
    this.w = 0;
  }
  derivadas(i, w, V, T) {
    return [(V - this.R * i - this.K * w) / this.L, (this.K * i - this.b * w - T) / this.J];
  }
  paso(V, dt, T = 0) {
    V = limitar(V, -12, 12);
    const { i, w } = this;
    const k1 = this.derivadas(i, w, V, T);
    const k2 = this.derivadas(i + (dt / 2) * k1[0], w + (dt / 2) * k1[1], V, T);
    const k3 = this.derivadas(i + (dt / 2) * k2[0], w + (dt / 2) * k2[1], V, T);
    const k4 = this.derivadas(i + dt * k3[0], w + dt * k3[1], V, T);
    this.i += (dt / 6) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
    this.w += (dt / 6) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
  }
  get rpm() { return (this.w * 60) / (2 * Math.PI); }
}

// Métricas del escalón que empieza en t0 (hasta t1), como simular.metricas
function metricas(t, r, y, t0, t1, banda = 0.02) {
  const idx = [];
  for (let k = 0; k < t.length; k++) if (t[k] >= t0 && t[k] < t1) idx.push(k);
  if (idx.length < 2) return null;
  const tt = idx.map((k) => t[k] - t0);
  const yy = idx.map((k) => y[k]);
  const ref = r[idx[0]];
  const inicial = yy[0];
  const salto = ref - inicial;
  const sobrepaso = salto > 0 ? Math.max(0, ((Math.max(...yy) - ref) / salto) * 100) : 0;
  const fuera = yy.map((v) => Math.abs(v - ref) > banda * Math.abs(salto));
  let ts;
  if (fuera[fuera.length - 1]) ts = NaN;
  else if (!fuera.some(Boolean)) ts = 0;
  else ts = tt[fuera.lastIndexOf(true) + 1];
  const primero = (cond) => { const k = yy.findIndex(cond); return k < 0 ? 0 : k; };
  const i10 = primero((v) => v >= inicial + 0.1 * salto);
  const i90 = primero((v) => v >= inicial + 0.9 * salto);
  const ultimos = yy.slice(-10);
  const errorFinal = Math.abs(ref - ultimos.reduce((a, b) => a + b, 0) / ultimos.length);
  const iae = idx.reduce((s, k) => s + Math.abs(r[k] - y[k]), 0) * TS;
  return { sobrepaso, tSubida: tt[i90] - tt[i10], tEstab: ts, errorFinal, iae };
}

if (typeof module !== "undefined") {
  module.exports = {
    ETIQUETAS, CENTROS, REGLAS, TS, RPM_MAX_SIM, RPM_MAX_REAL, pertenencia, fis,
    VELOCIDADES, centrosVelocidad, pertenenciaVelocidad,
    etiquetaDominante, ControladorDifuso, ControladorPI, nuevoControlador, MotorDC, metricas,
  };
}
