/*
 * Control difuso (Mamdani / Sugeno) de velocidad de un motor CD con ESP32.
 * Incluye un PI clásico (comando 'i') solo para comparar contra el difuso.
 *
 * Es el mismo controlador de fase_a/controlador.py:
 *   e = ref - rpm,  de = e - e_anterior
 *   du = KDU * FIS(KE*e, KDE*de),   u = sat(u + du)   [V]
 *
 * Hardware: ESP32-S3-WROOM-1 (DevKitC-1) + puente H L298N + motor GA25-370
 * (12 V, 130 rpm) con encoder Hall. En Arduino IDE: placa "ESP32S3 Dev Module".
 *
 * LED RGB integrado (comando l1 / l0):
 *   l1 (por defecto) VELOCIDAD medida: Parado apagado · Lento cian · Medio verde
 *                    · Rápido ámbar · Muy rápido rojo
 *   l0               ERROR: NG rojo · NP amarillo · Z verde · PP cian · PG azul
 *   - Lazo cerrado REAL: el encoder mide la velocidad y el FIS la corrige.
 *   - HIL (comando 'h'): la PC simula el motor; el ESP32 solo calcula el control.
 *
 * Comandos por Serial (115200, terminar con Enter):
 *   r100   -> referencia en rpm (negativo = giro inverso)
 *   m      -> inferencia Mamdani      s -> inferencia Sugeno
 *   i      -> PI clásico (para comparar; mismas ganancias que controlador.py)
 *   v6.0   -> lazo ABIERTO: voltaje fijo (para identificar el motor)
 *   p      -> paro (ref = 0, u = 0)
 *   h      -> modo HIL on/off (la PC simula el motor, ver hil_planta.py)
 *   yNNN   -> (solo HIL) velocidad simulada que manda la PC; se responde "uV"
 *   c      -> imprime las cuentas del encoder (para calibrar CUENTAS_POR_VUELTA)
 *   q      -> silencia / reactiva la salida CSV (para leer el Monitor Serie)
 *   d      -> diagnóstico del encoder: cambios y nivel de A y B en 0.5 s
 *   z4.5   -> cambia la zona muerta en vivo (para medirla; se pierde al reiniciar)
 *   l1/l0  -> el LED muestra la velocidad (l1) o el error (l0)
 *
 * Salida en modo normal (CSV, cada TS):  t_ms,ref,rpm,u,metodo
 */

// ======================= CONFIGURACIÓN (ajustar) =======================
const bool HAY_SENSOR = true;       // false = sin encoder (solo HIL)
const bool MOTOR_SIGUE_HIL = true;  // en HIL, aplicar u al motor real (demo visual)

// Pines de la ESP32-S3 (la S3 NO tiene GPIO 25-33: los usa la memoria flash).
// Se evitan los de arranque (0, 3, 45, 46), USB (19, 20) y UART (43, 44).
const int PIN_PWM = 4;     // ENA del L298N
const int PIN_IN1 = 5;     // IN1
const int PIN_IN2 = 6;     // IN2
const int PIN_ENC_A = 7;   // canal A del encoder (C1). En nuestra placa el GPIO15 se queda en bajo
const int PIN_ENC_B = 16;  // canal B del encoder (C2)

// LED RGB integrado (WS2812). En la DevKitC-1 es el GPIO48; en algunas
// placas v1.1 es el GPIO38: si no prende, cambie 48 por 38.
#ifdef RGB_BUILTIN
const int PIN_LED_RGB = RGB_BUILTIN;
#else
const int PIN_LED_RGB = 48;
#endif
const uint8_t BRILLO_LED = 40;  // 0-255 (a 255 deslumbra)
const bool INVERTIR_ENCODER = true;   // medido: con voltaje positivo contaba en negativo

// Cuentas por vuelta del EJE DE SALIDA leyendo A y B, subida y bajada (x4):
// 11 pulsos/vuelta del motor x 4 flancos x 45 de reducción = 1980.
// La reducción 1:45 es la típica del GA25-370 de 130 rpm: VERIFICAR con el
// comando 'c' (ver README) y corregir aquí si sale otro número.
const float CUENTAS_POR_VUELTA = 1980.0;

const float V_FUENTE = 12.0;   // voltaje de la fuente del puente H
// rpm en vacío a 12 V. Las ganancias se escalan con este número.
const float RPM_MAX_REAL = 113.0;  // medido: 113 rpm con 12 V (el L298N se come ~2 V)
const float RPM_MAX_HIL = 2274.0;  // motor simulado de la Fase A (no cambiar)
const float ZONA_MUERTA_V = 6.0;   // medido: la recta rpm-voltaje cruza el cero en ~6.3 V
float zonaMuerta = ZONA_MUERTA_V;  // se puede cambiar en vivo con el comando 'z'

const unsigned long TS_US = 10000;  // periodo de control: 10 ms
const float ALFA_FILTRO = 0.4;      // filtro de la velocidad (1 = sin filtro)

// Ganancias relativas a las rpm máximas, para que la misma sintonía sirva en
// el motor simulado (2274 rpm) y en el real (~120 rpm). En la simulación,
// 455 rpm de error y 150 rpm de cambio por periodo eran "grandes" = 20 % y 6.6 %.
const float E_GRANDE = 0.20;    // fracción de las rpm máximas que ya es error "grande"
const float DE_GRANDE = 0.066;  // fracción que ya es cambio "grande" por periodo
const float KDU = 2.5;          // [V por periodo]

// PI clásico de comparación: con 2274 rpm máximas da KP = 0.004 y KI = 0.12,
// los mismos de ControladorPI en controlador.py.
const float KP_REL = 0.004 * 2274.0;
const float KI_REL = 0.12 * 2274.0;
const float TS_S = TS_US / 1.0e6;  // periodo en segundos
// =======================================================================

const int PWM_FREQ = 20000;  // 20 kHz: fuera del rango audible
const int PWM_BITS = 10;     // resolución 0..1023
const int PWM_MAX = (1 << PWM_BITS) - 1;

// ------------------------- Sistema difuso -------------------------------
// Etiquetas: 0=NG 1=NP 2=Z 3=PP 4=PG. Centros de los triángulos:
const float CENTROS[5] = {-1.0, -0.5, 0.0, 0.5, 1.0};

// Tabla de reglas: REGLAS[e][de] = etiqueta de du (igual que en Python)
const uint8_t REGLAS[5][5] = {
  // de: NG NP Z  PP PG
  {0, 0, 0, 1, 2},  // e = NG
  {0, 0, 1, 2, 3},  // e = NP
  {0, 1, 2, 3, 4},  // e = Z
  {1, 2, 3, 4, 4},  // e = PP
  {2, 3, 4, 4, 4},  // e = PG
};

// Color de cada etiqueta (R, G, B): NG, NP, Z, PP, PG
const uint8_t COLORES[5][3] = {
  {255, 0, 0},    // NG rojo: muy por ENCIMA de la referencia
  {255, 160, 0},  // NP amarillo
  {0, 255, 0},    // Z  verde: en la referencia
  {0, 200, 255},  // PP cian
  {0, 0, 255},    // PG azul: muy por DEBAJO de la referencia
};

// Triángulo de ancho 0.5 a cada lado; los extremos son hombros.
float pertenencia(float x, int j) {
  if (j == 0 && x <= CENTROS[0]) return 1.0;
  if (j == 4 && x >= CENTROS[4]) return 1.0;
  float mu = 1.0 - fabsf(x - CENTROS[j]) / 0.5;
  return mu > 0.0 ? mu : 0.0;
}

// Etiqueta con mayor grado de pertenencia (0=NG ... 4=PG).
int etiquetaDominante(float x) {
  int mejor = 0;
  float muMejor = -1.0;
  for (int j = 0; j < 5; j++) {
    float mu = pertenencia(x, j);
    if (mu > muMejor) {
      muMejor = mu;
      mejor = j;
    }
  }
  return mejor;
}

// Qué muestra el LED: 'v' = etiqueta de la VELOCIDAD medida (por defecto),
// 'e' = etiqueta del ERROR. Se cambia con los comandos l1 (velocidad) y l0 (error).
char modoLED = 'v';
int etiquetaLED = -2;  // lo que se está mostrando (-1 = apagado; 10..14 = velocidad)

void escribirLED(uint8_t r, uint8_t g, uint8_t b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(PIN_LED_RGB, r, g, b);
#else
  neopixelWrite(PIN_LED_RGB, r, g, b);
#endif
}

// Escribe el color solo si cambió lo que se muestra (clave), para no saturar el LED.
void ponerColorLED(int clave, const uint8_t rgb[3]) {
  if (clave == etiquetaLED) return;
  etiquetaLED = clave;
  escribirLED(rgb[0] * BRILLO_LED / 255, rgb[1] * BRILLO_LED / 255, rgb[2] * BRILLO_LED / 255);
}

const uint8_t APAGADO[3] = {0, 0, 0};

// Modo error: color de la etiqueta del error (j = -1 apaga el LED).
void mostrarEtiqueta(int j) {
  if (modoLED != 'e') return;
  if (j < 0) ponerColorLED(-1, APAGADO);
  else ponerColorLED(j, COLORES[j]);
}

float evaluarFIS(float e, float de, bool mamdani) {
  e = constrain(e, -1.0f, 1.0f);
  de = constrain(de, -1.0f, 1.0f);

  float muE[5], muDE[5];
  for (int j = 0; j < 5; j++) {
    muE[j] = pertenencia(e, j);
    muDE[j] = pertenencia(de, j);
  }

  // Fuerza de cada regla = min(muE, muDE). Para cada etiqueta de salida se
  // guarda la máxima fuerza de las reglas que la producen (unión con max).
  float W[5] = {0, 0, 0, 0, 0};
  float num = 0.0, den = 0.0;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      float w = fminf(muE[i], muDE[j]);
      if (w <= 0.0) continue;
      uint8_t out = REGLAS[i][j];
      num += w * CENTROS[out];  // Sugeno de orden cero
      den += w;
      if (w > W[out]) W[out] = w;
    }
  }
  if (den == 0.0) return 0.0;
  if (!mamdani) return num / den;

  // Mamdani: centroide del conjunto agregado, muestreado en 201 puntos.
  num = 0.0;
  den = 0.0;
  for (int k = 0; k <= 200; k++) {
    float x = -1.0 + k * 0.01;
    float agregado = 0.0;
    for (int j = 0; j < 5; j++) {
      float recorte = fminf(W[j], pertenencia(x, j));
      if (recorte > agregado) agregado = recorte;
    }
    num += x * agregado;
    den += agregado;
  }
  return den > 0.0 ? num / den : 0.0;
}

// ------------------------- Estado ---------------------------------------
volatile long cuentas = 0;
volatile uint8_t estadoEnc = 0;  // último estado (A,B) leído por la interrupción
bool imprimirCSV = true;
int pasosSospechosos = 0;        // para la protección
long cuentasPrevias = 0;
float rpm = 0.0;
float ref = 0.0;
float u = 0.0;
float ePrev = 0.0;
float integral = 0.0;  // acumulador del PI
char metodo = 'm';     // 'm' Mamdani, 's' Sugeno, 'i' PI clásico
bool lazoAbierto = false;
bool modoHIL = false;
unsigned long tProximo = 0;
unsigned long tUltimoY = 0;  // último dato de la PC en HIL (para el watchdog)
String linea = "";

// Decodificador en cuadratura x4. Índice = (estado anterior << 2) | estado nuevo;
// +1/-1 según el sentido, 0 si no cambió o si se saltó un estado (ruido).
DRAM_ATTR static const int8_t TABLA_CUADRATURA[16] = {
  0, -1, 1, 0,  1, 0, 0, -1,  -1, 0, 0, 1,  0, 1, -1, 0};

void IRAM_ATTR isrEncoder() {
  uint8_t estado = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  cuentas += TABLA_CUADRATURA[(estadoEnc << 2) | estado];
  estadoEnc = estado;
}

float rpmMax() { return modoHIL ? RPM_MAX_HIL : RPM_MAX_REAL; }

// Variable lingüística "velocidad" (igual que en la interfaz web):
// Parado, Lento, Medio, Rápido, Muy rápido con centros en 0, 22, 44, 66 y 88 %
// de las rpm máximas y triángulos de ancho 22 %.
const float FRAC_VEL[5] = {0.0, 0.22, 0.44, 0.66, 0.88};
const uint8_t COLORES_VEL[5][3] = {
  {0, 0, 0},      // Parado: apagado
  {0, 180, 255},  // Lento: cian
  {0, 255, 0},    // Medio: verde
  {255, 140, 0},  // Rápido: ámbar
  {255, 0, 0},    // Muy rápido: rojo
};

float pertenenciaVel(float v, int j) {
  float c = FRAC_VEL[j] * rpmMax();
  if (j == 0 && v <= c) return 1.0;
  if (j == 4 && v >= c) return 1.0;
  float mu = 1.0 - fabsf(v - c) / (0.22 * rpmMax());
  return mu > 0.0 ? mu : 0.0;
}

// Modo velocidad: color de la etiqueta con mayor pertenencia de la velocidad medida.
void mostrarVelocidad(float v) {
  if (modoLED != 'v') return;
  v = fabsf(v);
  int mejor = 0;
  for (int j = 1; j < 5; j++)
    if (pertenenciaVel(v, j) > pertenenciaVel(v, mejor)) mejor = j;
  ponerColorLED(10 + mejor, COLORES_VEL[mejor]);
}

void escribirPWM(int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PWM, duty);
#else
  ledcWrite(0, duty);
#endif
}

// Voltaje con signo -> dirección (IN1/IN2) + ciclo de trabajo (PWM).
void aplicarVoltaje(float v) {
  v = constrain(v, -V_FUENTE, V_FUENTE);
  if (fabsf(v) < 0.05) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    escribirPWM(0);
    return;
  }
  // Compensación de zona muerta: el motor no gira por debajo de ZONA_MUERTA_V
  float magnitud = zonaMuerta + fabsf(v) * (V_FUENTE - zonaMuerta) / V_FUENTE;
  digitalWrite(PIN_IN1, v > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, v > 0 ? LOW : HIGH);
  escribirPWM((int)(magnitud / V_FUENTE * PWM_MAX));
}

float leerRPM() {
  noInterrupts();
  long c = cuentas;
  interrupts();
  long delta = c - cuentasPrevias;
  cuentasPrevias = c;
  float medida = (delta / CUENTAS_POR_VUELTA) * (60.0e6 / TS_US);
  if (INVERTIR_ENCODER) medida = -medida;
  return ALFA_FILTRO * medida + (1.0 - ALFA_FILTRO) * rpm;  // filtro pasa-bajas
}

float pasoControl(float medida) {
  float e = ref - medida;
  float de = e - ePrev;
  ePrev = e;
  // LED: etiqueta dominante del error normalizado (apagado si está parado)
  if (ref == 0.0 && fabsf(medida) < 1.0) {
    mostrarEtiqueta(-1);
  } else {
    mostrarEtiqueta(etiquetaDominante(e / (E_GRANDE * rpmMax())));
  }
  // Límites del voltaje. En el motor real no se deja que u cambie de signo
  // respecto a la referencia: un voltaje "al revés" + la compensación de zona
  // muerta es un frenazo de ~6 V que hunde la velocidad. Para bajar, el motor
  // queda libre y lo frena su propia fricción. (En HIL se deja igual que en Python.)
  float uMin = -V_FUENTE, uMax = V_FUENTE;
  if (!modoHIL && ref > 0.0) uMin = 0.0;
  if (!modoHIL && ref < 0.0) uMax = 0.0;

  if (metodo == 'i') {
    // PI con anti-windup: solo integra si la salida no está saturada
    float uLibre = (KP_REL / rpmMax()) * e + (KI_REL / rpmMax()) * (integral + e * TS_S);
    u = constrain(uLibre, uMin, uMax);
    if (u == uLibre) integral += e * TS_S;
    return u;
  }
  float ke = 1.0 / (E_GRANDE * rpmMax());
  float kde = 1.0 / (DE_GRANDE * rpmMax());
  float du = KDU * evaluarFIS(ke * e, kde * de, metodo == 'm');
  u = constrain(u + du, uMin, uMax);  // anti-windup por saturación
  return u;
}

void reiniciarControl() {
  u = 0.0;
  ePrev = ref;  // evita un "salto" de de en el primer paso
  integral = 0.0;
  pasosSospechosos = 0;
}

// Protección en lazo cerrado real: si el voltaje está al máximo y el motor no
// gira, o el encoder cuenta al revés, el controlador se iría a 12 V sin parar.
void revisarProteccion() {
  bool sinGiro = fabsf(u) > 0.9 * V_FUENTE && fabsf(rpm) < 2.0;
  bool alReves = (u > 3.0 && rpm < -5.0) || (u < -3.0 && rpm > 5.0);
  pasosSospechosos = (sinGiro || alReves) ? pasosSospechosos + 1 : 0;
  if (pasosSospechosos > 100) {  // 1 s seguido
    Serial.println(alReves
        ? "# PROTECCION: el encoder cuenta al reves. Ponga INVERTIR_ENCODER = true"
        : "# PROTECCION: voltaje al maximo y el motor no gira. Revise motor y encoder");
    ref = 0;
    reiniciarControl();
  }
}

void procesarComando(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);
  float valor = cmd.substring(1).toFloat();
  switch (c) {
    case 'r':
      if (!HAY_SENSOR && !modoHIL) {
        Serial.println("# Sin sensor no hay lazo cerrado real: use HIL (h) o lazo abierto (v)");
        break;
      }
      ref = valor;
      lazoAbierto = false;
      break;
    case 'm':
    case 's':
    case 'i': metodo = c; break;
    case 'v': lazoAbierto = true; u = valor; mostrarEtiqueta(-1); break;
    case 'p': ref = 0; lazoAbierto = false; reiniciarControl(); break;
    case 'c': {
      noInterrupts();
      long c0 = cuentas;
      interrupts();
      if (INVERTIR_ENCODER) c0 = -c0;
      Serial.print("# cuentas ");
      Serial.print(c0);
      Serial.print("  (= ");
      Serial.print(c0 / CUENTAS_POR_VUELTA, 2);
      Serial.println(" vueltas segun CUENTAS_POR_VUELTA)");
      break;
    }
    case 'd': {
      // Diagnóstico del encoder: lee A y B directo durante 0.5 s (sin interrupciones)
      long cambiosA = 0, cambiosB = 0, altosA = 0, altosB = 0, n = 0;
      int a0 = digitalRead(PIN_ENC_A), b0 = digitalRead(PIN_ENC_B);
      unsigned long fin = millis() + 500;
      while (millis() < fin) {
        int a = digitalRead(PIN_ENC_A), b = digitalRead(PIN_ENC_B);
        cambiosA += (a != a0);
        cambiosB += (b != b0);
        altosA += a;
        altosB += b;
        a0 = a;
        b0 = b;
        n++;
      }
      Serial.printf("# diag A: cambios=%ld alto=%ld%% | B: cambios=%ld alto=%ld%%\n",
                    cambiosA, 100 * altosA / n, cambiosB, 100 * altosB / n);
      break;
    }
    case 'z':  // zona muerta en vivo (para medirla); se pierde al reiniciar
      zonaMuerta = constrain(valor, 0.0f, V_FUENTE - 1.0f);
      Serial.print("# zona muerta = ");
      Serial.println(zonaMuerta, 2);
      break;
    case 'l':  // l1 = LED muestra la velocidad, l0 = LED muestra el error
      modoLED = (valor >= 0.5) ? 'v' : 'e';
      etiquetaLED = -2;  // forzar que se repinte
      ponerColorLED(-1, APAGADO);
      Serial.println(modoLED == 'v' ? "# LED: velocidad" : "# LED: error");
      break;
    case 'q':
      imprimirCSV = !imprimirCSV;
      Serial.println(imprimirCSV ? "# CSV ON" : "# CSV OFF");
      break;
    case 'h':
      modoHIL = !modoHIL;
      ref = 0;
      reiniciarControl();
      aplicarVoltaje(0);
      Serial.println(modoHIL ? "# HIL ON" : "# HIL OFF");
      break;
    case 'y':  // HIL: la PC manda la velocidad simulada y espera u
      if (modoHIL) {
        pasoControl(valor);
        mostrarVelocidad(valor);  // en HIL, la velocidad simulada
        if (MOTOR_SIGUE_HIL) aplicarVoltaje(u);
        tUltimoY = millis();
        Serial.print('u');
        Serial.println(u, 4);
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_PWM, 0);
#endif
  if (HAY_SENSOR) {
    estadoEnc = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncoder, CHANGE);
  }
  aplicarVoltaje(0);
  ponerColorLED(-1, APAGADO);
  Serial.println("# t_ms,ref,rpm,u,metodo");
  tProximo = micros();
}

void loop() {
  // Leer comandos sin bloquear
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      procesarComando(linea);
      linea = "";
    } else if (ch != '\r') {
      linea += ch;
    }
  }

  if (modoHIL) {
    // En HIL el ritmo lo marca la PC con los comandos 'y'. Si la PC deja de
    // mandar datos (script cerrado o error), se apaga el motor por seguridad.
    if (millis() - tUltimoY > 300) aplicarVoltaje(0);
    return;
  }

  // Lazo de control a periodo fijo
  if ((long)(micros() - tProximo) >= 0) {
    tProximo += TS_US;
    rpm = HAY_SENSOR ? leerRPM() : 0.0;
    mostrarVelocidad(rpm);
    if (!lazoAbierto && HAY_SENSOR) {
      if (ref == 0.0) {
        // Parado: motor libre. No se controla alrededor de 0 rpm, porque con la
        // zona muerta eso daría patadas de +-6 V. Se reinicia para el próximo arranque.
        u = 0.0;
        ePrev = 0.0;
        integral = 0.0;
        pasosSospechosos = 0;
        mostrarEtiqueta(-1);
      } else {
        pasoControl(rpm);
        revisarProteccion();
      }
    }
    aplicarVoltaje(u);

    if (!imprimirCSV) return;
    Serial.print(millis());   Serial.print(',');
    Serial.print(ref, 1);     Serial.print(',');
    Serial.print(rpm, 1);     Serial.print(',');
    Serial.print(u, 3);       Serial.print(',');
    Serial.println(lazoAbierto ? "abierto"
                   : (metodo == 'm' ? "mamdani" : (metodo == 's' ? "sugeno" : "pi")));
  }
}
