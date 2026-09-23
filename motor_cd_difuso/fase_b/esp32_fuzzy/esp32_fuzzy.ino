/*
 * Control difuso (Mamdani / Sugeno) de velocidad de un motor CD con ESP32.
 *
 * Es el mismo controlador de fase_a/controlador.py:
 *   e = ref - rpm,  de = e - e_anterior
 *   du = KDU * FIS(KE*e, KDE*de),   u = sat(u + du)   [V]
 *
 * Hardware: ESP32 + puente H (L298N / TB6612 / BTS7960) + motor CD.
 *   - SIN sensor (HAY_SENSOR = false): solo HIL. La PC simula la velocidad y el
 *     voltaje calculado también se aplica al motor real para verlo reaccionar.
 *   - CON encoder u optoacoplador (HAY_SENSOR = true): lazo cerrado real.
 *
 * Comandos por Serial (115200, terminar con Enter):
 *   r1500  -> referencia en rpm (negativo = giro inverso; sin sensor solo en HIL)
 *   m      -> inferencia Mamdani      s -> inferencia Sugeno
 *   v6.0   -> lazo ABIERTO: voltaje fijo (para identificar el motor)
 *   p      -> paro (ref = 0, u = 0)
 *   h      -> modo HIL on/off (la PC simula el motor, ver hil_planta.py)
 *   yNNN   -> (solo HIL) velocidad simulada que manda la PC; se responde "uV"
 *
 * Salida en modo normal (CSV, cada TS):  t_ms,ref,rpm,u,metodo
 */

// ======================= CONFIGURACIÓN (ajustar) =======================
const bool HAY_SENSOR = false;      // true solo si conectan un encoder/sensor
const bool MOTOR_SIGUE_HIL = true;  // en HIL, aplicar u al motor real (demo visual)

const int PIN_PWM = 25;    // ENA del L298N (o PWMA del TB6612)
const int PIN_IN1 = 26;    // IN1
const int PIN_IN2 = 27;    // IN2
const int PIN_ENC_A = 32;  // canal A del encoder (con interrupción)
const int PIN_ENC_B = 33;  // canal B (dirección). Si su sensor es de 1 canal,
                           // ponga ENCODER_UN_CANAL = true.
const bool ENCODER_UN_CANAL = false;
const bool INVERTIR_ENCODER = false;  // si con u > 0 las rpm salen negativas

// Pulsos flanco-de-subida del canal A por vuelta del EJE DE SALIDA
// = PPR del encoder x relación de engranes. Ej.: JGA25-370 -> 11 x 34 = 374.
const float PULSOS_POR_VUELTA = 374.0;

const float V_FUENTE = 12.0;   // voltaje de la fuente del puente H
// rpm en vacío a V_FUENTE. Para HIL = 2274 (las del motor simulado);
// con sensor, el valor medido con "prueba_motor.py identificar".
const float RPM_MAX = 2274.0;
const float ZONA_MUERTA_V = 0.0;  // voltaje mínimo al que el motor empieza a girar

const unsigned long TS_US = 10000;  // periodo de control: 10 ms
const float ALFA_FILTRO = 0.4;      // filtro de la velocidad (1 = sin filtro)

// Ganancias de escala. Se expresan relativas a RPM_MAX para que sirvan
// con cualquier motor: en simulación, 450 rpm y 150 rpm eran ~20 % y ~6.6 %
// de las 2270 rpm máximas del modelo.
const float KE = 1.0 / (0.20 * RPM_MAX);
const float KDE = 1.0 / (0.066 * RPM_MAX);
const float KDU = 2.5;  // [V por periodo]
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

// Triángulo de ancho 0.5 a cada lado; los extremos son hombros.
float pertenencia(float x, int j) {
  if (j == 0 && x <= CENTROS[0]) return 1.0;
  if (j == 4 && x >= CENTROS[4]) return 1.0;
  float mu = 1.0 - fabsf(x - CENTROS[j]) / 0.5;
  return mu > 0.0 ? mu : 0.0;
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
long cuentasPrevias = 0;
float rpm = 0.0;
float ref = 0.0;
float u = 0.0;
float ePrev = 0.0;
bool mamdani = true;
bool lazoAbierto = false;
bool modoHIL = false;
unsigned long tProximo = 0;
unsigned long tUltimoY = 0;  // último dato de la PC en HIL (para el watchdog)
String linea = "";

void IRAM_ATTR isrEncoder() {
  if (ENCODER_UN_CANAL) {
    cuentas++;  // sin canal B no se sabe el sentido
  } else {
    cuentas += digitalRead(PIN_ENC_B) ? 1 : -1;
  }
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
  float magnitud = ZONA_MUERTA_V + fabsf(v) * (V_FUENTE - ZONA_MUERTA_V) / V_FUENTE;
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
  float medida = (delta / PULSOS_POR_VUELTA) * (60.0e6 / TS_US);
  if (INVERTIR_ENCODER) medida = -medida;
  if (ENCODER_UN_CANAL && u < 0) medida = -medida;  // se asume el sentido de u
  return ALFA_FILTRO * medida + (1.0 - ALFA_FILTRO) * rpm;  // filtro pasa-bajas
}

float pasoControl(float medida) {
  float e = ref - medida;
  float de = e - ePrev;
  ePrev = e;
  float du = KDU * evaluarFIS(KE * e, KDE * de, mamdani);
  u = constrain(u + du, -V_FUENTE, V_FUENTE);  // anti-windup por saturación
  return u;
}

void reiniciarControl() {
  u = 0.0;
  ePrev = ref;  // evita un "salto" de de en el primer paso
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
    case 'm': mamdani = true; break;
    case 's': mamdani = false; break;
    case 'v': lazoAbierto = true; u = valor; break;
    case 'p': ref = 0; lazoAbierto = false; reiniciarControl(); break;
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
  if (HAY_SENSOR) attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncoder, RISING);
  aplicarVoltaje(0);
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
    if (!lazoAbierto && HAY_SENSOR) pasoControl(rpm);
    aplicarVoltaje(u);

    Serial.print(millis());   Serial.print(',');
    Serial.print(ref, 1);     Serial.print(',');
    Serial.print(rpm, 1);     Serial.print(',');
    Serial.print(u, 3);       Serial.print(',');
    Serial.println(lazoAbierto ? "abierto" : (mamdani ? "mamdani" : "sugeno"));
  }
}
