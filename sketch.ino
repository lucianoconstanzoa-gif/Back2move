#include <Wire.h>

// ---------- Pines ----------
const int MPU = 0x68;
const int PIN_NTC = 34;        // temperatura del alambre SMA
const int PIN_NTC_PIEL = 32;   // temperatura del lado de la piel
const int PIN_BAT = 35;        // voltaje de la bateria
const int BTN_PARARME = 18;
const int BTN_MANUAL = 19;
const int BTN_DETENER = 23;
const int PIN_TENS = 26;       // LED rojo
const int PIN_SMA = 27;        // LED azul
const int LED_ALERTA = 25;

// ---------- Parametros de uso ----------
const unsigned long T_ALERTA = 10000;
const float ANG_SENTADA = 20;
const float UMBRAL_PREP = 8;
const float ANG_DE_PIE = 60;
const unsigned long T_CONFIRMAR = 2000;
const unsigned long T_ACOMPANA = 8000;
const int MAX_ACT_DIA = 20;

// ---------- Limites de seguridad ----------
const unsigned long T_MAX_SMA = 4000;     // SMA nunca mas de 4 s seguidos
const unsigned long T_DESCANSO = 30000;   // descanso entre usos del SMA
const unsigned long T_MAX_TENS = 15000;   // TENS maximo por activacion
const float TEMP_MAX_SMA = 90;            // alambre: sobre esto se corta
const float TEMP_MAX_PIEL = 40;           // piel: sobre esto se bloquea el SMA
const float TEMP_OK_PIEL = 37;            // piel: bajo esto se desbloquea
const float BAT_BAJA = 3.5;               // bajo esto: sin SMA, solo TENS
const float BAT_CRITICA = 3.3;            // bajo esto: todo apagado

// ---------- Calibracion (se ajusta en la prueba de mesa) ----------
const float AJUSTE_ALAMBRE = 0;   // grados a sumar o restar al sensor del alambre
const float AJUSTE_PIEL = 0;      // grados a sumar o restar al sensor de piel
const float BETA = 3950;

enum Estado { DE_PIE, SENTADA, ALERTA, PREACTIVACION, ACOMPANA, MANUAL };
Estado estado = DE_PIE;

unsigned long tEstado = 0;
unsigned long tSmaOn = 0;
unsigned long tSmaOff = 0;
unsigned long tTensOn = 0;
bool smaOn = false;
bool tensOn = false;
bool pielCaliente = false;
float angBase = 0;
int activaciones = 0;
int nivelBat = -1;   // 0 = OK, 1 = baja, 2 = critica

int16_t leer16() {
  int hi = Wire.read();
  int lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

float leerAngulo() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6);
  float ax = leer16();
  float ay = leer16();
  float az = leer16();
  return atan2(sqrt(ax * ax + ay * ay), az) * 180.0 / PI;
}

float leerTemp(int pin, float ajuste) {
  int raw = analogRead(pin);
  if (raw <= 0 || raw >= 4095) return -99;   // sensor no conectado
  float t = 1 / (log(1 / (4095.0 / raw - 1)) / BETA + 1.0 / 298.15) - 273.15;
  return t + ajuste;
}

float leerBateria() {
  int raw = analogRead(PIN_BAT);
  // SIMULACION: la barrita va de 3,0 V (vacia) a 4,2 V (llena)
  // VIDA REAL:  return raw / 4095.0 * 3.3 * 2;   (divisor con 2 resistencias de 100k)
  return 3.0 + raw / 4095.0 * 1.2;
}

bool apretado(int pin) {
  return digitalRead(pin) == LOW;
}

void tens(bool on) {
  if (on && !tensOn) tTensOn = millis();
  tensOn = on;
  digitalWrite(PIN_TENS, on);
}

void sma(bool on) {
  if (on && !smaOn) tSmaOn = millis();
  if (!on && smaOn) tSmaOff = millis();   // empieza el descanso
  smaOn = on;
  digitalWrite(PIN_SMA, on);
}

void cambiar(Estado e, const char* texto) {
  estado = e;
  tEstado = millis();
  Serial.println(texto);
}

// Devuelve por que NO se puede usar el SMA ahora ("" = si se puede)
const char* motivoSinSma(float tempAlambre) {
  if (nivelBat != 0) return "bateria baja";
  if (pielCaliente) return "piel caliente";
  if (tempAlambre > TEMP_MAX_SMA) return "alambre caliente";
  if (tSmaOff > 0 && millis() - tSmaOff < T_DESCANSO) return "alambre en descanso";
  return "";
}

void activar(float tempAlambre) {
  if (nivelBat == 2) {
    Serial.println("Sin bateria: no se activa");
    delay(300);
    return;
  }
  if (activaciones >= MAX_ACT_DIA) {
    Serial.println("Limite diario alcanzado");
    delay(300);
    return;
  }
  tens(true);
  activaciones++;
  const char* motivo = motivoSinSma(tempAlambre);
  if (motivo[0] == '\0') {
    sma(true);
    cambiar(PREACTIVACION, "PRE-ACTIVACION: TENS suave + SMA tensa la banda");
  } else {
    cambiar(PREACTIVACION, "PRE-ACTIVACION: solo TENS");
    Serial.print("   motivo: ");
    Serial.println(motivo);
  }
}

void revisarBateria(float ang) {
  float v = leerBateria();
  int nivel = 0;
  if (v < BAT_CRITICA) nivel = 2;
  else if (v < BAT_BAJA) nivel = 1;
  if (nivel == nivelBat) return;

  nivelBat = nivel;
  Serial.print("Bateria: ");
  Serial.print(v, 2);
  if (nivel == 0) Serial.println(" V -> OK");
  if (nivel == 1) Serial.println(" V -> BAJA: SMA desactivado, solo TENS");
  if (nivel == 2) Serial.println(" V -> CRITICA: todo apagado, cargar la rodillera");

  if (nivel >= 1) sma(false);
  if (nivel == 2 && tensOn) {
    tens(false);
    if (ang < ANG_SENTADA) cambiar(SENTADA, "Apagado por bateria");
    else cambiar(DE_PIE, "Apagado por bateria");
  }
}

void revisarPiel(float tPiel) {
  if (!pielCaliente && tPiel > TEMP_MAX_PIEL) {
    pielCaliente = true;
    sma(false);
    Serial.print("Seguridad: piel a ");
    Serial.print(tPiel, 1);
    Serial.println(" C -> SMA bloqueado");
  }
  if (pielCaliente && tPiel > -50 && tPiel < TEMP_OK_PIEL) {
    pielCaliente = false;
    Serial.print("Piel OK (");
    Serial.print(tPiel, 1);
    Serial.println(" C) -> SMA disponible");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  pinMode(BTN_PARARME, INPUT_PULLUP);
  pinMode(BTN_MANUAL, INPUT_PULLUP);
  pinMode(BTN_DETENER, INPUT_PULLUP);
  pinMode(PIN_TENS, OUTPUT);
  pinMode(PIN_SMA, OUTPUT);
  pinMode(LED_ALERTA, OUTPUT);

  Serial.println("ElectroKine listo");
}

void loop() {
  unsigned long ahora = millis();
  float ang = leerAngulo();
  float temp = leerTemp(PIN_NTC, AJUSTE_ALAMBRE);
  float tPiel = leerTemp(PIN_NTC_PIEL, AJUSTE_PIEL);
  revisarBateria(ang);
  revisarPiel(tPiel);

  // Seguridad del SMA: temperatura del alambre y tiempo maximo
  if (smaOn && (temp > TEMP_MAX_SMA || ahora - tSmaOn > T_MAX_SMA)) {
    sma(false);
    Serial.println("Seguridad: SMA apagado");
  }

  // Seguridad del TENS: tiempo maximo
  if (tensOn && ahora - tTensOn > T_MAX_TENS) {
    tens(false);
    Serial.println("Seguridad: TENS apagado (tiempo maximo)");
  }

  // Boton DETENER
  if (apretado(BTN_DETENER)) {
    tens(false);
    sma(false);
    if (ang < ANG_SENTADA) cambiar(SENTADA, "DETENIDO por el usuario");
    else cambiar(DE_PIE, "DETENIDO por el usuario");
    delay(300);
    return;
  }

  // Boton MANUAL
  if (apretado(BTN_MANUAL) && (estado == DE_PIE || estado == SENTADA || estado == ALERTA)) {
    if (nivelBat == 2) {
      Serial.println("Sin bateria: no se activa");
    } else if (activaciones < MAX_ACT_DIA) {
      tens(true);
      activaciones++;
      cambiar(MANUAL, "MODO MANUAL: TENS activo");
    }
    delay(300);
  }

  switch (estado) {
    case DE_PIE:
      if (ang < ANG_SENTADA) cambiar(SENTADA, "SENTADA: contando tiempo...");
      break;

    case SENTADA:
      if (apretado(BTN_PARARME)) activar(temp);
      else if (ang >= ANG_DE_PIE) cambiar(DE_PIE, "Se paro antes de la alerta");
      else if (ahora - tEstado > T_ALERTA) {
        angBase = ang;
        cambiar(ALERTA, "ALERTA: mucho rato sentada");
      }
      break;

    case ALERTA:
      if (apretado(BTN_PARARME) || ang - angBase > UMBRAL_PREP) activar(temp);
      else if (ang < angBase) angBase = ang;
      break;

    case PREACTIVACION:
      if (ang >= ANG_DE_PIE) {
        cambiar(ACOMPANA, "CONFIRMADO: se paro. TENS acompana los primeros pasos");
      } else if (ahora - tEstado > T_CONFIRMAR) {
        tens(false);
        sma(false);
        angBase = ang;
        cambiar(ALERTA, "FALSA ALARMA: todo apagado");
      }
      break;

    case ACOMPANA:
      if (smaOn && ahora - tEstado > 1000) sma(false);
      if (ahora - tEstado > T_ACOMPANA) {
        tens(false);
        cambiar(DE_PIE, "MARCHA ESTABLE: TENS apagado");
      }
      break;

    case MANUAL:
      if (!tensOn) {
        if (ang < ANG_SENTADA) cambiar(SENTADA, "Fin del modo manual");
        else cambiar(DE_PIE, "Fin del modo manual");
      }
      break;
  }

  digitalWrite(LED_ALERTA, estado == ALERTA);
  delay(50);
}