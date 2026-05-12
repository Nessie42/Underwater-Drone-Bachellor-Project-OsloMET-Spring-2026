#include <Wire.h>
#include <MS5837.h>

// =========================
// Pinout
// =========================
static const int PIN_I2C_SDA   = 0;   // GP0
static const int PIN_I2C_SCL   = 1;   // GP1

static const int PIN_PWM       = 2;   // GP2
static const int PIN_INA       = 3;   // GP3
static const int PIN_INB       = 4;   // GP4

static const int PIN_RELAY_IN  = 16;  // GP16 (korrigert fra skjema)

// =========================
// Fysiske parametre
// =========================
const float rho = 1000.0f;     // kg/m^3, ferskvann
const float g   = 9.81f;       // m/s^2
const float dt  = 0.1f;        // s

const float Cd  = 1.2f;
const float A   = 0.012f;      // m^2

// Skrog / ballast
const float V_disp          = 0.0038f;   // m^3
const float m_skrog         = 3.529f;     // kg
const float V_ballast_maks  = 0.00095f;  // m^3

// Krav: bare halvfull tank for å unngå mottrykk
const float BALLAST_FILL_FRACTION = 0.5f;
const float m_ballast_maks = rho * V_ballast_maks * BALLAST_FILL_FRACTION;

// Målt flowrate
const float Q_maalt = 0.0012f / 1000.0f;   // 0.0006 L/s -> m^3/s

// =========================
// Regulering
// =========================
float ønsket_dybde = 10.0f;   // m
float h_ref        = 0.0f;

const float Kp = 0.4f;
const float Kd = 20.0f;
const float Ki = 0.0004f;
const float e_deadzone = 0.1f;

// =========================
// Tilstand
// =========================
float h = 0.0f;            // dybde [m]
float v = 0.0f;            // vertikal hastighet [m/s]
float integrator = 0.0f;
float m_ballast  = 0.0f;
float Q          = 0.0f;
bool started = false;
bool remotePrev = false;

// PWM-ramping / myk start
float pwm_actual = 0.0f;   // Faktisk PWM som sendes til motoren etter ramping
const float PWM_RAMP_STEP = 0.05f;  // Maks PWM-endring per loop. 0.05 ved dt=0.1s gir ca. 2 sek fra 0 til 100%

// Sensor
MS5837 sensor;
bool sensorOK = false; //Sensor status
float surfacePressure_mbar = 1013.25f;  // kalibreres ved oppstart

// Tidsstyring
unsigned long lastStepMs = 0;
unsigned long startMs    = 0;

// ======================
// Hjelpefunksjoner
// ======================

// Myk ramping av PWM.
// PID-en bestemmer ønsket PWM, men denne funksjonen gjør at faktisk PWM
// endres gradvis for å unngå rykk, høy startstrøm og trykkstøt.
float rampPWM(float pwm_cmd) {
  pwm_cmd = constrain(pwm_cmd, 0.0f, 1.0f);

  if (pwm_actual < pwm_cmd) {
    pwm_actual = min(pwm_actual + PWM_RAMP_STEP, pwm_cmd);
  } else {
    pwm_actual = max(pwm_actual - PWM_RAMP_STEP, pwm_cmd);
  }

  return pwm_actual;
}

// Sett motorretning og hastighet
// dir = +1 => fyll ballasttank
// dir = -1 => tøm ballasttank
// pwm01 i [0..1]
void drivePump(int dir, float pwm01) {
  int pwm = (int)(255.0f * pwm01);

  if (dir > 0) {
    digitalWrite(PIN_INA, HIGH);
    digitalWrite(PIN_INB, LOW);
    analogWrite(PIN_PWM, pwm);
  } else if (dir < 0) {
    digitalWrite(PIN_INA, LOW);
    digitalWrite(PIN_INB, HIGH);
    analogWrite(PIN_PWM, pwm);
  } else {
    digitalWrite(PIN_INA, LOW);
    digitalWrite(PIN_INB, LOW);
    analogWrite(PIN_PWM, 0);
  }
}

void stopPump() {
  pwm_actual = 0.0f;   // Nullstill ramp slik at pumpa starter mykt neste gang
  drivePump(0, 0.0f);
}

// Les dybde fra MS5837-02BA
// Biblioteket må ha fått riktig modell satt.
// Trykksensoren måler absolutt trykk, så vi trekker fra overflatetrykk
// og regner om til dybde i ferskvann.
float readDepthMeters() {
  if (!sensorOK) return h; // Ekstra fallback til estimert verdi, failsafe håndteres i loop

  sensor.read();

  // pressure() returnerer mbar i Blue Robotics-biblioteket
  float pressure_mbar = sensor.pressure();

  // Konverter mbar -> Pa
  float pressurePa = (pressure_mbar - surfacePressure_mbar) * 100.0f;

  float depth = pressurePa / (rho * g);
  if (depth < 0.0f) depth = 0.0f;
  return depth;
}

// Enkel relélesing.
bool remoteEnabled() {
  const bool invertLogic = true;
  bool raw = digitalRead(PIN_RELAY_IN);
  return invertLogic ? !raw : raw;
}

bool remoteStartPressed() {
  bool now = remoteEnabled();
  bool pressed = (now && !remotePrev);   // rising edge: ikke aktiv -> aktiv
  remotePrev = now;
  return pressed;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  // I2C på GP0/GP1
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();

  pinMode(PIN_INA, OUTPUT);
  pinMode(PIN_INB, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);

  pinMode(PIN_RELAY_IN, INPUT_PULLUP); 
  // Bytt til INPUT hvis relay/mottaker allerede gir definert HIGH/LOW

  stopPump();

  // Pico-kjernen lar deg remappe SDA/SCL før Wire.begin()

  // Init sensor
  sensorOK = sensor.init();
  if (!sensorOK) {
    Serial.println("FEIL: Fant ikke MS5837 sensor.");
  } else {
    sensor.setModel(MS5837::MS5837_02BA);
    // Hvorfor ikke setFluidDensity() kommer her: Valgfritt i biblioteket: salt/ferkvann brukes ofte i depth()-funksjonen,
    // men vi regner dybde selv under for kontroll.
    delay(500); //Gir sensor litt tid til å initialisere

    // Kalibrer overflatetrykk ved oppstart
    float sum = 0.0f;
    const int samples = 20;
    for (int i = 0; i < samples; i++) {
      sensor.read();
      sum += sensor.pressure();
      delay(50); //Tid mellom målingene
    }
    surfacePressure_mbar = sum / samples;

    Serial.print("Surface pressure [mbar]: ");
    Serial.println(surfacePressure_mbar, 3);
  }

  h = 0.0f;
  v = 0.0f;
  integrator = 0.0f;
  m_ballast = 0.0f;
  h_ref = 0.0f;
  Q = 0.0f;
  pwm_actual = 0.0f;

  startMs = millis();
  lastStepMs = millis();

  Serial.println("System klart.");
}

void loop() {
  unsigned long now = millis();
  if (now - lastStepMs < (unsigned long)(dt * 1000.0f)) { 
    return; //"Hvis det ikke har gått 100 ms siden sist → gjør ingenting"
  }
  lastStepMs += (unsigned long)(dt * 1000.0f);

  //FAILSAFE
  if (!sensorOK) {
    Serial.println("FAILSAFE: Sensor ikke funnet. Tømmer ballasttank.");
    drivePump(-1, 1.0f);
    return;
  }

// Vent på momentary start fra fjernkontroll / relé
if (!started && remoteStartPressed()) {
  delay(50); // debounce
  if (remoteEnabled()) {
    started = true;
    startMs = now;  // start mission-tidspunkt ved knappetrykk
  }
}

// Hvis ikke startet:
if (!started) {
    stopPump();
    integrator = 0.0f;   // unngå windup når systemet er deaktivert

    // Oppdater målt dybde selv om pumpa er av
    float h_meas = readDepthMeters();
    h = h_meas;

    Serial.print("DISABLED, depth="); // Dybde når fjernkontroll er av
    Serial.println(h, 3);
    return;
  }

// Beregn mission-tid ETTER start
  float t = (now - startMs) / 1000.0f;

  // ======================
  // Mission control & setpoint shaping
  // ======================
  if (t < 5000.0f) {
    h_ref = min(ønsket_dybde - 0.1f, h_ref + 0.2f * dt); //Inkrementering av referansedybden clampet til 9.9m
  } else {
    h_ref = max(0.0f, h_ref - 0.5f * dt);
  }

  // ======================
  // Sensor
  // ======================
  float h_prev = h;
  float h_meas = readDepthMeters();
  h = h_meas;

  // Deriver hastighet fra sensor
  v = (h - h_prev) / dt; //Forrige og nåværende loopiterasjon

  // ======================
  // Feil
  // ======================
  float error = h_ref - h;

  // Full / tom tank
  bool full = (m_ballast >= m_ballast_maks);
  bool tom  = (m_ballast <= 0.0f);

  // ======================
  // PID
  // ======================
  float u_raw = Kp * error - Kd * v + Ki * integrator;
  float u = constrain(u_raw, -1.0f, 1.0f);

  int retning = 0;
  float pwm = 0.0f;

  if (fabs(error) < e_deadzone || fabs(u) < 0.01f) {
    retning = 0;
    pwm = 0.0f;
  } else {
    retning = (u > 0.0f) ? 1 : -1;
    pwm = fabs(u);
  }

  // ======================
  // Anti-windup
  // ======================
  bool blocked = (full && retning == 1) || (tom && retning == -1);
  bool stopped = (retning == 0);
  bool metning = (fabs(u_raw) > 1.0f) || blocked || stopped;

  if (!metning) {
    integrator += error * dt;
  }

  // ======================
  // Pumpelogikk
  // ======================
  if (blocked) {
    Q = 0.0f;
    stopPump();
  } else {
    // Bruk PWM-ramping for myk start/stopp
    float pwm_smooth = rampPWM(pwm);  // Faktisk PWM etter myk ramping

    Q = retning * pwm_smooth * Q_maalt;
    drivePump(retning, pwm_smooth);
  }

  // ======================
  // Ballastmodell
  // ======================
  m_ballast += rho * Q * dt;

  if (m_ballast < 0.0f) {
    m_ballast = 0.0f;
  } else if (m_ballast > m_ballast_maks) {
    m_ballast = m_ballast_maks;
  }

  // ======================
  // Debug
  // ======================
  Serial.print("t=");
  Serial.print(t, 1);
  Serial.print("  h_ref=");
  Serial.print(h_ref, 3);
  Serial.print("  h=");
  Serial.print(h, 3);
  Serial.print("  v=");
  Serial.print(v, 3);
  Serial.print("  err=");
  Serial.print(error, 3);
  Serial.print("  u=");
  Serial.print(u, 3);
  Serial.print("  pwm=");
  Serial.print(pwm, 3);
  Serial.print("  pwm_actual=");
  Serial.print(pwm_actual, 3);
  Serial.print("  dir=");
  Serial.print(retning);
  Serial.print("  m_ballast=");
  Serial.println(m_ballast, 4); //Tall er antall desimaler
}