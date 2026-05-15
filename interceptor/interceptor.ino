// iBooster Travel Sensor Interceptor
#include <avr/wdt.h>
#include <util/atomic.h>
#include <stdlib.h>

// --- Pin definitions ---
constexpr uint8_t PIN_S2_IN   = 3;
constexpr uint8_t PIN_S4_OUT  = 9;
constexpr uint8_t PIN_S2_OUT  = 10;
constexpr uint8_t PIN_SWT_SRC = 7;

// --- Sensor transfer function ---
constexpr float S2_REST = 83.0f;
constexpr float S2_FULL = 61.0f;

// --- Travel detection ---
constexpr float REST_TOLERANCE       = 2.0f;
constexpr unsigned long REST_HOLD_MS = 100;

// --- Input signal bounds ---
constexpr unsigned long PWM_PERIOD_MIN = 900UL;   // reject periods outside
constexpr unsigned long PWM_PERIOD_MAX = 1100UL;  // this window (~10% tolerance)

// --- Update rates ---
constexpr unsigned int PWM_SET_RATE = 2;
constexpr unsigned int PRINT_RATE   = 20;
constexpr float COMMAND_RAMP_RATE_PCT_PER_SEC = 25.0f;
constexpr uint8_t CMD_BUFFER_SIZE = 32;
constexpr uint16_t PWM_TOP_VALUE = 1999;

// --- EMA smoothing (0.0 = max smooth, 1.0 = no smoothing) ---
constexpr float EMA_ALPHA = 0.1f;

// --- Calibration curve ---
constexpr uint8_t CAL_POINTS = 6;
constexpr float CAL_IN[CAL_POINTS]  = {  0.0f, 20.0f, 40.0f, 60.0f,  80.0f, 100.0f };
//constexpr float CAL_OUT[CAL_POINTS] = {  0.0f, 6.0f, 20.0f, 32.0f,  48.0f,  60.0f };
constexpr float CAL_OUT[CAL_POINTS] = {  0.0f, 12.0f, 24.0f, 36.0f,  48.0f, 64.0f };
//constexpr float CAL_OUT[CAL_POINTS]  = {  0.0f, 20.0f, 40.0f, 60.0f,  80.0f, 100.0f };

// --- State ---
enum class Mode { PASSTHROUGH, ACTIVE, COMMAND };
volatile Mode mode = Mode::PASSTHROUGH;

// Manual override: when true, serial commands lock the mode and the
// zero-detect state machine is disabled so it can't fight the override.
bool manualOverride = false;

volatile unsigned long rise_s2 = 0;
volatile bool s2_valid = false;
volatile float s2inputDuty = S2_REST;
volatile unsigned long s2_last_period = 1000UL;  // sensible default until first measurement
volatile unsigned long s2_highTime = 0;

unsigned long timerPwmSet, timerPrint;
unsigned long restEnteredAt = 0;
bool atRest = false;
float commandTargetTravel = 0.0f;
float commandCurrentTravel = 0.0f;
unsigned long commandLastUpdateAt = 0;

// -------------------------------------------------------
// Calibration interpolation
// -------------------------------------------------------
float lookupCurve(float travel) {
  travel = constrain(travel, CAL_IN[0], CAL_IN[CAL_POINTS - 1]);
  for (uint8_t i = 0; i < CAL_POINTS - 1; i++) {
    if (travel <= CAL_IN[i + 1]) {
      float t = (travel - CAL_IN[i]) / (CAL_IN[i + 1] - CAL_IN[i]);
      return CAL_OUT[i] + t * (CAL_OUT[i + 1] - CAL_OUT[i]);
    }
  }
  return CAL_OUT[CAL_POINTS - 1];
}

// -------------------------------------------------------
// Convert spoofed travel% to output duty cycles
// -------------------------------------------------------
void travelToDuty(float travel, float &s2duty, float &s4duty) {
  float t = constrain(travel, 0.0f, 100.0f) / 100.0f;
  s2duty = S2_REST + t * (S2_FULL - S2_REST);
  // Complementary outputs: S2 + S4 must always equal 100%.
  s4duty = 100.0f - s2duty;
}

// -------------------------------------------------------
// ISRs
// -------------------------------------------------------
void isr_s2() {
  unsigned long now = micros();
  if (digitalRead(PIN_S2_IN) == HIGH) {
    if (s2_valid) {
      unsigned long period = now - rise_s2;
      if (period >= PWM_PERIOD_MIN && period <= PWM_PERIOD_MAX) {
        s2_last_period = period;
      }
    }
    rise_s2 = now;
    s2_valid = true;
  } else if (s2_valid && s2_last_period > 0) {
    unsigned long highTime = now - rise_s2;
    if (highTime > 0UL && highTime < s2_last_period) {
      s2_highTime = highTime;
      float raw = (float)highTime / (float)s2_last_period * 100.0f;
      s2inputDuty = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * s2inputDuty;
    }
  }
}

// -------------------------------------------------------
// Helpers
// -------------------------------------------------------
float computeTravelFromS2(float s2) {
  return constrain((S2_REST - s2) / (S2_REST - S2_FULL) * 100.0f, 0.0f, 100.0f);
}

bool pedalAtRest(float travel) {
  return travel <= REST_TOLERANCE;
}

void setMode(Mode m) {
  Mode oldMode = mode;
  mode = m;
  digitalWrite(PIN_SWT_SRC, (m == Mode::ACTIVE || m == Mode::COMMAND) ? HIGH : LOW);
  if (oldMode != Mode::COMMAND && m == Mode::COMMAND) {
    commandCurrentTravel = 0.0f;
  }
}

void setCommandTarget(float target) {
  commandTargetTravel = constrain(target, 0.0f, 100.0f);
  manualOverride = true;
  setMode(Mode::COMMAND);
  Serial.print("# MANUAL: COMMAND target=");
  Serial.println(commandTargetTravel, 1);
}

void processSerialCommand(const char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  char first = *line;
  if ((first == 'p' || first == 'P') && line[1] == '\0') {
    manualOverride = true;
    setMode(Mode::PASSTHROUGH);
    atRest = false;
    Serial.println("# MANUAL: PASSTHROUGH");
    return;
  }
  if ((first == 'a' || first == 'A') && line[1] == '\0') {
    manualOverride = true;
    setMode(Mode::ACTIVE);
    Serial.println("# MANUAL: ACTIVE");
    return;
  }
  if ((first == 'r' || first == 'R') && line[1] == '\0') {
    manualOverride = false;
    atRest = false;
    restEnteredAt = 0;
    setMode(Mode::PASSTHROUGH);
    Serial.println("# AUTO: zero-detect resumed");
    return;
  }
  if ((first == 'c' || first == 'C') && line[1] == '\0') {
    manualOverride = true;
    setMode(Mode::COMMAND);
    Serial.println("# MANUAL: COMMAND");
    return;
  }

  const char *parseStart = line;
  if (*parseStart == 'c' || *parseStart == 'C') {
    parseStart++;
    while (*parseStart == ' ' || *parseStart == '\t') parseStart++;
  }

  char *endPtr = nullptr;
  float requestedTravel = (float)strtod(parseStart, &endPtr);
  if (endPtr != parseStart) {
    while (*endPtr == ' ' || *endPtr == '\t') endPtr++;
    if (*endPtr == '\0') {
      setCommandTarget(requestedTravel);
      return;
    }
  }

  Serial.println("# Unknown command");
}

// -------------------------------------------------------
// Serial command handler
// Commands (send line + Enter):
//   p     -> force PASSTHROUGH, lock manual override
//   a     -> force ACTIVE, lock manual override
//   r     -> release manual override, resume auto zero-detect
//   c     -> force COMMAND mode (uses last target)
//   0-100 -> set COMMAND target travel (%), constrained to 0..100
//   c 35  -> same as above, with explicit command prefix
// -------------------------------------------------------
void handleSerial() {
  static char cmdBuffer[CMD_BUFFER_SIZE];
  static uint8_t cmdLen = 0;
  static bool discardingLongCommand = false;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (discardingLongCommand) {
      if (c == '\n') {
        discardingLongCommand = false;
      }
      continue;
    }
    if (c == '\n') {
      cmdBuffer[cmdLen] = '\0';
      processSerialCommand(cmdBuffer);
      cmdLen = 0;
      continue;
    }

    if (cmdLen < CMD_BUFFER_SIZE - 1) {
      cmdBuffer[cmdLen++] = c;
    } else {
      cmdLen = 0;
      discardingLongCommand = true;
      Serial.println("# Command too long");
    }
  }
}

// -------------------------------------------------------
// Setup
// -------------------------------------------------------
void setup() {
  pinMode(PIN_SWT_SRC, OUTPUT);
  digitalWrite(PIN_SWT_SRC, LOW);

  Serial.begin(115200);

  pinMode(PIN_S2_IN, INPUT);
  pinMode(PIN_S4_OUT, OUTPUT);
  pinMode(PIN_S2_OUT, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13)  | _BV(WGM12)  | _BV(CS11);
  ICR1   = PWM_TOP_VALUE;
  OCR1A  = (uint16_t)((100.0f - S2_REST) / 100.0f * PWM_TOP_VALUE);
  OCR1B  = (uint16_t)(S2_REST / 100.0f * PWM_TOP_VALUE);

  attachInterrupt(digitalPinToInterrupt(PIN_S2_IN), isr_s2, CHANGE);

  wdt_enable(WDTO_2S);
  commandLastUpdateAt = millis();

  Serial.println("# Commands: P=passthrough  A=active  C=command  R=resume auto");
  Serial.println("# Command target: send 0..100 (or 'c <value>'), ramps at 25%/sec");
  Serial.println("pedal_in_pct\tpedal_out_pct\tmode");
}

// -------------------------------------------------------
// Loop
// -------------------------------------------------------
void loop() {
  wdt_reset();

  handleSerial();

  float s2;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    s2 = s2inputDuty;
  }
  float travel = computeTravelFromS2(s2);
  float reportedTravel = travel;
  unsigned long now = millis();
  float dtSec = (now - commandLastUpdateAt) / 1000.0f;
  commandLastUpdateAt = now;

  if (mode == Mode::COMMAND) {
    float step = COMMAND_RAMP_RATE_PCT_PER_SEC * dtSec;
    if (commandCurrentTravel < commandTargetTravel) {
      commandCurrentTravel = min(commandCurrentTravel + step, commandTargetTravel);
    } else if (commandCurrentTravel > commandTargetTravel) {
      commandCurrentTravel = max(commandCurrentTravel - step, commandTargetTravel);
    }
  }

  // --- Zero-detect state machine (suppressed during manual override) ---
  if (!manualOverride && mode == Mode::PASSTHROUGH) {
    if (pedalAtRest(travel)) {
      if (!atRest) {
        atRest = true;
        restEnteredAt = now;
      } else if (now - restEnteredAt >= REST_HOLD_MS) {
        setMode(Mode::ACTIVE);
      }
    } else {
      atRest = false;
    }
  }

  if (mode == Mode::ACTIVE) {
    reportedTravel = lookupCurve(travel);
  } else if (mode == Mode::COMMAND) {
    reportedTravel = commandCurrentTravel;
  }

  // --- PWM output update ---
  if (now - timerPwmSet >= PWM_SET_RATE) {
    float s2out, s4out;
    if (mode == Mode::ACTIVE) {
      travelToDuty(reportedTravel, s2out, s4out);
    } else if (mode == Mode::COMMAND) {
      travelToDuty(reportedTravel, s2out, s4out);
    } else {
      s2out = s2;
      s4out = 100.0f - s2out;
    }
    OCR1A = (uint16_t)constrain(s4out / 100.0f * PWM_TOP_VALUE, 0, PWM_TOP_VALUE);
    OCR1B = (uint16_t)constrain(s2out / 100.0f * PWM_TOP_VALUE, 0, PWM_TOP_VALUE);
    timerPwmSet = now;
  }

  // --- Serial output ---
  if (now - timerPrint >= PRINT_RATE) {
    Serial.print(travel, 1);         Serial.print("\t");
    Serial.print(reportedTravel, 1); Serial.print("\t");
    if (mode == Mode::ACTIVE) {
      Serial.println("ACTIVE");
    } else if (mode == Mode::COMMAND) {
      Serial.println("COMMAND");
    } else {
      Serial.println("PASSTHRU");
    }
    timerPrint = now;
  }
}
