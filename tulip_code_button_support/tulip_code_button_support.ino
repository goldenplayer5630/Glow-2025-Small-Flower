#include <Arduino.h>

// ================== CONFIG ==================

// Pins
const uint8_t motorOpenPin  = 10;   // PWM for OPEN direction
const uint8_t motorClosePin = 9;    // PWM for CLOSE direction
const uint8_t ledStripPin   = 3;    // LED strip (PWM)

// Reuse old MAX485 DE/RE pin as relay/button input
const uint8_t relayHoldPin  = 2;    // Relay COM->GND, NO->D2 (active LOW)

// Motor speeds (0..255)
const uint8_t openSpeed  = 250;
const uint8_t closeSpeed = 250;

// Timings (ms)
const unsigned long openTimeMs   = 1500;
const unsigned long closeTimeMs  = 1500;
const unsigned long ledRampMs    = 1300;
const unsigned long deadTimeMs   = 10;

// LED brightness (0..120 scale)
const int ledMinBrightness = 0;
const int ledMaxBrightness = 20;
const int ledPeak          = 20;   // target brightness while "open"

// Debounce
const unsigned long debounceMs = 60;

// ================== STATE ==================
enum Phase { CLOSED_IDLE, OPEN_RAMP, OPEN_HOLD, CLOSE_RAMP };
Phase phase = CLOSED_IDLE;

unsigned long phaseStart = 0;
int currentLed = ledMinBrightness;

// ================== HELPERS ==================
static inline void stopMotor() {
  analogWrite(motorOpenPin,  0);
  analogWrite(motorClosePin, 0);
}

static inline void motorOpenOn() {
  analogWrite(motorClosePin, 0);
  delayMicroseconds((unsigned int)(deadTimeMs * 1000UL));
  analogWrite(motorOpenPin, openSpeed);
}

static inline void motorCloseOn() {
  analogWrite(motorOpenPin, 0);
  delayMicroseconds((unsigned int)(deadTimeMs * 1000UL));
  analogWrite(motorClosePin, closeSpeed);
}

static inline void setLed(int v) {
  if (v < ledMinBrightness) v = ledMinBrightness;
  if (v > ledMaxBrightness) v = ledMaxBrightness;
  currentLed = v;
  analogWrite(ledStripPin, currentLed);
}

// Linear ramp helper: compute LED level for elapsed time
static inline int rampValue(int from, int to, unsigned long elapsed, unsigned long rampMs) {
  if (rampMs == 0 || elapsed >= rampMs) return to;
  long delta = (long)to - (long)from;
  long val = (long)from + (delta * (long)elapsed) / (long)rampMs;
  return (int)val;
}

// Debounced stable read of relay (active LOW with INPUT_PULLUP)
static inline bool relayPressedStable() {
  static bool lastRead = true;          // HIGH = not pressed
  static bool stable   = true;
  static unsigned long lastChangeAt = 0;

  bool r = digitalRead(relayHoldPin);
  unsigned long now = millis();

  if (r != lastRead) {
    lastRead = r;
    lastChangeAt = now;
  }

  if (now - lastChangeAt >= debounceMs) {
    stable = r;
  }

  return (stable == LOW); // pressed
}

// ================== SETUP ==================
void setup() {
  pinMode(motorOpenPin,  OUTPUT);
  pinMode(motorClosePin, OUTPUT);
  pinMode(ledStripPin,   OUTPUT);

  pinMode(relayHoldPin, INPUT_PULLUP);

  stopMotor();
  setLed(ledMinBrightness);

  phase = CLOSED_IDLE;
  phaseStart = millis();
}

// ================== LOOP (non-blocking state machine) ==================
void loop() {
  const unsigned long now = millis();
  const bool wantOpen = relayPressedStable();

  switch (phase) {
    case CLOSED_IDLE:
      // Ensure fully closed: motor off, LED off
      stopMotor();
      if (currentLed != ledMinBrightness) setLed(ledMinBrightness);

      if (wantOpen) {
        // Start opening + ramp up
        motorOpenOn();
        phase = OPEN_RAMP;
        phaseStart = now;
      }
      break;

    case OPEN_RAMP: {
      // Ramp 0 -> ledPeak within min(ledRampMs, openTimeMs)
      const unsigned long rampDur = (ledRampMs < openTimeMs) ? ledRampMs : openTimeMs;
      const unsigned long elapsed = now - phaseStart;

      setLed(rampValue(ledMinBrightness, ledPeak, elapsed, rampDur));

      // If relay released early, abort opening and start closing immediately
      if (!wantOpen) {
        stopMotor();
        motorCloseOn();
        phase = CLOSE_RAMP;
        phaseStart = now;
        break;
      }

      // End OPEN motor precisely at openTimeMs, then hold open
      if (elapsed >= openTimeMs) {
        stopMotor();
        phase = OPEN_HOLD;
        phaseStart = now;
        // ensure LED ends exactly at ledPeak
        if (currentLed != ledPeak) setLed(ledPeak);
      }
      break;
    }

    case OPEN_HOLD:
      // Keep LED at ledPeak while relay is ON
      if (currentLed != ledPeak) setLed(ledPeak);

      if (!wantOpen) {
        // Start closing + ramp down
        motorCloseOn();
        phase = CLOSE_RAMP;
        phaseStart = now;
      }
      break;

    case CLOSE_RAMP: {
      // Ramp ledPeak -> 0 within min(ledRampMs, closeTimeMs)
      const unsigned long rampDur = (ledRampMs < closeTimeMs) ? ledRampMs : closeTimeMs;
      const unsigned long elapsed = now - phaseStart;

      setLed(rampValue(ledPeak, ledMinBrightness, elapsed, rampDur));

      // If relay pressed again mid-close, reverse immediately to opening
      if (wantOpen) {
        stopMotor();
        motorOpenOn();
        phase = OPEN_RAMP;
        phaseStart = now;
        break;
      }

      if (elapsed >= closeTimeMs) {
        stopMotor();
        setLed(ledMinBrightness);
        phase = CLOSED_IDLE;
        phaseStart = now;
      }
      break;
    }
  }
}
