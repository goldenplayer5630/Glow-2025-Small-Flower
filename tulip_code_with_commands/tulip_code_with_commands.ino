#include <Arduino.h>

// ================== RS-485 Flower Node (Serial-only, blocking moves) ==================
const uint8_t myID = 4;

// === Pin Definitions ===
const uint8_t motorOpenPin  = 10;  // PWM for OPEN direction
const uint8_t motorClosePin = 9;   // PWM for CLOSE direction
const uint8_t ledStripPin   = 3;   // LED strip (PWM)
const uint8_t rs485DirPin   = 2;   // MAX485 DE/RE tied together

// === Motor Speeds (0..255) ===
const uint8_t openSpeed  = 250;
const uint8_t closeSpeed = 250;

// === Hard-coded run times (ms) ===
const unsigned long openTimeMs    = 1300;
const unsigned long closeTimeMs   = 1300;
const unsigned long wiggleTimeMs  = 100;

// === H-bridge dead time ===
const unsigned long deadTimeMs    = 10;

// === LED Brightness Levels ===
const int ledMinBrightness = 0;
const int ledMaxBrightness = 120;

// === LED Ramping ===
const unsigned long ledRampDuration   = 7500; // total ramp duration (ms)
const int           ledStep           = 2;    // change per step
const unsigned long ledRampStepDelay  = 20;   // ms between LED steps

// Derived
const int ledBrightnessRange = ledMaxBrightness - ledMinBrightness;

// State
bool isOpen = false;                         // tracked by timers (no sensors)
int  currentLedBrightness = ledMinBrightness;

// Busy flag to ignore new commands during a move/ramp
enum SystemState { IDLE, BUSY };
SystemState sys = IDLE;

// --- Forward declarations ---
void runMotorForward(unsigned long motorMs);              // NEW: motor-only
void runMotorBackward(unsigned long motorMs);             // NEW: motor-only
void runMotorForwardWithLed(unsigned long motorMs);
void runMotorBackwardWithLed(unsigned long motorMs);
void stopMotor();
void setLedRaw(int value);
void rampLedTo(int targetBrightness, unsigned long rampMs);
void initSequence();
void blinkSequence();

// ======== RS-485 (Serial on pins 0/1) ========
#define RS485 Serial

void rs485Begin() {
  pinMode(rs485DirPin, OUTPUT);
  digitalWrite(rs485DirPin, LOW); // RX (DE/RE low)
  RS485.begin(115200);
  delay(50);
}

void rs485SendLine(const String& s) {
  digitalWrite(rs485DirPin, HIGH);  // TX enable
  delayMicroseconds(8);
  RS485.print(s);
  RS485.print('\n');
  RS485.flush();
  digitalWrite(rs485DirPin, LOW);   // back to RX
}

// ACK helper (only for directed messages)
void ack(const String& payload, bool isBroadcast) {
  if (!isBroadcast) rs485SendLine(String(myID) + "/ACK:" + payload);
}

// ======== Setup / Loop ========
void setup() {
  pinMode(motorOpenPin,  OUTPUT);
  pinMode(motorClosePin, OUTPUT);
  pinMode(ledStripPin,   OUTPUT);

  stopMotor();
  setLedRaw(ledMinBrightness);

  rs485Begin();
}

void loop() {
  handleSerialCommand();
}

// ======== Serial Command Handler ========
void processLine(String msg) {
  msg.trim();
  int slash = msg.indexOf('/');
  if (slash < 0) return;

  int id = msg.substring(0, slash).toInt();
  bool isBroadcast = (id == 0);
  if (!isBroadcast && id != myID) return;

  String rest = msg.substring(slash + 1);
  int colon = rest.indexOf(':');
  String cmd = (colon < 0) ? rest : rest.substring(0, colon);
  String val = (colon < 0) ? ""   : rest.substring(colon + 1);

  // Ignore new commands while busy (simple + robust)
  if (sys == BUSY) return;

  if (cmd == "OPEN") {
    if (!isOpen) {
      ack("OPEN", isBroadcast);
      runMotorForward(openTimeMs);     // motor-only
      isOpen = true;
    } else {
      ack("NOOP:ALREADY_OPEN", isBroadcast);
    }
  }
  else if (cmd == "CLOSE") {
    if (isOpen) {
      ack("CLOSE", isBroadcast);
      runMotorBackward(closeTimeMs);   // motor-only
      isOpen = false;
    } else {
      ack("NOOP:ALREADY_CLOSED", isBroadcast);
    }
  }
  else if (cmd == "LED") {
    int v = constrain(val.toInt(), ledMinBrightness, ledMaxBrightness);
    setLedRaw(v);
    ack("LED:" + String(v), isBroadcast);
  }
  else if (cmd == "LEDRAMP") {
    int comma = val.indexOf(',');
    int target = ledMaxBrightness;
    unsigned long dur = 1500; // default
    if (comma >= 0) {
      target = val.substring(0, comma).toInt();
      dur    = val.substring(comma + 1).toInt();
    } else if (val.length()) {
      target = val.toInt();
    }
    target = constrain(target, ledMinBrightness, ledMaxBrightness);
    ack("LEDRAMP:" + String(target) + "," + String(dur), isBroadcast);
    sys = BUSY; rampLedTo(target, dur); sys = IDLE;
  }
  else if (cmd == "OPENLEDRAMP") {
    int comma = val.indexOf(',');
    if (comma >= 0) {
      int target = constrain(val.substring(0, comma).toInt(), ledMinBrightness, ledMaxBrightness);
      unsigned long dur = val.substring(comma + 1).toInt();
      if (!isOpen) {
        ack("OPENLEDRAMP:" + String(target) + "," + String(dur), isBroadcast);
        sys = BUSY;
        // run motor and ramp in parallel-ish (LED ramp limited by motor time)
        unsigned long rampMs = min(openTimeMs, dur);
        analogWrite(motorClosePin, 0);
        delay(deadTimeMs);
        analogWrite(motorOpenPin, openSpeed);
        rampLedTo(target, rampMs);
        if (openTimeMs > rampMs) delay(openTimeMs - rampMs);
        stopMotor();
        isOpen = true;
        sys = IDLE;
      } else {
        ack("NOOP:ALREADY_OPEN", isBroadcast);
      }
    }
  }
  else if (cmd == "CLOSELEDRAMP") {
    int comma = val.indexOf(',');
    if (comma >= 0) {
      int target = constrain(val.substring(0, comma).toInt(), ledMinBrightness, ledMaxBrightness);
      unsigned long dur = val.substring(comma + 1).toInt();
      if (isOpen) {
        ack("CLOSELEDRAMP:" + String(target) + "," + String(dur), isBroadcast);
        sys = BUSY;
        unsigned long rampMs = min(closeTimeMs, dur);
        analogWrite(motorOpenPin, 0);
        delay(deadTimeMs);
        analogWrite(motorClosePin, closeSpeed);
        rampLedTo(target, rampMs);
        if (closeTimeMs > rampMs) delay(closeTimeMs - rampMs);
        stopMotor();
        isOpen = false;
        sys = IDLE;
      } else {
        ack("NOOP:ALREADY_CLOSED", isBroadcast);
      }
    }
  }
  else if (cmd == "INIT") {
    initSequence();
  }
  else if (cmd == "STATE?") {
    rs485SendLine(String(myID) + "/STATE:ISOPEN=" + String(isOpen ? 1 : 0) +
                  ",BUSY=" + String(sys == BUSY ? 1 : 0) +
                  ",LED=" + String(currentLedBrightness));
  }
}

void handleSerialCommand() {
  static String in;
  while (RS485.available()) {
    char c = (char)RS485.read();
    if (c == '\n' || c == '\r') {
      if (in.length() > ledMinBrightness) processLine(in);
      in = "";
    } else if (in.length() < ledMaxBrightness) {
      in += c;
    }
  }
}

// === Motor-only helpers (no LED changes) ===
void runMotorForward(unsigned long motorMs) {
  sys = BUSY;
  analogWrite(motorClosePin, 0);
  delay(deadTimeMs);
  analogWrite(motorOpenPin, openSpeed);
  delay(motorMs);
  stopMotor();
  isOpen = true;
  sys = IDLE;
}

void runMotorBackward(unsigned long motorMs) {
  sys = BUSY;
  analogWrite(motorOpenPin, 0);
  delay(deadTimeMs);
  analogWrite(motorClosePin, closeSpeed);
  delay(motorMs);
  stopMotor();
  isOpen = false;
  sys = IDLE;
}

// === Motor + LED helpers (kept for *LEDRAMP* combos) ===
void runMotorForwardWithLed(unsigned long motorMs) {
  sys = BUSY;
  analogWrite(motorClosePin, 0);
  delay(deadTimeMs);
  analogWrite(motorOpenPin, openSpeed);

  const unsigned long rampMs = min(motorMs, ledRampDuration);
  rampLedTo(ledMaxBrightness, rampMs);
  if (motorMs > rampMs) delay(motorMs - rampMs);

  stopMotor();
  isOpen = true;
  sys = IDLE;
}

void runMotorBackwardWithLed(unsigned long motorMs) {
  sys = BUSY;
  analogWrite(motorOpenPin, 0);
  delay(deadTimeMs);
  analogWrite(motorClosePin, closeSpeed);

  const unsigned long rampMs = min(motorMs, ledRampDuration);
  rampLedTo(ledMinBrightness, rampMs);
  if (motorMs > rampMs) delay(motorMs - rampMs);

  stopMotor();
  isOpen = false;
  sys = IDLE;
}

void stopMotor() {
  analogWrite(motorOpenPin,  0);
  analogWrite(motorClosePin, 0);
}

// === LED control ===
void setLedRaw(int value) {
  if (value < ledMinBrightness) value = ledMinBrightness;
  if (value > ledMaxBrightness) value = ledMaxBrightness;
  currentLedBrightness = value;
  analogWrite(ledStripPin, currentLedBrightness);
}

// Smoothly ramp from currentLedBrightness to targetBrightness over rampMs.
void rampLedTo(int targetBrightness, unsigned long rampMs) {
  if (targetBrightness < ledMinBrightness) targetBrightness = ledMinBrightness;
  if (targetBrightness > ledMaxBrightness) targetBrightness = ledMaxBrightness;

  if (targetBrightness == currentLedBrightness || rampMs == 0) {
    setLedRaw(targetBrightness);
    return;
  }

  const int direction = (targetBrightness > currentLedBrightness) ? 1 : -1;

  unsigned long stepsByTime  = max(1UL, rampMs / ledRampStepDelay);
  int delta = abs(targetBrightness - currentLedBrightness);
  unsigned long stepsByDelta = max(1, delta / max(1, ledStep));
  unsigned long steps        = max(stepsByTime, stepsByDelta);

  unsigned long perStepDelay = (steps > 0) ? max(1UL, rampMs / steps) : ledRampStepDelay;
  float exactStep = (float)delta / (float)steps;
  float accumulator = 0.0f;

  for (unsigned long i = 0; i < steps; ++i) {
    accumulator += exactStep;
    int inc = (int)round(accumulator);
    if (inc != 0) {
      accumulator -= inc;
      int next = currentLedBrightness + direction * inc;
      if ((direction > 0 && next > targetBrightness) ||
          (direction < 0 && next < targetBrightness)) {
        next = targetBrightness;
      }
      setLedRaw(next);
    }
    delay(perStepDelay);
  }

  setLedRaw(targetBrightness);
}

void initSequence() {
    blinkSequence();

    analogWrite(motorClosePin, 0);
    analogWrite(motorOpenPin, openSpeed);
    delay(wiggleTimeMs);
    stopMotor(); delay(200);
    analogWrite(motorOpenPin, 0);
    analogWrite(motorClosePin, closeSpeed);
    delay(wiggleTimeMs);
    stopMotor(); delay(300);

    // force known closed state (timed close)
    analogWrite(motorOpenPin, 0);
    delay(deadTimeMs);
    analogWrite(motorClosePin, closeSpeed);
    delay(closeTimeMs);
    stopMotor();
    isOpen = false;
}

// --- Small blink sequence (blocking) ---
// Blinks 'times' at 'peak' brightness, with on/off durations in ms.
void blinkSequence()
{
  for (uint8_t i = 0; i < 3; ++i) {
    setLedRaw(120);
    delay(300);
    setLedRaw(0);
    delay(300);
  }
}

