// IBT-2 (BTS7960) + Arduino Mega
// Control + monitoring + serial command interface
// --- With limit switches (ACTIVE when pin is LOW) ---

// ----- Pin Map -----
const int RPWM = 3;   // Right PWM (extend)
const int LPWM = 4;   // Left  PWM (retract)

// One set per actuator (A, B, C). Active LOW when hit.
const int R_LIMIT_A = 6; // actuator A - right/extend end-stop
const int L_LIMIT_A = 7; // actuator A - left/retract end-stop
const int R_LIMIT_B = 9; // actuator B - right/extend end-stop
const int L_LIMIT_B = 10; // actuator B - left/retract end-stop
const int R_LIMIT_C = 12; // actuator C - right/extend end-stop
const int L_LIMIT_C = 13; // actuator C - left/retract end-stop

// ----- Settings -----
// Wire each switch so it shorts to GND when the limit is hit.
// We use INPUT_PULLUP so the pin idles HIGH, goes LOW when hit.
const bool USE_PULLUPS = true;

int  defaultSpeed = 200;                 // 0..255 PWM
const unsigned long STATUS_MS = 500;     // periodic status while moving

// ----- State -----
enum Motion { IDLE, EXTENDING, RETRACTING };
Motion motion = IDLE;

unsigned long lastStatusMs = 0;

// ----- Helpers -----
// ACTIVE-LOW limit: returns true when pin reads LOW (limit hit)
inline bool limitActiveRaw(int pin) {
  return digitalRead(pin) == LOW;
}

// Any of the 3 EXT (right) limits active?
bool extendLimitAnyActive() {
  return limitActiveRaw(R_LIMIT_A) ||
         limitActiveRaw(R_LIMIT_B) ||
         limitActiveRaw(R_LIMIT_C);
}

// Any of the 3 RET (left) limits active?
bool retractLimitAnyActive() {
  return limitActiveRaw(L_LIMIT_A) ||
         limitActiveRaw(L_LIMIT_B) ||
         limitActiveRaw(L_LIMIT_C);
}

void stopMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  motion = IDLE;
}

void extendMotor(int speed) {
  // If already at ANY extend end-stop, refuse to move further
  if (extendLimitAnyActive()) {
    stopMotor();
    Serial.println(F("[WARN] Extend blocked: one or more EXT limits are ACTIVE (LOW)."));
    return;
  }

  analogWrite(LPWM, 0);
  analogWrite(RPWM, constrain(speed, 0, 255));
  motion = EXTENDING;
}

void retractMotor(int speed) {
  // If already at ANY retract end-stop, refuse to move further
  if (retractLimitAnyActive()) {
    stopMotor();
    Serial.println(F("[WARN] Retract blocked: one or more RET limits are ACTIVE (LOW)."));
    return;
  }

  analogWrite(RPWM, 0);
  analogWrite(LPWM, constrain(speed, 0, 255));
  motion = RETRACTING;
}

void printMotion() {
  Serial.print(F("Motion="));
  switch (motion) {
    case IDLE:       Serial.println(F("IDLE")); break;
    case EXTENDING:  Serial.println(F("EXTENDING")); break;
    case RETRACTING: Serial.println(F("RETRACTING")); break;
  }
}

void printLimits() {
  // Read all 6
  const bool extA = limitActiveRaw(R_LIMIT_A);
  const bool extB = limitActiveRaw(R_LIMIT_B);
  const bool extC = limitActiveRaw(R_LIMIT_C);
  const bool retA = limitActiveRaw(L_LIMIT_A);
  const bool retB = limitActiveRaw(L_LIMIT_B);
  const bool retC = limitActiveRaw(L_LIMIT_C);

  Serial.print(F("EXT_A=")); Serial.print(extA ? F("ACTIVE") : F("INACTIVE"));
  Serial.print(F("  EXT_B=")); Serial.print(extB ? F("ACTIVE") : F("INACTIVE"));
  Serial.print(F("  EXT_C=")); Serial.println(extC ? F("ACTIVE") : F("INACTIVE"));

  Serial.print(F("RET_A=")); Serial.print(retA ? F("ACTIVE") : F("INACTIVE"));
  Serial.print(F("  RET_B=")); Serial.print(retB ? F("ACTIVE") : F("INACTIVE"));
  Serial.print(F("  RET_C=")); Serial.println(retC ? F("ACTIVE") : F("INACTIVE"));

  if ((extA || extB || extC) && (retA || retB || retC)) {
    Serial.println(F("[WARN] Some EXT and some RET limits ACTIVE simultaneously — check wiring/mechanics."));
  }
}

void printOutputs() {
  int rCmd = 0, lCmd = 0;
  if (motion == EXTENDING)  rCmd = defaultSpeed;
  if (motion == RETRACTING) lCmd = defaultSpeed;

  Serial.print(F("CMD RPWM≈")); Serial.print(rCmd);
  Serial.print(F("  LPWM≈"));   Serial.println(lCmd);
}

void printStatus() {
  Serial.println(F("---- STATUS ----"));
  printMotion();
  printLimits();
  printOutputs();
  Serial.print(F("DefaultSpeed=")); Serial.println(defaultSpeed);
  Serial.println(F("----------------"));
}

void showHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  EXT [speed]    -> extend (0..255 optional)"));
  Serial.println(F("  RET [speed]    -> retract (0..255 optional)"));
  Serial.println(F("  STOP           -> stop motor"));
  Serial.println(F("  SPEED n        -> set default speed (0..255)"));
  Serial.println(F("  STATUS         -> print current status"));
  Serial.println(F("  HELP           -> this help"));
}

void handleCommand(const String &line) {
  String cmd = line; cmd.trim();
  if (cmd.length() == 0) return;

  String c = cmd; c.toUpperCase();
  int sp = c.indexOf(' ');
  String op  = (sp == -1) ? c : c.substring(0, sp);
  String arg = (sp == -1) ? "" : c.substring(sp + 1);
  arg.trim();

  if (op == "HELP" || op == "?") { showHelp(); return; }
  if (op == "STATUS") { printStatus(); return; }

  if (op == "STOP") {
    stopMotor();
    Serial.println(F("[OK] Stopped."));
    return;
  }

  if (op == "SPEED") {
    if (arg.length() == 0) {
      Serial.print(F("[INFO] DefaultSpeed=")); Serial.println(defaultSpeed);
      return;
    }
    int v = constrain(arg.toInt(), 0, 255);
    defaultSpeed = v;
    Serial.print(F("[OK] DefaultSpeed set to ")); Serial.println(defaultSpeed);
    return;
  }

  if (op == "EXT") {
    int v = (arg.length() ? constrain(arg.toInt(), 0, 255) : defaultSpeed);
    defaultSpeed = v;
    extendMotor(v);
    Serial.print(F("[OK] EXT speed=")); Serial.println(v);
    printLimits();
    return;
  }

  if (op == "RET") {
    int v = (arg.length() ? constrain(arg.toInt(), 0, 255) : defaultSpeed);
    defaultSpeed = v;
    retractMotor(v);
    Serial.print(F("[OK] RET speed=")); Serial.println(v);
    printLimits();
    return;
  }

  Serial.println(F("[ERR] Unknown command. Type HELP."));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB on Mega */ }

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  if (USE_PULLUPS) {
    pinMode(R_LIMIT_A, INPUT_PULLUP);
    pinMode(L_LIMIT_A, INPUT_PULLUP);
    pinMode(R_LIMIT_B, INPUT_PULLUP);
    pinMode(L_LIMIT_B, INPUT_PULLUP);
    pinMode(R_LIMIT_C, INPUT_PULLUP);
    pinMode(L_LIMIT_C, INPUT_PULLUP);
  } else {
    pinMode(R_LIMIT_A, INPUT);
    pinMode(L_LIMIT_A, INPUT);
    pinMode(R_LIMIT_B, INPUT);
    pinMode(L_LIMIT_B, INPUT);
    pinMode(R_LIMIT_C, INPUT);
    pinMode(L_LIMIT_C, INPUT);
  }

  stopMotor();

  Serial.println(F("IBT-2 Controller Ready (3 actuators A/B/C; limits active LOW). Type HELP for commands."));
  printStatus();
  Serial.print(F("> "));
}

void loop() {
  // ---- Command handling ----
  static String line;
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (line.length() > 0) {
        handleCommand(line);
        line = "";
        Serial.print(F("> "));
      } else {
        Serial.print(F("> "));
      }
    } else {
      if (line.length() < 80) line += ch; // cap length
    }
  }

  // ---- Safety: stop when ANY end limit reached in current motion (active LOW) ----
  if (motion == EXTENDING && extendLimitAnyActive()) {
    stopMotor();
    Serial.println(F("[INFO] Reached an EXT limit (LOW). All actuators stopped."));
  } else if (motion == RETRACTING && retractLimitAnyActive()) {
    stopMotor();
    Serial.println(F("[INFO] Reached a RET limit (LOW). All actuators stopped."));
  }

  // ---- Periodic status while moving ----
  unsigned long now = millis();
  if ((motion != IDLE) && (now - lastStatusMs >= STATUS_MS)) {
    lastStatusMs = now;
    printStatus();
  }
}
