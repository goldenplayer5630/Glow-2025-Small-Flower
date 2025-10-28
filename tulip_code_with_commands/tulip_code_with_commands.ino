// ================== RS-485 Flower Node (ACK-only, no debug) ==================
const int myID = 17;

// ---------- Pins ----------
const int motorOpenPin  = 9;
const int motorClosePin = 10;

const int hallOpenPin   = 12;
const int hallClosePin  = 11;

const int buttonPin     = 13;

const int petalLedPin   = 3;
const int centralLedPin = 8;

const int rs485DirPin   = 2;   // DE/RE (tie RE to DE on the MAX485)

// ---------- Motor / LED ----------
const int motorOpenSpeed  = 130;
const int motorCloseSpeed = 250;
const unsigned long wiggleTimeMs = 100;

const int ledMinBrightness = 0;
const int ledMaxBrightness = 120;

const unsigned long ledRampDefaultMs = 1500;

// ---------- Debounce / Safety ----------
const unsigned long debounceDelay = 250;
const unsigned long motorSafetyTimeoutMs = 10000;

unsigned long motorStartTime = 0;

// ---------- State ----------
enum SystemState { WAITING, OPENING, CLOSING };
SystemState currentState = WAITING;

bool lastButtonState = LOW;
unsigned long lastButtonTime = 0;

int  currentLedBrightness = ledMinBrightness;
bool motorStarted = false;

// LED ramp
bool           ledRamping = false;
int            ledRampStartBrightness = 0;
int            ledTargetBrightness = ledMinBrightness;
unsigned long  ledRampStartTime = 0;
unsigned long  ledRampDurationMs = 0;

// ======== RS-485 (Serial on Nano pins 0/1) ========
#define RS485 Serial

void rs485Begin() {
  pinMode(rs485DirPin, OUTPUT);
  digitalWrite(rs485DirPin, LOW);   // idle = receive (DE/RE low)
  RS485.begin(115200);
  delay(100);                       // allow transceiver to settle
}

// Enable driver, send a line, return to RX
void rs485SendLine(const String& s) {
  digitalWrite(rs485DirPin, HIGH);  // enable driver
  delayMicroseconds(8);
  RS485.print(s);
  RS485.print('\n');
  RS485.flush();                    // wait until TX shift reg empty
  digitalWrite(rs485DirPin, LOW);   // back to receive
}

// ACK helper (the ONLY thing we transmit on the bus)
void ack(const String& payload, bool isBroadcast) {
  // 0/... -> broadcast (executed by all, NO ACK)
  // N/... -> direct to node N (with ACK)
  if (!isBroadcast) {
    rs485SendLine(String(myID) + "/ACK:" + payload);
  }
}

// ======== Setup / Loop ========
void setup() {
  pinMode(motorOpenPin, OUTPUT);
  pinMode(motorClosePin, OUTPUT);

  pinMode(hallOpenPin, INPUT_PULLUP);
  pinMode(hallClosePin, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(petalLedPin, OUTPUT);
  pinMode(centralLedPin, OUTPUT);

  stopMotor();
  analogWrite(petalLedPin, ledMinBrightness);
  digitalWrite(centralLedPin, LOW);

  rs485Begin();
  startupSequence();   // quiet mechanical init
}

void loop() {
  unsigned long now = millis();

  handleSerialCommand();

  // Local button
  bool buttonPressed = (digitalRead(buttonPin) == LOW);
  if (buttonPressed && lastButtonState == HIGH && (now - lastButtonTime > debounceDelay)) {
    lastButtonTime = now;

    if (currentState == WAITING) {
      if (digitalRead(hallClosePin) == LOW) {
        currentState = OPENING;
        motorStarted = false;
      } else if (digitalRead(hallOpenPin) == LOW) {
        currentState = CLOSING;
        motorStarted = false;
      }
    } else {
      stopMotor();
      currentState = WAITING;
    }
  }
  lastButtonState = buttonPressed ? LOW : HIGH;

  handleMotorState();
  updateLedRamp();
}

// ======== Motor control ========
void handleMotorState() {
  if (currentState == OPENING) {
    if (!motorStarted) {
      analogWrite(motorClosePin, 0);
      analogWrite(motorOpenPin, motorOpenSpeed);
      motorStarted = true;
      motorStartTime = millis();
    }

    if (motorStarted && (millis() - motorStartTime >= motorSafetyTimeoutMs)) {
      stopMotor();
      currentState = WAITING;
      return;
    }

    if (digitalRead(hallOpenPin) == LOW) {
      stopMotor();
      currentState = WAITING;
      return;
    }
  }
  else if (currentState == CLOSING) {
    if (!motorStarted) {
      analogWrite(motorOpenPin, 0);
      analogWrite(motorClosePin, motorCloseSpeed);
      motorStarted = true;
      motorStartTime = millis();
    }

    if (motorStarted && (millis() - motorStartTime >= motorSafetyTimeoutMs)) {
      stopMotor();
      currentState = WAITING;
      return;
    }

    if (digitalRead(hallClosePin) == LOW) {
      stopMotor();
      currentState = WAITING;
      return;
    }
  }
}

void stopMotor() {
  analogWrite(motorOpenPin, 0);
  analogWrite(motorClosePin, 0);
  motorStarted = false;
}

// ======== LED ramp ========
void startLedRamp(int target, unsigned long durationMs) {
  target = constrain(target, ledMinBrightness, ledMaxBrightness);
  ledRampStartBrightness = currentLedBrightness;
  ledTargetBrightness    = target;
  ledRampDurationMs      = durationMs;
  ledRampStartTime       = millis();
  ledRamping             = true;
}

void updateLedRamp() {
  if (!ledRamping) return;

  unsigned long elapsed = millis() - ledRampStartTime;
  if (elapsed >= ledRampDurationMs) {
    currentLedBrightness = ledTargetBrightness;
    analogWrite(petalLedPin, currentLedBrightness);
    digitalWrite(centralLedPin, currentLedBrightness > 0 ? HIGH : LOW);
    ledRamping = false;
    return;
  }

  float progress = (float)elapsed / (float)ledRampDurationMs;
  int newBrightness =
      ledRampStartBrightness +
      (int)((ledTargetBrightness - ledRampStartBrightness) * progress);

  currentLedBrightness = newBrightness;
  analogWrite(petalLedPin, currentLedBrightness);
  digitalWrite(centralLedPin, currentLedBrightness > 0 ? HIGH : LOW);
}

// ======== Command handler (RS-485) ========
void handleSerialCommand() {
  static String in;
  while (RS485.available()) {
    char c = (char)RS485.read();
    if (c == '\n' || c == '\r') {
      if (in.length() > 0) processLine(in);
      in = "";
    } else if (in.length() < 120) {
      in += c;
    }
  }
}

void processLine(String msg) {
  msg.trim();

  int slash = msg.indexOf('/');
  if (slash < 0) return;

  int id = msg.substring(0, slash).toInt();
  bool isBroadcast = (id == 0);
  if (!isBroadcast && id != myID) return;

  String rest = msg.substring(slash + 1);
  int colon = rest.indexOf(':');
  String command = (colon < 0) ? rest : rest.substring(0, colon);
  String value   = (colon < 0) ? ""   : rest.substring(colon + 1);

  if (command == "OPEN") {
    currentState = OPENING;
    motorStarted = false;
    ack("OPEN", isBroadcast);
  }
  else if (command == "CLOSE") {
    currentState = CLOSING;
    motorStarted = false;
    ack("CLOSE", isBroadcast);
  }
  else if (command == "LED") {
    int val = value.toInt();
    currentLedBrightness = constrain(val, ledMinBrightness, ledMaxBrightness);
    analogWrite(petalLedPin, currentLedBrightness);
    digitalWrite(centralLedPin, currentLedBrightness > 0 ? HIGH : LOW);
    ack("LED:" + String(currentLedBrightness), isBroadcast);
  }
  else if (command == "LEDRAMP") {
    int comma = value.indexOf(',');
    if (comma >= 0) {
      int target = value.substring(0, comma).toInt();
      unsigned long duration = value.substring(comma + 1).toInt();
      startLedRamp(target, duration);
      ack("LEDRAMP:" + String(target) + "," + String(duration), isBroadcast);
    } else {
      int target = value.toInt();
      startLedRamp(target, ledRampDefaultMs);
      ack("LEDRAMP:" + String(target) + "," + String(ledRampDefaultMs), isBroadcast);
    }
  }
  else if (command == "OPENLEDRAMP") {
    int comma = value.indexOf(',');
    if (comma >= 0) {
      int target = value.substring(0, comma).toInt();
      unsigned long duration = value.substring(comma + 1).toInt();
      currentState = OPENING;
      motorStarted = false;
      startLedRamp(target, duration);
      ack("OPENLEDRAMP:" + String(target) + "," + String(duration), isBroadcast);
    }
  }
  else if (command == "CLOSELEDRAMP") {
    int comma = value.indexOf(',');
    if (comma >= 0) {
      int target = value.substring(0, comma).toInt();
      unsigned long duration = value.substring(comma + 1).toInt();
      currentState = CLOSING;
      motorStarted = false;
      startLedRamp(target, duration);
      ack("CLOSELEDRAMP:" + String(target) + "," + String(duration), isBroadcast);
    }
  }
  else if (command == "INIT") {
    startupSequence();
    ack("INIT", isBroadcast);
  }
}

// ======== Quiet startup helpers (no RS-485 prints) ========
void startupSequence() {
  wiggleMotor();
  blinkLights(3);

  if (digitalRead(hallClosePin) == HIGH) {
    analogWrite(motorOpenPin, 0);
    analogWrite(motorClosePin, motorCloseSpeed);
    unsigned long start = millis();
    while (digitalRead(hallClosePin) == HIGH && (millis() - start) < motorSafetyTimeoutMs) {
      delay(10);
    }
    stopMotor();
  }

  analogWrite(petalLedPin, ledMinBrightness);
  digitalWrite(centralLedPin, LOW);
}

void wiggleMotor() {
  analogWrite(motorClosePin, 0);
  analogWrite(motorOpenPin, motorOpenSpeed);
  delay(wiggleTimeMs);
  stopMotor(); delay(200);

  analogWrite(motorOpenPin, 0);
  analogWrite(motorClosePin, motorCloseSpeed);
  delay(wiggleTimeMs);
  stopMotor(); delay(600);
}

void blinkLights(int n) {
  for (int i = 0; i < n; i++) {
    analogWrite(petalLedPin, ledMaxBrightness); delay(150);
    analogWrite(petalLedPin, 0);               delay(150);
  }
}
