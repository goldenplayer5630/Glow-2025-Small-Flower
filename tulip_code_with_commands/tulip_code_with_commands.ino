const int myID = 2;

// === Pin Definitions ===
const int motorOpenPin = 9;
const int motorClosePin = 10;

const int hallOpenPin = 12;
const int hallClosePin = 11;

const int buttonPin = 13;

const int petalLedPin = 3;
const int centralLedPin = 8;

// === Motor Speeds ===
const int motorOpenSpeed = 130;
const int motorCloseSpeed = 250;

const unsigned long wiggleTimeMs = 100;

// === LED Brightness ===
const int ledMinBrightness = 0;
const int ledMaxBrightness = 120;

// === LED Ramp Defaults ===
const unsigned long ledRampDuration = 1500;  // ms (used when LEDRAMP has no duration)
const int ledStep = 5;

const int rs485DirPin = 2;

// Debounce
const unsigned long debounceDelay = 250;

// === System State ===
enum SystemState { WAITING, OPENING, CLOSING };
SystemState currentState = WAITING;

bool lastButtonState = LOW;
unsigned long lastButtonTime = 0;

int currentLedBrightness = ledMinBrightness;
bool motorStarted = false;

// === LED Ramp State ===
bool ledRamping = false;
int ledRampStartBrightness = 0;
int ledTargetBrightness = ledMinBrightness;
unsigned long ledRampStartTime = 0;
unsigned long ledRampDurationMs = 0;

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

  rs485Begin();  // <-- instead of Serial.begin()

  logMsg("System initialized.");
  logMsg("Ready for button press or serial command.");
  startupSequence();
}

void loop() {
  unsigned long now = millis();
  bool buttonPressed = (digitalRead(buttonPin) == LOW);

  // Handle serial
  handleSerialCommand();

  // Handle button
  if (buttonPressed && lastButtonState == HIGH && (now - lastButtonTime > debounceDelay)) {
    lastButtonTime = now;

    if (currentState == WAITING) {
      if (digitalRead(hallClosePin) == LOW) {
        logMsg("Button pressed at CLOSED — starting OPEN sequence.");
        currentState = OPENING;
        motorStarted = false;
      } else if (digitalRead(hallOpenPin) == LOW) {
        logMsg("Button pressed at OPEN — starting CLOSE sequence.");
        currentState = CLOSING;
        motorStarted = false;
      } else {
        logMsg("Ignored — neither HES active.");
      }
    } else {
      logMsg("Force stop requested.");
      stopMotor();
      currentState = WAITING;
    }
  }
  lastButtonState = buttonPressed ? LOW : HIGH;

  // Motor state machine
  handleMotorState();

  // LED ramp state machine
  updateLedRamp();
}

// === Motor Control ===
void handleMotorState() {
  if (currentState == OPENING) {
    if (!motorStarted) {
      logMsg("Motor opening...");
      analogWrite(motorClosePin, motorOpenSpeed);
      analogWrite(motorOpenPin, 0);
      motorStarted = true;
    }

    if (digitalRead(hallOpenPin) == LOW) {
      logMsg("OPEN HES triggered — stopping motor.");
      stopMotor();
      currentState = WAITING;
    }
  }

  if (currentState == CLOSING) {
    if (!motorStarted) {
      logMsg("Motor closing...");
      analogWrite(motorOpenPin, motorCloseSpeed);
      analogWrite(motorClosePin, 0);
      motorStarted = true;
    }

    if (digitalRead(hallClosePin) == LOW) {
      logMsg("CLOSE HES triggered — stopping motor.");
      stopMotor();
      currentState = WAITING;
    }
  }
}

void stopMotor() {
  analogWrite(motorOpenPin, 0);
  analogWrite(motorClosePin, 0);
  motorStarted = false;
  logMsg("Motor stopped.");
}

// === LED Ramp Control ===
void startLedRamp(int target, unsigned long durationMs) {
  target = constrain(target, ledMinBrightness, ledMaxBrightness);
  ledRampStartBrightness = currentLedBrightness;
  ledTargetBrightness = target;
  ledRampDurationMs = durationMs;
  ledRampStartTime = millis();
  ledRamping = true;

  logMsg("Starting LED ramp to " + String(target) + " over " + String(durationMs) + " ms");
}

void updateLedRamp() {
  if (!ledRamping) return;

  unsigned long now = millis();
  unsigned long elapsed = now - ledRampStartTime;

  if (elapsed >= ledRampDurationMs) {
    currentLedBrightness = ledTargetBrightness;
    analogWrite(petalLedPin, currentLedBrightness);
    digitalWrite(centralLedPin, currentLedBrightness > 0 ? HIGH : LOW);
    ledRamping = false;
    logMsg("LED ramp complete → " + String(currentLedBrightness));
    return;
  }

  float progress = (float)elapsed / (float)ledRampDurationMs;
  int newBrightness = ledRampStartBrightness +
                      (int)((ledTargetBrightness - ledRampStartBrightness) * progress);

  currentLedBrightness = newBrightness;
  analogWrite(petalLedPin, currentLedBrightness);
  digitalWrite(centralLedPin, currentLedBrightness > 0 ? HIGH : LOW);
}

// === Serial Command Handling ===
void handleSerialCommand() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    int slashIndex = msg.indexOf('/');
    if (slashIndex == -1) return;

    int id = msg.substring(0, slashIndex).toInt();
    if (id != myID && id != 0) return;

    String cmd = msg.substring(slashIndex + 1);
    int colonIndex = cmd.indexOf(':');
    String command = (colonIndex == -1) ? cmd : cmd.substring(0, colonIndex);
    String value   = (colonIndex == -1) ? ""  : cmd.substring(colonIndex + 1);

    if (command == "OPEN") {
      currentState = OPENING;
      motorStarted = false;
      ack("OPEN");
    }
    else if (command == "CLOSE") {
      currentState = CLOSING;
      motorStarted = false;
      ack("CLOSE");
    }
    else if (command == "LED") {
      int val = value.toInt();
      currentLedBrightness = constrain(val, ledMinBrightness, ledMaxBrightness);
      analogWrite(petalLedPin, currentLedBrightness);
      digitalWrite(centralLedPin, currentLedBrightness > 0 ? HIGH : LOW);
      ack("LED:" + String(currentLedBrightness));
    }
    else if (command == "LEDRAMP") {
      int commaIndex = value.indexOf(',');
      if (commaIndex != -1) {
        int target = value.substring(0, commaIndex).toInt();
        unsigned long duration = value.substring(commaIndex + 1).toInt();
        startLedRamp(target, duration);
        ack("LEDRAMP:" + String(target) + "," + String(duration));
      } else {
        int target = value.toInt();
        startLedRamp(target, ledRampDuration);
        ack("LEDRAMP:" + String(target));
      }
    }
    else if (command == "OPENLEDRAMP") {
      int commaIndex = value.indexOf(',');
      if (commaIndex != -1) {
        int target = value.substring(0, commaIndex).toInt();
        unsigned long duration = value.substring(commaIndex + 1).toInt();
        currentState = OPENING;
        motorStarted = false;
        startLedRamp(target, duration);
        ack("OPENLEDRAMP:" + String(target) + "," + String(duration));
      }
    }
    else if (command == "CLOSELEDRAMP") {
      int commaIndex = value.indexOf(',');
      if (commaIndex != -1) {
        int target = value.substring(0, commaIndex).toInt();
        unsigned long duration = value.substring(commaIndex + 1).toInt();
        currentState = CLOSING;
        motorStarted = false;
        startLedRamp(target, duration);
        ack("CLOSELEDRAMP:" + String(target) + "," + String(duration));
      }
    }
    else if (command == "INIT") {
      startupSequence();
      ack("INIT");
    }
  }
}

void rs485Begin() {
  pinMode(rs485DirPin, OUTPUT);
  digitalWrite(rs485DirPin, LOW); // idle = receive
  Serial.begin(9600);
}

// Send helper: toggles TX driver briefly
void rs485SendLine(const String &s) {
  digitalWrite(rs485DirPin, HIGH);   // enable driver
  delayMicroseconds(10);             // allow driver to turn on
  Serial.println(s);
  Serial.flush();                    // wait until all is sent
  digitalWrite(rs485DirPin, LOW);    // back to receive
}

// === Startup Helpers ===
void startupSequence() {
  logMsg("Startup sequence initiated: moving to CLOSED position...");

  wiggleMotor();
  blinkLights(3);

  if (digitalRead(hallClosePin) == HIGH) {
    logMsg("Closing motor (startup)...");
    analogWrite(motorOpenPin, motorCloseSpeed);
    analogWrite(motorClosePin, 0);

    while (digitalRead(hallClosePin) == HIGH) {
      delay(10);
    }

    stopMotor();
    logMsg("Closed HES triggered — flower is now closed.");
  } else {
    logMsg("Already in closed position.");
  }

  analogWrite(petalLedPin, ledMinBrightness);
  digitalWrite(centralLedPin, LOW);
}

void wiggleMotor() {
  analogWrite(motorClosePin, 0);
  analogWrite(motorOpenPin, motorOpenSpeed);
  delay(wiggleTimeMs);
  stopMotor();
  delay(200);

  analogWrite(motorOpenPin, 0);
  analogWrite(motorClosePin, motorCloseSpeed);
  delay(wiggleTimeMs);
  stopMotor();
  delay(1000);
}

void blinkLights(int amount) {
  for (int i = 0; i < amount; i++) {
    analogWrite(petalLedPin, ledMaxBrightness);
    delay(200);
    analogWrite(petalLedPin, 0);
    delay(200);
  }
}

// === Logging ===
void logMsg(String msg) {
  Serial.print(myID);
  Serial.print(": ");
  Serial.println(msg);
}

void ack(String cmd) {
  Serial.print(myID);
  Serial.print("/ACK:");
  Serial.println(cmd);
}
