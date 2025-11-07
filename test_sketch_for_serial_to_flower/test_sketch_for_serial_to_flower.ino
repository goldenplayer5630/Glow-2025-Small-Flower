#include <Arduino.h>

/*
  ========= RS-485 / Serial Test Skeleton =========
  - Accepts commands in the form: "<id>/<CMD[:ARGS]>"
  - Recognizes: OPEN, CLOSE, LED, LEDRAMP, OPENLEDRAMP, CLOSELEDRAMP, INIT, STATE?
  - Always responds with the **same ID** it received, e.g.:
      IN : "7/OPEN"
      OUT: "7/ACK:OPEN"
  - No hardware control; pure serial simulation.
*/

#define RS485 Serial

// If you wire a MAX485, you can set this to a real pin to toggle DE/RE.
// Leave at -1 to disable toggling and just use plain Serial.
const int rs485DirPin = -1;

// --- Helpers ---
static void rs485Begin(unsigned long baud = 115200) {
  if (rs485DirPin >= 0) {
    pinMode(rs485DirPin, OUTPUT);
    digitalWrite(rs485DirPin, LOW);   // RX by default
  }
  RS485.begin(baud);
  delay(20);
}

static void rs485SendLine(const String& s) {
  if (rs485DirPin >= 0) {
    digitalWrite(rs485DirPin, HIGH);  // enable TX
    delayMicroseconds(8);
  }
  RS485.print(s);
  RS485.print('\n');
  RS485.flush();
  if (rs485DirPin >= 0) {
    digitalWrite(rs485DirPin, LOW);   // back to RX
  }
}

// Echo ACK with same ID, optionally with payload (e.g., "LED:120")
static void ackWithId(int id, const String& payload) {
  rs485SendLine(String(id) + "/ACK:" + payload);
}

static void sendStateWithId(int id) {
  // Simulated state (fixed values)
  rs485SendLine(String(id) + "/STATE:ISOPEN=0,BUSY=0,LED=0");
}

// --- Command processing ---
static void processLine(String msg) {
  msg.trim();
  if (msg.length() == 0) return;

  // Expect "<id>/<rest>"
  int slash = msg.indexOf('/');
  if (slash <= 0) return; // no id or malformed

  // Parse ID (keep as-is even if 0 or not "this" node)
  int id = msg.substring(0, slash).toInt();
  String rest = msg.substring(slash + 1);

  // Split cmd[:val]
  int colon = rest.indexOf(':');
  String cmd = (colon < 0) ? rest : rest.substring(0, colon);
  String val = (colon < 0) ? ""   : rest.substring(colon + 1);

  cmd.trim();
  val.trim();

  // Normalize to uppercase (for robustness)
  cmd.toUpperCase();

  // Recognized commands -> always ACK with the same ID
  if (cmd == "OPEN") {
    ackWithId(id, "OPEN");
  }
  else if (cmd == "CLOSE") {
    ackWithId(id, "CLOSE");
  }
  else if (cmd == "LED") {
    // Echo value if present, else just LED
    ackWithId(id, val.length() ? ("LED:" + val) : "LED");
  }
  else if (cmd == "LEDRAMP") {
    ackWithId(id, val.length() ? ("LEDRAMP:" + val) : "LEDRAMP");
  }
  else if (cmd == "OPENLEDRAMP") {
    ackWithId(id, val.length() ? ("OPENLEDRAMP:" + val) : "OPENLEDRAMP");
  }
  else if (cmd == "CLOSELEDRAMP") {
    ackWithId(id, val.length() ? ("CLOSELEDRAMP:" + val) : "CLOSELEDRAMP");
  }
  else if (cmd == "INIT") {
    ackWithId(id, "INIT");
  }
  else if (cmd == "STATE?") {
    // Respond with a STATE line (not an ACK), keeping same ID
    sendStateWithId(id);
  }
  else {
    // Unknown command: still ACK the literal command for easy debugging
    ackWithId(id, "UNKNOWN:" + rest);
  }
}

// --- Line reader (CR/LF tolerant, length-limited) ---
static void pollSerial() {
  static String in;
  while (RS485.available()) {
    char c = (char)RS485.read();
    if (c == '\n' || c == '\r') {
      if (in.length() > 0) processLine(in);
      in = "";
    } else {
      // Guard against runaway input
      if (in.length() < 128) in += c;
    }
  }
}

// --- Arduino entry points ---
void setup() {
  rs485Begin(9600);
  // Optional banner to confirm the sketch is alive
  rs485SendLine("0/HELLO:TEST_SKELETON_READY");
}

void loop() {
  pollSerial();
}
