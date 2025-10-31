#include <FastLED.h>

/* -------- Debug flag -------- */
#define DEBUG 1
#define DBG_BEGIN(baud)   do{ if (DEBUG){ Serial.begin(baud); while(!Serial){} } }while(0)
#define DBG_PRINT(x)      do{ if (DEBUG){ Serial.print(x); } }while(0)
#define DBG_PRINTLN(x)    do{ if (DEBUG){ Serial.println(x); } }while(0)
/* single-arg print with precision */
#define DBG_PRINT2(x,prec) do{ if (DEBUG){ Serial.print((x),(prec)); } }while(0)

/* -------- Node identity -------- */
const int myID = 1;                 // set unique ID

/* -------- RS485 on UNO (Serial) --------
   NOTE: UNO has only one HW UART. We share it:
   - Debug prints occur while DE/RE = LOW (receiver only), so they won't hit the bus.
   - ACKs toggle DE/RE HIGH for the duration of the send. */
#define RS485 Serial
const int rs485DirPin = 2;          // MAX485 DE/RE (tied together)

/* -------- Motor (IBT-2) -------- */
const int RPWM = 5;                 // extend (PWM)
const int LPWM = 6;                 // retract (PWM)

/* -------- Limits (ACTIVE LOW) -------- */
const int R_LIMIT_A = 8;
const int L_LIMIT_A = 9;
const int R_LIMIT_B = 10;
const int L_LIMIT_B = 11;
const int R_LIMIT_C = 12;
const int L_LIMIT_C = 13;
const bool USE_PULLUPS = true;

/* -------- Ultrasonic -------- */
const int US_TRIG = A2;             // off D2 (DE/RE)
const int US_ECHO = A3;
const unsigned long US_PULSE_TIMEOUT_US   = 30000;
const unsigned long US_SAMPLE_INTERVAL_MS = 30;
const float         US_MIN_DELTA_CM       = 0.5f;
const unsigned long US_DEBUG_INTERVAL_MS  = 100; // how often to print while moving
unsigned long usLastDebugMs = 0;

/* -------- Motor timings -------- */
int  defaultSpeed = 200;            // 0..255
const unsigned long motorSafetyTimeoutMs = 10000;

/* -------- FastLED WS2811: INNER + OUTER -------- */
#define LED_PIN_INNER    3
#define LED_PIN_OUTER    4
#define NUM_LEDS_INNER   80   // <-- set to your inner ring count
#define NUM_LEDS_OUTER   80   // <-- set to your outer ring count
#define LED_TYPE         WS2811
#define COLOR_ORDER      BRG

CRGB ledsInner[NUM_LEDS_INNER];
CRGB ledsOuter[NUM_LEDS_OUTER];

// Per-ring color + brightness (use global brightness=255; scale per ring)
uint8_t iR=255, iG=255, iB=255, iBright=0;
uint8_t oR=255, oG=255, oB=255, oBright=0;

// Non-blocking brightness ramps (per ring)
bool           iRamp=false, oRamp=false;
uint8_t        iStart=0,   oStart=0;
uint8_t        iTarget=0,  oTarget=0;
unsigned long  iStartMs=0, oStartMs=0;
unsigned long  iDurMs=0,   oDurMs=0;

/* -------- State -------- */
enum Motion { IDLE, EXTENDING, RETRACTING };
Motion motion = IDLE;
unsigned long motorStartMs = 0;

// Ultrasonic state
float         usLastDistCm = NAN;
int           usNoProgressCount = 0;
unsigned long usLastSampleMs = 0;

/* ================= Helpers ================= */
inline bool limitActiveLOW(int pin){ return digitalRead(pin) == LOW; }
bool anyExtendLimit(){
  return limitActiveLOW(R_LIMIT_A) || limitActiveLOW(R_LIMIT_B) || limitActiveLOW(R_LIMIT_C);
}
bool anyRetractLimit(){
  return limitActiveLOW(L_LIMIT_A) || limitActiveLOW(L_LIMIT_B) || limitActiveLOW(L_LIMIT_C);
}

void stopMotor(){
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  if (motion != IDLE) DBG_PRINTLN(F("[DBG] Motor -> STOP"));
  motion = IDLE;
  usLastDistCm = NAN;
  usNoProgressCount = 0;
  usLastSampleMs = 0;
}

void extendMotor(int speed){
  if (anyExtendLimit()) { stopMotor(); DBG_PRINTLN(F("[DBG] Blocked: EXT limit active")); return; }
  analogWrite(LPWM, 0);
  analogWrite(RPWM, constrain(speed, 0, 255));
  motion = EXTENDING;
  motorStartMs = millis();
  usLastDistCm = NAN; usNoProgressCount = 0; usLastSampleMs = 0;
  DBG_PRINT(F("[DBG] Motor -> EXT speed=")); DBG_PRINTLN(speed);
}

void retractMotor(int speed){
  if (anyRetractLimit()) { stopMotor(); DBG_PRINTLN(F("[DBG] Blocked: RET limit active")); return; }
  analogWrite(RPWM, 0);
  analogWrite(LPWM, constrain(speed, 0, 255));
  motion = RETRACTING;
  motorStartMs = millis();
  usLastDistCm = NAN; usNoProgressCount = 0; usLastSampleMs = 0;
  DBG_PRINT(F("[DBG] Motor -> RET speed=")); DBG_PRINTLN(speed);
}

/* -------- FastLED helpers (per-ring) -------- */
inline CRGB scaledColor(uint8_t r, uint8_t g, uint8_t b, uint8_t bright){
  CRGB c(r,g,b);
  c.nscale8_video(bright);   // per-ring brightness
  return c;
}
inline void showRings(){
  // Global brightness fixed to 255; we scale per ring
  fill_solid(ledsInner, NUM_LEDS_INNER, scaledColor(iR,iG,iB,iBright));
  fill_solid(ledsOuter, NUM_LEDS_OUTER, scaledColor(oR,oG,oB,oBright));
  FastLED.show();
}
inline void setRGBInner(uint8_t r,uint8_t g,uint8_t b){ iR=r; iG=g; iB=b; showRings(); }
inline void setRGBOuter(uint8_t r,uint8_t g,uint8_t b){ oR=r; oG=g; oB=b; showRings(); }
inline void setBrightInner(uint8_t b){ iBright=b; showRings(); }
inline void setBrightOuter(uint8_t b){ oBright=b; showRings(); }

inline void startRampInner(uint8_t target, unsigned long durMs){
  iStart = iBright; iTarget = target; iStartMs = millis(); iDurMs = durMs?durMs:1; iRamp = true;
  DBG_PRINT(F("[DBG] RGBRAMP I -> ")); DBG_PRINT((int)target); DBG_PRINT(F(" over ")); DBG_PRINTLN(durMs);
}
inline void startRampOuter(uint8_t target, unsigned long durMs){
  oStart = oBright; oTarget = target; oStartMs = millis(); oDurMs = durMs?durMs:1; oRamp = true;
  DBG_PRINT(F("[DBG] RGBRAMP O -> ")); DBG_PRINT((int)target); DBG_PRINT(F(" over ")); DBG_PRINTLN(durMs);
}
void updateRamps(){
  unsigned long now = millis();
  if (iRamp){
    unsigned long e = now - iStartMs;
    if (e >= iDurMs){ iBright = iTarget; iRamp=false; showRings(); DBG_PRINTLN(F("[DBG] RGBRAMP I complete")); }
    else { float t=(float)e/(float)iDurMs; int v=(int)iStart + (int)((int)iTarget-(int)iStart)*t; if(v<0)v=0; if(v>255)v=255; iBright=(uint8_t)v; showRings(); }
  }
  if (oRamp){
    unsigned long e = now - oStartMs;
    if (e >= oDurMs){ oBright = oTarget; oRamp=false; showRings(); DBG_PRINTLN(F("[DBG] RGBRAMP O complete")); }
    else { float t=(float)e/(float)oDurMs; int v=(int)oStart + (int)((int)oTarget-(int)oStart)*t; if(v<0)v=0; if(v>255)v=255; oBright=(uint8_t)v; showRings(); }
  }
}

/* -------- Ultrasonic -------- */
float readDistanceCm(){
  digitalWrite(US_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  unsigned long echo = pulseIn(US_ECHO, HIGH, US_PULSE_TIMEOUT_US);
  if (echo == 0) return -1.0f;
  return echo / 58.0f;
}
void checkUltrasonicProgress(){
  if (motion == IDLE) return;

  unsigned long now = millis();

  // Keep sampling cadence for stall detection
  if (now - usLastSampleMs < US_SAMPLE_INTERVAL_MS) return;
  usLastSampleMs = now;

  float d = readDistanceCm();

  // Print raw distance periodically (even if invalid)
  if (DEBUG && (now - usLastDebugMs >= US_DEBUG_INTERVAL_MS)) {
    usLastDebugMs = now;
    if (d <= 0.0f) {
      DBG_PRINTLN(F("[US] d=--- (timeout)"));
    } else {
      DBG_PRINT(F("[US] d=")); DBG_PRINT(d); DBG_PRINT(F(" cm"));
      // If we already have a previous sample, also show delta + trend expectation
      if (!isnan(usLastDistCm)) {
        float delta = d - usLastDistCm;
        DBG_PRINT(F("  \xE2\x88\x86=")); DBG_PRINT2(delta, 2); // Δ with 2 decimals
        DBG_PRINT(F("  expect: "));
        DBG_PRINT((motion == EXTENDING) ? F("\xE2\x88\x86>=") : F("\xE2\x88\x86<="));
        DBG_PRINTLN(US_MIN_DELTA_CM);
      } else {
        DBG_PRINTLN(F("  (first)"));
      }
    }
  }

  // If no valid reading, don't update stall logic
  if (d <= 0.0f) return;

  // Stall detection vs expected trend
  if (isnan(usLastDistCm)){
    usLastDistCm = d;
    return;
  }

  float delta = d - usLastDistCm;
  bool ok = (motion==EXTENDING) ? (delta >= US_MIN_DELTA_CM)
                                : (delta <= -US_MIN_DELTA_CM);

  if (ok){
    if (usNoProgressCount) {
      if (DEBUG){
        DBG_PRINT(F("[US] progress OK; reset no-progress count from "));
        DBG_PRINT(usNoProgressCount);
        DBG_PRINTLN(F(" → 0"));
      }
    }
    usNoProgressCount = 0;
  } else {
    usNoProgressCount++;
    if (DEBUG){
      DBG_PRINT(F("[US] NO-PROGRESS #")); DBG_PRINT(usNoProgressCount);
      DBG_PRINT(F("  (d=")); DBG_PRINT(d); DBG_PRINT(F(" cm, \xE2\x88\x86=")); DBG_PRINT2(delta, 2); DBG_PRINTLN(')');
    }
    if (usNoProgressCount >= 3){
      if (DEBUG) DBG_PRINTLN(F("[US] TRIP → stopping motor"));
      stopMotor();
      return;
    }
  }

  usLastDistCm = d;
}

/* -------- RS485 -------- */
void rs485Begin(){
  pinMode(rs485DirPin, OUTPUT);
  digitalWrite(rs485DirPin, LOW); // RX
  RS485.begin(115200);
  delay(30);
  DBG_PRINTLN(F("[DBG] RS485 up @115200 (DE/RE LOW)"));
}
void rs485SendLine(const String &s){
  digitalWrite(rs485DirPin, HIGH); delayMicroseconds(8);
  RS485.print(s); RS485.print('\n'); RS485.flush();
  digitalWrite(rs485DirPin, LOW);
  DBG_PRINT(F("[DBG] TX ACK: ")); DBG_PRINTLN(s);
}
// short ACK (only for direct, not broadcast)
inline void ack(bool isBroadcast){ if(!isBroadcast) rs485SendLine(String(myID)+"/A"); }

/* ================= Setup / Loop ================= */
void setup(){
  DBG_BEGIN(115200);

  // Motor
  pinMode(RPWM, OUTPUT); pinMode(LPWM, OUTPUT); stopMotor();

  // Limits
  if (USE_PULLUPS){
    pinMode(R_LIMIT_A, INPUT_PULLUP); pinMode(L_LIMIT_A, INPUT_PULLUP);
    pinMode(R_LIMIT_B, INPUT_PULLUP); pinMode(L_LIMIT_B, INPUT_PULLUP);
    pinMode(R_LIMIT_C, INPUT_PULLUP); pinMode(L_LIMIT_C, INPUT_PULLUP);
  } else {
    pinMode(R_LIMIT_A, INPUT); pinMode(L_LIMIT_A, INPUT);
    pinMode(R_LIMIT_B, INPUT); pinMode(L_LIMIT_B, INPUT);
    pinMode(R_LIMIT_C, INPUT); pinMode(L_LIMIT_C, INPUT);
  }

  // Ultrasonic
  pinMode(US_TRIG, OUTPUT); pinMode(US_ECHO, INPUT); digitalWrite(US_TRIG, LOW);

  // FastLED
  FastLED.addLeds<LED_TYPE, LED_PIN_INNER, COLOR_ORDER>(ledsInner, NUM_LEDS_INNER);
  FastLED.addLeds<LED_TYPE, LED_PIN_OUTER, COLOR_ORDER>(ledsOuter, NUM_LEDS_OUTER);
  FastLED.setBrightness(255);   // global max; we scale per ring
  showRings();

  // RS485
  rs485Begin();

  DBG_PRINTLN(F("[DBG] Setup complete"));
}

void loop(){
  handleBus();
  handleMotorState();
  checkUltrasonicProgress();
  updateRamps();
}

/* -------- Motor state safety -------- */
void handleMotorState(){
  if (motion == IDLE) return;
  if (millis() - motorStartMs >= motorSafetyTimeoutMs){ DBG_PRINTLN(F("[DBG] Safety timeout")); stopMotor(); return; }
  if (motion == EXTENDING && anyExtendLimit()){ DBG_PRINTLN(F("[DBG] EXT limit triggered")); stopMotor(); return; }
  if (motion == RETRACTING && anyRetractLimit()){ DBG_PRINTLN(F("[DBG] RET limit triggered")); stopMotor(); return; }
}

/* -------- Helpers to parse ring token -------- */
char parseRing(const String& token){
  String t = token; t.trim(); t.toUpperCase();
  if (t == "I" || t == "INNER") return 'I';
  if (t == "O" || t == "OUTER") return 'O';
  if (t == "B" || t == "BOTH")  return 'B';
  return '?';
}
bool isNumberLike(const String& s){
  if (s.length()==0) return false;
  for (uint8_t i=0;i<s.length();++i){
    char c=s[i]; if (!(c=='-'||c=='+'|| (c>='0'&&c<='9'))) return false;
  }
  return true;
}

/* -------- Protocol: "<id>/<CMD>[:val]" --------
   EXT[:spd] | OPEN[:spd]
   RET[:spd] | CLOSE[:spd]
   STOP
   SPEED:n
   RGB:[ring,]r,g,b        ring = I|INNER, O|OUTER, B|BOTH (default BOTH if omitted)
   BRIGHT:[ring,]n
   RGBRAMP:[ring,]n,ms
*/
void handleBus(){
  static String in;
  while (RS485.available()){
    char c = (char)RS485.read();
    if (c=='\n' || c=='\r'){ if(in.length()>0) processLine(in); in=""; }
    else if (in.length()<200){ in+=c; }
  }
}

void processLine(String msg){
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
  command.toUpperCase();

  DBG_PRINT(F("[DBG] RX ")); DBG_PRINT(id); DBG_PRINT('/'); DBG_PRINT(command);
  if (value.length()){ DBG_PRINT(':'); DBG_PRINT(value); } DBG_PRINTLN("");

  if (command == "EXT" || command == "OPEN"){
    int spd = value.length()? constrain(value.toInt(), 0, 255) : defaultSpeed;
    defaultSpeed = spd; extendMotor(spd); ack(isBroadcast); return;
  }
  if (command == "RET" || command == "CLOSE"){
    int spd = value.length()? constrain(value.toInt(), 0, 255) : defaultSpeed;
    defaultSpeed = spd; retractMotor(spd); ack(isBroadcast); return;
  }
  if (command == "STOP"){ stopMotor(); ack(isBroadcast); return; }
  if (command == "SPEED"){ defaultSpeed = constrain(value.toInt(), 0, 255); ack(isBroadcast); return; }

  // ---------- RGB ----------
  if (command == "RGB"){
    int c1 = value.indexOf(',');
    if (c1 < 0) return; // need at least two tokens
    String first = value.substring(0,c1);
    char ring = '?';
    int startIdx = 0;

    if (!isNumberLike(first)){
      ring = parseRing(first);
      startIdx = c1+1;
    } else {
      ring = 'B'; // default BOTH
      startIdx = 0;
    }

    // Parse r,g,b starting from startIdx
    String restVals = value.substring(startIdx);
    int p1 = restVals.indexOf(',');
    int p2 = (p1>=0) ? restVals.indexOf(',', p1+1) : -1;
    if (p1<0 || p2<0) return;

    int r = constrain(restVals.substring(0,p1).toInt(),0,255);
    int g = constrain(restVals.substring(p1+1,p2).toInt(),0,255);
    int b = constrain(restVals.substring(p2+1).toInt(),0,255);

    if (ring=='I'){ setRGBInner((uint8_t)r,(uint8_t)g,(uint8_t)b); }
    else if (ring=='O'){ setRGBOuter((uint8_t)r,(uint8_t)g,(uint8_t)b); }
    else { setRGBInner((uint8_t)r,(uint8_t)g,(uint8_t)b); setRGBOuter((uint8_t)r,(uint8_t)g,(uint8_t)b); }

    ack(isBroadcast); return;
  }

  // ---------- BRIGHT ----------
  if (command == "BRIGHT"){
    if (value.length()==0) return;
    int c1 = value.indexOf(',');
    if (c1 < 0){
      // BRIGHT:n  (both)
      uint8_t n = (uint8_t)constrain(value.toInt(),0,255);
      setBrightInner(n); setBrightOuter(n); ack(isBroadcast); return;
    } else {
      // BRIGHT:ring,n
      char ring = parseRing(value.substring(0,c1));
      uint8_t n = (uint8_t)constrain(value.substring(c1+1).toInt(),0,255);
      if (ring=='I') setBrightInner(n);
      else if (ring=='O') setBrightOuter(n);
      else { setBrightInner(n); setBrightOuter(n); }
      ack(isBroadcast); return;
    }
  }

  // ---------- RGBRAMP ----------
  if (command == "RGBRAMP"){
    if (value.length()==0) return;
    int c1 = value.indexOf(',');
    if (c1 < 0) return; // need at least one comma

    // Maybe RGBRAMP:n,ms   (both)
    String first = value.substring(0,c1);
    if (isNumberLike(first)){
      uint8_t tgt = (uint8_t)constrain(first.toInt(),0,255);
      unsigned long dur = (unsigned long)max(0, value.substring(c1+1).toInt());
      startRampInner(tgt, dur); startRampOuter(tgt, dur); ack(isBroadcast); return;
    }

    // RGBRAMP:ring,n,ms
    char ring = parseRing(first);
    String restVals = value.substring(c1+1);
    int c2 = restVals.indexOf(',');
    if (c2 < 0) return;
    uint8_t tgt = (uint8_t)constrain(restVals.substring(0,c2).toInt(),0,255);
    unsigned long dur = (unsigned long)max(0, restVals.substring(c2+1).toInt());

    if (ring=='I') startRampInner(tgt, dur);
    else if (ring=='O') startRampOuter(tgt, dur);
    else { startRampInner(tgt, dur); startRampOuter(tgt, dur); }
    ack(isBroadcast); return;
  }

  // unknown -> silent (no ACK, no prints)
}
