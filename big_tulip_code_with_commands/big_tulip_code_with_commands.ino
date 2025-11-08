#include <FastLED.h>

/* -------- Debug flag -------- */
#define DEBUG 0
#define DBG_BEGIN(baud)   do{ if (DEBUG){ Serial.begin(baud); while(!Serial){} } }while(0)
#define DBG_PRINT(x)      do{ if (DEBUG){ Serial.print(x); } }while(0)
#define DBG_PRINTLN(x)    do{ if (DEBUG){ Serial.println(x); } }while(0)
#define DBG_PRINT2(x,prec) do{ if (DEBUG){ Serial.print((x),(prec)); } }while(0)

/* -------- Feature toggles -------- */
// Set to 1 to enable ultrasonic stall safety; 0 disables it entirely.
#define USE_US_SAFETY 0

/* -------- Node identity -------- */
const int myID = 1;                 // set unique ID

/* -------- RS485 on UNO (Serial) -------- */
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
const unsigned long motorSafetyTimeoutMs = 15000;
bool ignoreLimits = false;

/* -------- FastLED WS2811: INNER + OUTER -------- */
#define LED_PIN_INNER    3
#define LED_PIN_OUTER    4
#define NUM_LEDS_INNER   112   // <-- set to your inner ring count
#define NUM_LEDS_OUTER   119   // <-- set to your outer ring count
#define LED_TYPE         WS2811
#define COLOR_ORDER      BRG

CRGB ledsInner[NUM_LEDS_INNER];
CRGB ledsOuter[NUM_LEDS_OUTER];

/* -------- Per-ring base color (set by RGB/RGBIN/RGBOUT) + intensity --------
   Base color is 0..255. Intensity is 0..255 (mapped from UI 0..120).
   The final shown color = BaseRGB * (Intensity/255), done via nscale8_video.
*/
uint8_t iBaseR=255, iBaseG=255, iBaseB=255;   // inner base color
uint8_t oBaseR=255, oBaseG=255, oBaseB=255;   // outer base color
uint8_t iBright=0;  // inner intensity 0..255
uint8_t oBright=0;  // outer intensity 0..255

// Non-blocking intensity ramps (per ring)
bool           iRamp=false, oRamp=false;
uint8_t        iStart=0,   oStart=0;
uint8_t        iTarget=0,  oTarget=0;
unsigned long  iStartMs=0, oStartMs=0;
unsigned long  iDurMs=0,   oDurMs=0;

/* -------- State -------- */
enum Motion { IDLE, EXTENDING, RETRACTING };
Motion motion = IDLE;
unsigned long motorStartMs = 0;

// Track last commanded PWM values for STATUS
int lastRPWM = 0;
int lastLPWM = 0;

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
  lastRPWM = 0;
  lastLPWM = 0;
  if (motion != IDLE) DBG_PRINTLN(F("[DBG] Motor -> STOP"));
  motion = IDLE;

  ignoreLimits = false;
  // Clear ultrasonic progress tracking (safe even if disabled)
  usLastDistCm = NAN;
  usNoProgressCount = 0;
  usLastSampleMs = 0;
}

void extendMotor(int speed){
  ignoreLimits = false;  
  if (anyExtendLimit()) { stopMotor(); DBG_PRINTLN(F("[DBG] Blocked: EXT limit active")); return; }
  analogWrite(LPWM, 0);
  analogWrite(RPWM, constrain(speed, 0, 255));
  lastLPWM = 0;
  lastRPWM = constrain(speed, 0, 255);
  motion = EXTENDING;
  motorStartMs = millis();
  usLastDistCm = NAN; usNoProgressCount = 0; usLastSampleMs = 0;
  DBG_PRINT(F("[DBG] Motor -> EXT speed=")); DBG_PRINTLN(speed);
}

void retractMotor(int speed){
  ignoreLimits = false;  
  if (anyRetractLimit()) { stopMotor(); DBG_PRINTLN(F("[DBG] Blocked: RET limit active")); return; }
  analogWrite(RPWM, 0);
  analogWrite(LPWM, constrain(speed, 0, 255));
  lastRPWM = 0;
  lastLPWM = constrain(speed, 0, 255);
  motion = RETRACTING;
  motorStartMs = millis();
  usLastDistCm = NAN; usNoProgressCount = 0; usLastSampleMs = 0;
  DBG_PRINT(F("[DBG] Motor -> RET speed=")); DBG_PRINTLN(speed);
}

void extendMotorOverride(int speed){
  ignoreLimits = true; 
  // Ignore extend-limit switches on purpose
  analogWrite(LPWM, 0);
  analogWrite(RPWM, constrain(speed, 0, 255));
  lastLPWM = 0;
  lastRPWM = constrain(speed, 0, 255);
  motion = EXTENDING;
  motorStartMs = millis();
  usLastDistCm = NAN; usNoProgressCount = 0; usLastSampleMs = 0;
  DBG_PRINT(F("[DBG] Motor -> EXT (override) speed=")); DBG_PRINTLN(speed);
}

void retractMotorOverride(int speed){
  ignoreLimits = true; 
  // Ignore retract-limit switches on purpose
  analogWrite(RPWM, 0);
  analogWrite(LPWM, constrain(speed, 0, 255));
  lastRPWM = 0;
  lastLPWM = constrain(speed, 0, 255);
  motion = RETRACTING;
  motorStartMs = millis();
  usLastDistCm = NAN; usNoProgressCount = 0; usLastSampleMs = 0;
  DBG_PRINT(F("[DBG] Motor -> RET (override) speed=")); DBG_PRINTLN(speed);
}

/* -------- Intensity scaling -------- */
// Map 0..120 UI intensity to 0..255 internal (rounded)
static inline uint8_t map120to255(int v) {
  v = constrain(v, 0, 120);
  return (uint8_t)((v * 255 + 60) / 120);
}

/* -------- FastLED helpers (per-ring) -------- */
inline CRGB scaledColor(uint8_t r, uint8_t g, uint8_t b, uint8_t bright){
  CRGB c(r,g,b);
  c.nscale8_video(bright);   // per-ring intensity
  return c;
}
inline void showRings(){
  // Global brightness fixed to 255; we scale per ring with intensity
  fill_solid(ledsInner, NUM_LEDS_INNER, scaledColor(iBaseR,iBaseG,iBaseB,iBright));
  fill_solid(ledsOuter, NUM_LEDS_OUTER, scaledColor(oBaseR,oBaseG,oBaseB,oBright));
  FastLED.show();
}
inline void setBaseRGBInner(uint8_t r,uint8_t g,uint8_t b){ iBaseR=r; iBaseG=g; iBaseB=b; showRings(); }
inline void setBaseRGBOuter(uint8_t r,uint8_t g,uint8_t b){ oBaseR=r; oBaseG=g; oBaseB=b; showRings(); }
inline void setBrightInner(uint8_t b){ iBright=b; showRings(); }
inline void setBrightOuter(uint8_t b){ oBright=b; showRings(); }

/* -------- Intensity ramps (0..255 target) -------- */
inline void startRampInner(uint8_t target, unsigned long durMs){
  iStart = iBright; iTarget = target; iStartMs = millis(); iDurMs = durMs?durMs:1; iRamp = true;
  DBG_PRINT(F("[DBG] LEDRAMP I -> ")); DBG_PRINT((int)target); DBG_PRINT(F(" over ")); DBG_PRINTLN(durMs);
}
inline void startRampOuter(uint8_t target, unsigned long durMs){
  oStart = oBright; oTarget = target; oStartMs = millis(); oDurMs = durMs?durMs:1; oRamp = true;
  DBG_PRINT(F("[DBG] LEDRAMP O -> ")); DBG_PRINT((int)target); DBG_PRINT(F(" over ")); DBG_PRINTLN(durMs);
}
void updateRamps(){
  unsigned long now = millis();
  if (iRamp){
    unsigned long e = now - iStartMs;
    if (e >= iDurMs){ iBright = iTarget; iRamp=false; showRings(); DBG_PRINTLN(F("[DBG] LEDRAMP I complete")); }
    else { float t=(float)e/(float)iDurMs; int v=(int)iStart + (int)((int)iTarget-(int)iStart)*t; if(v<0)v=0; if(v>255)v=255; iBright=(uint8_t)v; showRings(); }
  }
  if (oRamp){
    unsigned long e = now - oStartMs;
    if (e >= oDurMs){ oBright = oTarget; oRamp=false; showRings(); DBG_PRINTLN(F("[DBG] LEDRAMP O complete")); }
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
  if (!USE_US_SAFETY) return;   // fully disabled when toggle is 0
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
      if (!isnan(usLastDistCm)) {
        float delta = d - usLastDistCm;
        DBG_PRINT(F("  \xE2\x88\x86=")); DBG_PRINT2(delta, 2);
        DBG_PRINT(F("  expect: "));
        DBG_PRINT((motion == EXTENDING) ? F("\xE2\x88\x86>=") : F("\xE2\x88\x86<="));
        DBG_PRINTLN(US_MIN_DELTA_CM);
      } else {
        DBG_PRINTLN(F("  (first)"));
      }
    }
  }

  if (d <= 0.0f) return;

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
  RS485.begin(9600);
  delay(30);
  DBG_PRINTLN(F("[DBG] RS485 up @9600 (DE/RE LOW)"));
}
void rs485SendLine(const String &s){
  digitalWrite(rs485DirPin, HIGH); delayMicroseconds(8);
  RS485.print(s); RS485.print('\n'); RS485.flush();
  digitalWrite(rs485DirPin, LOW);
  DBG_PRINT(F("[DBG] TX: ")); DBG_PRINTLN(s);
}
// ACK helpers (only for direct; broadcasts don't get replies)
inline void ackOK(bool isBroadcast){ if (!isBroadcast) rs485SendLine(String(myID)+"/ACK"); }
inline void ackPong(bool isBroadcast){ if (!isBroadcast) rs485SendLine(String(myID)+"/ACK:PONG"); }

/* ------ STATUS reporting ------ */
const char* motionName(Motion m){
  switch(m){
    case EXTENDING: return "EXT";
    case RETRACTING: return "RET";
    default: return "IDLE";
  }
}
inline char bitChar(bool active){ return active ? '1' : '0'; }

void sendStatus(bool isBroadcast){
  if (isBroadcast) return; // respond only when directly addressed

  // Read limit states (ACTIVE LOW => 1 means active)
  char extA = bitChar(limitActiveLOW(R_LIMIT_A));
  char extB = bitChar(limitActiveLOW(R_LIMIT_B));
  char extC = bitChar(limitActiveLOW(R_LIMIT_C));
  char retA = bitChar(limitActiveLOW(L_LIMIT_A));
  char retB = bitChar(limitActiveLOW(L_LIMIT_B));
  char retC = bitChar(limitActiveLOW(L_LIMIT_C));

  String line;
  line.reserve(200);
  line += String(myID) + "/STATUS:";
  line += "motion=";   line += motionName(motion);
  line += ",speed=";   line += defaultSpeed;
  line += ",rpwm=";    line += lastRPWM;
  line += ",lpwm=";    line += lastLPWM;

  line += ",extend=";  line += extA; line += ","; line += extB; line += ","; line += extC; // A,B,C
  line += ",retract="; line += retA; line += ","; line += retB; line += ","; line += retC; // A,B,C

  line += ",iRGB=";    line += (int)iBaseR; line += ","; line += (int)iBaseG; line += ","; line += (int)iBaseB;
  line += ",iBright="; line += (int)iBright;
  line += ",oRGB=";    line += (int)oBaseR; line += ","; line += (int)oBaseG; line += ","; line += (int)oBaseB;
  line += ",oBright="; line += (int)oBright;
  line += ",de_re=";   line += (digitalRead(rs485DirPin)==HIGH ? "HIGH" : "LOW");
  line += ",uptime=";  line += millis();

  rs485SendLine(line);
}

/* ================= Setup / Loop ================= */
void setup(){
  DBG_BEGIN(9600);

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

  // Ultrasonic (only if enabled)
  if (USE_US_SAFETY){
    pinMode(US_TRIG, OUTPUT); pinMode(US_ECHO, INPUT); digitalWrite(US_TRIG, LOW);
  }

  // FastLED
  FastLED.addLeds<LED_TYPE, LED_PIN_INNER, COLOR_ORDER>(ledsInner, NUM_LEDS_INNER);
  FastLED.addLeds<LED_TYPE, LED_PIN_OUTER, COLOR_ORDER>(ledsOuter, NUM_LEDS_OUTER);
  FastLED.setBrightness(255);   // global max; we scale per ring with intensity
  showRings();

  // RS485
  rs485Begin();

  DBG_PRINTLN(F("[DBG] Setup complete"));
}

void loop(){
  handleBus();
  handleMotorState();
  if (USE_US_SAFETY) checkUltrasonicProgress();
  updateRamps();
}

/* -------- Motor state safety -------- */
void handleMotorState(){
  if (motion == IDLE) return;

  if (millis() - motorStartMs >= motorSafetyTimeoutMs){
    DBG_PRINTLN(F("[DBG] Safety timeout"));
    stopMotor();
    return;
  }

  // Only enforce limits when not overriding
  if (!ignoreLimits) {
    if (motion == EXTENDING && anyExtendLimit()){
      DBG_PRINTLN(F("[DBG] EXT limit triggered"));
      stopMotor();
      return;
    }
    if (motion == RETRACTING && anyRetractLimit()){
      DBG_PRINTLN(F("[DBG] RET limit triggered"));
      stopMotor();
      return;
    }
  }
}

/* -------- Parser helpers -------- */
bool isNumberLike(const String& s){
  if (s.length()==0) return false;
  for (uint8_t i=0;i<s.length();++i){
    char c=s[i]; if (!(c=='-'||c=='+'|| (c>='0'&&c<='9'))) return false;
  }
  return true;
}
// Extract command and value supporting both "CMD:..." and "CMD,..." (or none)
void splitCmdAndValue(const String& raw, String& cmd, String& value){
  int colon = raw.indexOf(':');
  if (colon >= 0){
    cmd = raw.substring(0, colon);
    value = raw.substring(colon + 1);
    return;
  }
  int comma = raw.indexOf(',');
  if (comma >= 0){
    cmd = raw.substring(0, comma);
    value = raw.substring(comma + 1);
    return;
  }
  cmd = raw; value = "";
}

/* -------- RS485 input -------- */
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

  String raw = msg.substring(slash + 1);
  raw.trim();

  String command, value;
  splitCmdAndValue(raw, command, value);
  command.toUpperCase();

  DBG_PRINT(F("[DBG] RX ")); DBG_PRINT(id); DBG_PRINT('/'); DBG_PRINT(command);
  if (value.length()){ DBG_PRINT(':'); DBG_PRINT(value); } DBG_PRINTLN("");

  /* ====== Core controls ====== */

  // PING -> PONG
  if (command == "PING"){
    ackPong(isBroadcast);
    return;
  }

  // STATUS (full line)
  if (command == "STATUS"){
    sendStatus(isBroadcast);
    return;
  }

  // OPEN/CLOSE always override (hardware protections trusted)
  if (command == "OPEN" || command == "EXT"){
    int spd = value.length()? constrain(value.toInt(), 0, 255) : defaultSpeed;
    defaultSpeed = spd;
    extendMotorOverride(spd);
    ackOK(isBroadcast);
    return;
  }
  if (command == "CLOSE" || command == "RET"){
    int spd = value.length()? constrain(value.toInt(), 0, 255) : defaultSpeed;
    defaultSpeed = spd;
    retractMotorOverride(spd);
    ackOK(isBroadcast);
    return;
  }
  if (command == "STOP"){
    stopMotor();
    ackOK(isBroadcast);
    return;
  }
  if (command == "SPEED"){
    defaultSpeed = constrain(value.toInt(), 0, 255);
    ackOK(isBroadcast);
    return;
  }

  /* ====== RGB base color (affects how intensities/ramps render) ====== */
  // RGB:r,g,b           -> both rings base
  // RGBIN:r,g,b         -> inner base
  // RGBOUT:r,g,b        -> outer base
  if (command == "RGB" || command == "RGBIN" || command == "RGBOUT"){
    if (value.length()==0) return;
    int p1 = value.indexOf(',');
    int p2 = (p1>=0) ? value.indexOf(',', p1+1) : -1;
    if (p1<0 || p2<0) return;

    uint8_t r = (uint8_t)constrain(value.substring(0,p1).toInt(), 0, 255);
    uint8_t g = (uint8_t)constrain(value.substring(p1+1,p2).toInt(), 0, 255);
    uint8_t b = (uint8_t)constrain(value.substring(p2+1).toInt(), 0, 255);

    if (command == "RGB"){
      setBaseRGBInner(r,g,b);
      setBaseRGBOuter(r,g,b);
    } else if (command == "RGBIN"){
      setBaseRGBInner(r,g,b);
    } else {
      setBaseRGBOuter(r,g,b);
    }
    ackOK(isBroadcast);
    return;
  }

  /* ====== Intensity set (0..120 UI) ====== */
  // LED:n           -> both rings
  // LEDIN:n         -> inner only
  // LEDOUT:n        -> outer only
  // Aliases: LEDSET, LEDSETIN, LEDSETOUT
  if (command == "LED" || command == "LEDIN" || command == "LEDOUT" ||
      command == "LEDSET" || command == "LEDSETIN" || command == "LEDSETOUT"){
    if (value.length()==0) return;
    uint8_t intensity = map120to255(value.toInt());

    if (command == "LED" || command == "LEDSET"){
      setBrightInner(intensity);
      setBrightOuter(intensity);
    } else if (command == "LEDIN" || command == "LEDSETIN"){
      setBrightInner(intensity);
    } else { // LEDOUT or LEDSETOUT
      setBrightOuter(intensity);
    }
    ackOK(isBroadcast);
    return;
  }

  /* ====== Intensity ramps (value 0..120, duration ms) ====== */
  // LEDRAMP:v,ms       -> both
  // LEDRAMPIN:v,ms     -> inner
  // LEDRAMPOUT:v,ms    -> outer
  if (command == "LEDRAMP" || command == "LEDRAMPIN" || command == "LEDRAMPOUT"){
    if (value.length()==0) return;
    int c = value.indexOf(',');
    if (c < 0) return;
    uint8_t tgt = map120to255(value.substring(0,c).toInt());
    unsigned long dur = (unsigned long)max(0, value.substring(c+1).toInt());

    if (command == "LEDRAMP"){
      startRampInner(tgt, dur);
      startRampOuter(tgt, dur);
    } else if (command == "LEDRAMPIN"){
      startRampInner(tgt, dur);
    } else { // LEDRAMPOUT
      startRampOuter(tgt, dur);
    }
    ackOK(isBroadcast);
    return;
  }

  // Unknown command -> silent
}
