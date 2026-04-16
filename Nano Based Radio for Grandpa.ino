// ============================================================
//  FM Radio v17 — Arduino Uno/Nano
//  TEA5767 + LCD I2C 0x27 + KY-040 + SG90 + Active Buzzer
//
//  D2=ENC_CLK  D3=ENC_DT   D4=ENC_SW (mute/menu)
//  D5=BTN_NEXT D6=BTN_PREV D9=SERVO  D13=BUZZER
//  A4=SDA  A5=SCL
//
//  Encoder push SHORT  -> mute / unmute
//  Encoder push LONG   -> servo toggle menu
//
//  Servo menu flow:
//  Line 1: "  Turn off servo" (or "Turn on  servo")
//  Line 2: "[YES]  NO       " or " YES  [NO]      "
//  PREV    -> toggle YES / NO selection
//  NEXT    -> confirm selection
//  After confirm -> "  Servo is OFF  " / "  Servo is ON   " (1.5s)
//  Then back to home screen automatically
// ============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define ENC_CLK      2
#define ENC_DT       3
#define ENC_SW       4
#define BTN_NEXT     5
#define BTN_PREV     6
#define SERVO_PIN    9
#define BUZZER_PIN   13
#define TEA_ADDR     0x60

#define LONG_PRESS_MS  800UL   // hold duration for long press

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo antServo;

// ── Stations ─────────────────────────────────────────────────
struct Station { float freq; const char* name; };
const Station ST[] = {
  { 91.1,  "Radio City  " },
  { 91.9,  "Radio Nasha " },
  { 92.7,  "Big FM      " },
  { 93.5,  "Red FM      " },
  { 94.3,  "Radio One   " },
  { 98.3,  "Mirchi 98.3 " },
  { 100.1, "AIR FM Gold " },
  { 102.8, "Vividh Bhrti" },
  { 104.8, "Ishq FM     " },
  { 106.4, "Magic FM    " },
  { 107.1, "AIR Rainbow " },
};
#define NUM_ST (sizeof(ST)/sizeof(ST[0]))

// ── State ─────────────────────────────────────────────────────
float freq       = ST[0].freq;
int   stIdx      = 0;
bool  muted      = false;
bool  manualMode = false;
bool  servoEnabled = true;   // servo on/off toggle

// ── App mode ──────────────────────────────────────────────────
enum AppMode { MODE_HOME, MODE_SERVO_MENU };
AppMode appMode = MODE_HOME;

// Servo menu state
bool menuSelYes = true;   // true = YES selected, false = NO selected

// ── Encoder ──────────────────────────────────────────────────
volatile int  encDelta  = 0;
int           lastCLK   = HIGH;
unsigned long tLastEnc  = 0;
bool          encActive = false;
#define ENC_IDLE_MS     500UL

// Encoder SW — short vs long press tracking
unsigned long tSWPress  = 0;    // when SW went LOW
bool          swHeld    = false;// true while holding
bool          swShortFired = false; // prevent double fire

// ── Buttons ───────────────────────────────────────────────────
#define DEB             200UL
unsigned long tNext = 0, tPrev = 0;

// ── Servo ─────────────────────────────────────────────────────
#define SV_MIN            0
#define SV_MAX          180
#define SV_STEP_CONT      1
#define SV_STEP_COARSE   15
#define SV_TICK_CONT      8
#define SV_TICK_COARSE  100
#define SV_TICK_PARK      6
#define RSSI_GOOD         5
#define COARSE_SWEEPS     2
#define PARK_HOLDOFF_MS  180000UL

enum SvMode { SV_IDLE, SV_CONT, SV_COARSE, SV_PARK };
SvMode        svMode       = SV_IDLE;
int           svPos        = 90;
int           svDir        = 1;
int           svBestAng    = 90;
int           svBestRSSI   = 0;
int           svSweepsDone = 0;
unsigned long tSvTick      = 0;
unsigned long tParkedAt    = 0;
bool          svEverParked = false;

// ── Watchdog ──────────────────────────────────────────────────
unsigned long tWatchdog     = 0;
int           wdLowCount    = 0;
#define WATCHDOG_MS         1000UL
#define WATCHDOG_LOW_THRESH 3
#define WATCHDOG_LOW_COUNT  2

// ── Periodic seek ─────────────────────────────────────────────
unsigned long tPeriodicSeek = 0;
#define PERIODIC_MS         180000UL

// ── Display ───────────────────────────────────────────────────
unsigned long tDbm         = 0;
unsigned long tLine2       = 0;
unsigned long tStatusShown = 0;
bool          showingStatus = false;
#define DBM_REFRESH_MS     300UL
#define STATUS_SHOW_MS    2000UL
#define LINE2_REFRESH_MS  5000UL

// ============================================================
//  TEA5767
// ============================================================
void teaWrite(float mhz, bool mute) {
  unsigned long hz  = (unsigned long)(mhz * 1000000.0 + 0.5);
  unsigned long pll = (4UL * (hz + 225000UL)) / 32768UL;
  byte b[5];
  b[0] = ((pll >> 8) & 0x3F) | (mute ? 0x80 : 0x00);
  b[1] = pll & 0xFF;
  b[2] = 0xB0;
  b[3] = 0x18;
  b[4] = 0x00;
  Wire.beginTransmission(TEA_ADDR);
  Wire.write(b, 5);
  Wire.endTransmission();
}

bool teaReadRaw(byte s[5]) {
  Wire.requestFrom((uint8_t)TEA_ADDR, (uint8_t)5);
  unsigned long t0 = millis();
  while (Wire.available() < 5) {
    if (millis() - t0 > 50) return false;
  }
  for (int i = 0; i < 5; i++) s[i] = Wire.read();
  return true;
}

int teaRSSI() {
  byte s[5];
  if (!teaReadRaw(s)) return 0;
  return (s[3] & 0x70) >> 4;
}

bool teaStereo() {
  byte s[5];
  if (!teaReadRaw(s)) return false;
  return (s[2] & 0x80) != 0;
}

int rssiToDbm(int r) { return -107 + r * 6; }

int stationIndex(float f) {
  for (int i = 0; i < NUM_ST; i++)
    if (fabs(ST[i].freq - f) < 0.05) return i;
  return -1;
}

// ============================================================
//  Buzzer
// ============================================================
void beepBoot() {
  tone(BUZZER_PIN, 1000, 80);  delay(110);
  tone(BUZZER_PIN, 1500, 80);  delay(110);
  tone(BUZZER_PIN, 2200, 130); delay(160);
  noTone(BUZZER_PIN);
}
void beepClick()    { tone(BUZZER_PIN, 1800, 55);  delay(75);  noTone(BUZZER_PIN); }
void beepMuteOn()   { tone(BUZZER_PIN, 1600, 50);  delay(80);  tone(BUZZER_PIN, 900,  50); delay(70); noTone(BUZZER_PIN); }
void beepMuteOff()  { tone(BUZZER_PIN, 900,  50);  delay(80);  tone(BUZZER_PIN, 1600, 50); delay(70); noTone(BUZZER_PIN); }
void beepConfirm()  { tone(BUZZER_PIN, 2000, 100); delay(120); noTone(BUZZER_PIN); }
void beepCancel()   { tone(BUZZER_PIN, 800,  100); delay(120); noTone(BUZZER_PIN); }

// ============================================================
//  Servo
// ============================================================
void svStartCont() {
  if (!servoEnabled) return;   // servo disabled — skip
  if (svMode == SV_CONT) return;
  svMode = SV_CONT;
}

void svStartCoarse() {
  if (!servoEnabled) return;   // servo disabled — skip
  svMode        = SV_COARSE;
  svBestRSSI    = 0;
  svBestAng     = svPos;
  svSweepsDone  = 0;
  svDir         = 1;
  showingStatus = true;
  tStatusShown  = millis();
}

void svStop() {
  svMode = SV_IDLE;
  // park at center when disabled
  antServo.write(90);
  svPos = 90;
}

void svTick() {
  if (!servoEnabled) return;   // servo disabled — don't move
  unsigned long now = millis();

  if (svMode == SV_CONT) {
    if (now - tSvTick < SV_TICK_CONT) return;
    tSvTick = now;
    svPos += svDir * SV_STEP_CONT;
    svPos  = constrain(svPos, SV_MIN, SV_MAX);
    antServo.write(svPos);
    if (svPos >= SV_MAX) svDir = -1;
    if (svPos <= SV_MIN) svDir =  1;
    return;
  }

  if (svMode == SV_COARSE) {
    if (now - tSvTick < SV_TICK_COARSE) return;
    tSvTick = now;
    svPos += svDir * SV_STEP_COARSE;
    svPos  = constrain(svPos, SV_MIN, SV_MAX);
    antServo.write(svPos);
    int r = teaRSSI();
    if (r > svBestRSSI) { svBestRSSI = r; svBestAng = svPos; }
    if (r >= RSSI_GOOD) {
      svMode        = SV_PARK;
      showingStatus = true;
      tStatusShown  = millis();
      return;
    }
    if (svPos >= SV_MAX) { svDir = -1; svSweepsDone++; }
    if (svPos <= SV_MIN) { svDir =  1; svSweepsDone++; }
    if (svSweepsDone >= COARSE_SWEEPS * 2) {
      svMode        = SV_PARK;
      showingStatus = true;
      tStatusShown  = millis();
    }
    return;
  }

  if (svMode == SV_PARK) {
    if (now - tSvTick < SV_TICK_PARK) return;
    tSvTick = now;
    if (svPos == svBestAng) {
      svMode       = SV_IDLE;
      svEverParked = true;
      tParkedAt    = now;
      return;
    }
    svPos += (svBestAng > svPos) ? 1 : -1;
    antServo.write(svPos);
    return;
  }
}

// ============================================================
//  RSSI Watchdog
// ============================================================
void rssiWatchdog() {
  if (!servoEnabled) return;
  if (svMode != SV_IDLE) return;
  if (encActive) return;
  if (appMode != MODE_HOME) return;
  unsigned long now = millis();
  if (now - tWatchdog < WATCHDOG_MS) return;
  tWatchdog = now;
  if (svEverParked && (now - tParkedAt < PARK_HOLDOFF_MS)) return;
  int r = teaRSSI();
  if (r <= WATCHDOG_LOW_THRESH) {
    wdLowCount++;
    if (wdLowCount >= WATCHDOG_LOW_COUNT) {
      wdLowCount = 0;
      svStartCoarse();
    }
  } else {
    wdLowCount = 0;
  }
}

// ============================================================
//  LCD — Home screen
// ============================================================
void lcdInitChars() {
  byte BAR[5][8] = {
    {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00},
    {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00},
    {0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x00},
    {0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x00},
    {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x00}
  };
  for (int i = 0; i < 5; i++) lcd.createChar(i, BAR[i]);
}

void lcdFull() {
  int  rssi   = teaRSSI();
  int  dbm    = rssiToDbm(rssi);
  bool stereo = teaStereo();
  int  known  = stationIndex(freq);

  lcd.clear();

  // Line 1: freq + dBm + ST/MO
  lcd.setCursor(0, 0);
  char fb[7]; dtostrf(freq, 5, 1, fb);
  lcd.print(fb);
  lcd.print("MHz");
  char db[8]; snprintf(db, sizeof(db), "%4ddB", dbm);
  lcd.setCursor(8, 0);
  lcd.print(db);
  lcd.setCursor(13, 0);
  lcd.print(stereo ? " ST" : " MO");

  // Line 2
  lcd.setCursor(0, 1);
  if (muted) {
    lcd.print(" [[ MUTED ]]    ");
    return;
  }
  if (known >= 0) {
    lcd.print(ST[known].name);
    lcd.setCursor(12, 1);
    lcd.print("    ");
  } else {
    lcd.print("Manual tune     ");
  }
}

void lcdDbm() {
  int dbm = rssiToDbm(teaRSSI());
  char db[8]; snprintf(db, sizeof(db), "%4ddB", dbm);
  lcd.setCursor(8, 0);
  lcd.print(db);
}

void lcdStereo() {
  lcd.setCursor(13, 0);
  lcd.print(teaStereo() ? " ST" : " MO");
}

void lcdFreqOnly() {
  lcd.setCursor(0, 0);
  char fb[7]; dtostrf(freq, 5, 1, fb);
  lcd.print(fb);
  lcd.print("MHz");
}

void lcdShowStatus(const char* s) { lcd.setCursor(12, 1); lcd.print(s); }
void lcdClearStatus()              { lcd.setCursor(12, 1); lcd.print("    "); }

void lcdLine2() {
  int known = stationIndex(freq);
  lcd.setCursor(0, 1);
  if (muted) { lcd.print(" [[ MUTED ]]    "); return; }
  if (known >= 0) {
    lcd.print(ST[known].name);
    lcd.setCursor(12, 1); lcd.print("    ");
  } else {
    lcd.print("Manual tune     ");
  }
}

// ============================================================
//  LCD — Servo menu screen
//
//  Line 1: "Turn off servo  "  or  "Turn on  servo  "
//  Line 2: "[YES]  NO       "  or  " YES  [NO]      "
// ============================================================
void lcdServoMenu() {
  lcd.clear();

  // Line 1 — what will happen if YES
  lcd.setCursor(0, 0);
  if (servoEnabled)
    lcd.print("Turn off servo  ");
  else
    lcd.print("Turn on  servo  ");

  // Line 2 — YES / NO with selection brackets
  lcd.setCursor(0, 1);
  if (menuSelYes)
    lcd.print("[YES]  NO       ");
  else
    lcd.print(" YES  [NO]      ");
}

// ============================================================
//  Enter / exit servo menu
// ============================================================
void enterServoMenu() {
  appMode    = MODE_SERVO_MENU;
  menuSelYes = true;   // default selection = YES
  lcdServoMenu();
  tone(BUZZER_PIN, 1200, 80); delay(100); noTone(BUZZER_PIN);
}

void confirmServoMenu() {
  if (menuSelYes) {
    // User confirmed — toggle servo
    servoEnabled = !servoEnabled;
    if (!servoEnabled) {
      svStop();   // stop servo immediately, park at center
    } else {
      // re-enable — start coarse seek
      svEverParked = false;
      svStartCoarse();
    }
  }
  // Show result for 1.5s
  lcd.clear();
  lcd.setCursor(0, 0);
  if (menuSelYes) {
    lcd.setCursor(2, 0);
    lcd.print(servoEnabled ? "  Servo is ON " : " Servo is OFF ");
  } else {
    lcd.setCursor(3, 0);
    lcd.print("  Cancelled   ");
  }
  lcd.setCursor(0, 1);
  lcd.print("                ");

  beepConfirm();
  delay(1500);

  // Back to home
  appMode = MODE_HOME;
  lcdFull();
}

// ============================================================
//  Tune to preset
// ============================================================
void tuneToPreset(int idx) {
  stIdx      = idx;
  manualMode = false;
  freq       = ST[idx].freq;
  teaWrite(freq, muted);
  delay(80);
  svStartCoarse();
  lcdFull();
}

// ============================================================
//  Encoder ISR
// ============================================================
void encISR() {
  int clk = digitalRead(ENC_CLK), dt = digitalRead(ENC_DT);
  if (clk != lastCLK) {
    encDelta += (dt != clk) ? 1 : -1;
    lastCLK = clk;
  }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  pinMode(ENC_CLK,    INPUT_PULLUP);
  pinMode(ENC_DT,     INPUT_PULLUP);
  pinMode(ENC_SW,     INPUT_PULLUP);
  pinMode(BTN_NEXT,   INPUT_PULLUP);
  pinMode(BTN_PREV,   INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin();
  Wire.setClock(100000);

  lcd.init();
  lcd.backlight();
  lcdInitChars();

  lcd.clear();
  lcd.setCursor(1, 0); lcd.print("FM Radio For");
  lcd.setCursor(0, 1); lcd.print("Smart Grandpa :D");
  delay(1500);

  antServo.attach(SERVO_PIN);
  antServo.write(svPos);
  delay(300);

  lastCLK = digitalRead(ENC_CLK);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encISR, CHANGE);

  beepBoot();
  teaWrite(freq, false);
  delay(200);
  svStartCoarse();
  lcdFull();
}

// ============================================================
//  Loop
// ============================================================
void loop() {
  unsigned long now = millis();

  // ── Servo and watchdog — always run ──────────────────────
  svTick();
  rssiWatchdog();

  // ══════════════════════════════════════════════════════════
  //  SERVO MENU MODE
  // ══════════════════════════════════════════════════════════
  if (appMode == MODE_SERVO_MENU) {

    // PREV — toggle YES / NO
    if (digitalRead(BTN_PREV) == LOW && now - tPrev > DEB) {
      tPrev      = now;
      menuSelYes = !menuSelYes;
      beepClick();
      lcdServoMenu();
      delay(150);
    }

    // NEXT — confirm selection
    if (digitalRead(BTN_NEXT) == LOW && now - tNext > DEB) {
      tNext = now;
      confirmServoMenu();
      delay(200);
    }

    // Encoder SW short press in menu = cancel (same as NO + confirm)
    // Handle in SW section below
    return;   // skip home screen logic while in menu
  }

  // ══════════════════════════════════════════════════════════
  //  HOME MODE
  // ══════════════════════════════════════════════════════════

  // ── Encoder SW — short press = mute, long press = menu ───
  bool swDown = (digitalRead(ENC_SW) == LOW);

  if (swDown && !swHeld) {
    // Button just pressed
    swHeld        = true;
    swShortFired  = false;
    tSWPress      = now;
  }

  if (swHeld && swDown) {
    // Still holding — check for long press threshold
    if (!swShortFired && (now - tSWPress > LONG_PRESS_MS)) {
      // Long press fired — enter menu
      swShortFired = true;   // prevent short press on release
      swHeld       = false;
      enterServoMenu();
    }
  }

  if (swHeld && !swDown) {
    // Released
    swHeld = false;
    if (!swShortFired && (now - tSWPress >= 20)) {
      // Short press — mute toggle
      muted = !muted;
      teaWrite(freq, muted);
      muted ? beepMuteOn() : beepMuteOff();
      lcdFull();
    }
    swShortFired = false;
  }

  // ── Encoder turn ─────────────────────────────────────────
  if (encDelta != 0) {
    noInterrupts();
    int d = encDelta; encDelta = 0;
    interrupts();
    manualMode = true;
    encActive  = true;
    tLastEnc   = now;
    float nf = freq + d * 0.1;
    if (nf < 87.5)  nf = 108.0;
    if (nf > 108.0) nf =  87.5;
    freq = nf;
    teaWrite(freq, muted);
    svStartCont();
    lcdFreqOnly();
  }

  // ── Encoder stopped ───────────────────────────────────────
  if (encActive && (now - tLastEnc > ENC_IDLE_MS)) {
    encActive = false;
    svStartCoarse();
    lcdFull();
  }

  // ── SEEK/PARK status display ──────────────────────────────
  if (showingStatus) {
    if (svMode == SV_COARSE || svMode == SV_CONT)
      lcdShowStatus("SEEK");
    else if (svMode == SV_PARK)
      lcdShowStatus("PARK");
    else if (now - tStatusShown > STATUS_SHOW_MS) {
      showingStatus = false;
      lcdClearStatus();
    }
  }

  // ── NEXT — next preset ────────────────────────────────────
  if (digitalRead(BTN_NEXT) == LOW && now - tNext > DEB) {
    tNext = now;
    beepClick();
    tuneToPreset((stIdx + 1) % NUM_ST);
    delay(200);
  }

  // ── PREV — prev preset ────────────────────────────────────
  if (digitalRead(BTN_PREV) == LOW && now - tPrev > DEB) {
    tPrev = now;
    beepClick();
    tuneToPreset((stIdx - 1 + NUM_ST) % NUM_ST);
    delay(200);
  }

  // ── Periodic 3-min seek ───────────────────────────────────
  if (servoEnabled && svMode == SV_IDLE && !encActive &&
      now - tPeriodicSeek > PERIODIC_MS) {
    tPeriodicSeek = now;
    svStartCoarse();
  }

  // ── Live dBm every 300ms ─────────────────────────────────
  if (now - tDbm > DBM_REFRESH_MS && !encActive) {
    tDbm = now;
    lcdDbm();
    lcdStereo();
  }

  // ── Line 2 refresh every 5s ──────────────────────────────
  if (!encActive && !showingStatus && now - tLine2 > LINE2_REFRESH_MS) {
    tLine2 = now;
    lcdLine2();
  }
}
