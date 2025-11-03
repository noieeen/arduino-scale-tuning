// ---------------------------------------------------------------
//  DIY Bean Doser v3 – ESP32 + MENU + EEPROM + Reverse + Silent
// ---------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include <AccelStepper.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>  // ← NEW

// ---------------------  PINOUT ---------------------
#define OLED_ADDR   0x3C
#define SCREEN_W    128
#define SCREEN_H    32
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

#define HX_DATA     13
#define HX_CLK      12
HX711 scale;

#define STEP_PIN    25
#define DIR_PIN     26
#define ENABLE_PIN  10
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

#define ENC_CLK     17
#define ENC_DT      16
#define ENC_SW      27
RotaryEncoder encoder(ENC_CLK, ENC_DT, RotaryEncoder::LatchMode::TWO03);

// ---------------------  EEPROM ---------------------
#define EEPROM_SIZE 64
#define ADDR_MAGIC  0
#define ADDR_FAST   4
#define ADDR_SLOW   8
#define ADDR_THRESH 12
#define ADDR_UNJAM  16
#define ADDR_STALL  20
#define ADDR_CAL    24
const uint32_t MAGIC = 0xCAFEBABE;

// ---------------------  SETTINGS (LIVE) ---------------------
struct Settings {
  int fastRPM = 120;
  int slowRPM = 30;
  float slowThresh = 1.5;
  int unjamSteps = 50;
  int stallTimeout = 800;
  float calFactor = 420.0983;
} cfg;

const float TARGET_MIN = 5.0, TARGET_MAX = 30.0;
float targetWeight = 15.0;

// ---------------------  Runtime ---------------------
float filterBuf[5] = {0};
uint8_t filterIdx = 0;
float filterSum = 0;
float lastDisp = -99.0;
unsigned long lastWeightChange = 0;
float lastWeight = 0;
bool motorPowered = false;

// Menu
bool inMenu = false;
int menuItem = 0;
const int MENU_ITEMS = 7;
const char* menuLabels[MENU_ITEMS] = {
  "Fast RPM", "Slow RPM", "Slow Thresh", 
  "Unjam Steps", "Stall ms", "Cal Factor", "SAVE & EXIT"
};

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(ENABLE_PIN, OUTPUT);
  motorOff();

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("OLED failed")); for(;;);
  }
  display.setTextColor(SSD1306_WHITE);
  splashScreen();

  // EEPROM
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(ADDR_MAGIC) != (MAGIC & 0xFF) ||
      EEPROM.read(ADDR_MAGIC+1) != ((MAGIC >> 8) & 0xFF) ||
      EEPROM.read(ADDR_MAGIC+2) != ((MAGIC >> 16) & 0xFF) ||
      EEPROM.read(ADDR_MAGIC+3) != ((MAGIC >> 24) & 0xFF)) {
    saveSettings();  // first boot
  } else {
    loadSettings();
  }

  // HX711
  scale.begin(HX_DATA, HX_CLK);
  scale.set_gain(128);
  scale.set_scale(cfg.calFactor);
  delay(500);
  showMessage("Taring...");
  scale.tare(15);
  showMessage("Ready");

  // Stepper
  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(1000);

  // Encoder
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT),  encISR, CHANGE);

  lastWeight = getWeight();
  lastWeightChange = millis();
}

void emergencyStop() {
  stopMotor();
  showMessage("STOP");
  delay(1000);
  showTarget();
}

void loop() {
  static enum {IDLE, DOSING, FINISH, UNJAMMING} state = IDLE;
  static unsigned long btnPressTime = 0;

  encoder.tick();
  long pos = encoder.getPosition();
  static long lastPos = 0;

  // --- Button handling ---
  if (digitalRead(ENC_SW) == LOW) {
    if (btnPressTime == 0) btnPressTime = millis();
    else if (millis() - btnPressTime > 2000) {
      if (!inMenu) { enterMenu(); }
      else if (menuItem == MENU_ITEMS-1) { saveAndExit(); }
    }
  } else {
    if (btnPressTime != 0 && millis() - btnPressTime < 2000) {
      if (!inMenu && state == IDLE) {
        state = DOSING; startDosing();
      } else if (inMenu) {
        menuItem = (menuItem + 1) % MENU_ITEMS;
        showMenu();
      } else {
        emergencyStop();
      }
    }
    btnPressTime = 0;
  }

  // --- Encoder rotate ---
  if (pos != lastPos) {
    if (inMenu) {
      adjustMenu(pos > lastPos);
      showMenu();
    } else {
      float delta = (pos > lastPos) ? 0.1 : -0.1;
      targetWeight = constrain(targetWeight + delta, TARGET_MIN, TARGET_MAX);
      showTarget();
    }
    lastPos = pos;
  }

  // --- Dosing ---
  float w = getWeight();
  if (state == DOSING || state == FINISH) {
    motorOn();
    if (state == DOSING) runFast();
    else runSlow();

    if (w >= targetWeight - cfg.slowThresh) {
      setSlowSpeed(); state = FINISH;
    }
    if (w >= targetWeight) {
      stopMotor(); showMessage("Done!"); delay(1000); showTarget(); state = IDLE;
    }

    // Unjam
    if (abs(w - lastWeight) > 0.05) {
      lastWeight = w; lastWeightChange = millis();
    } else if (millis() - lastWeightChange > cfg.stallTimeout) {
      unjamReverse(); delay(200);
      if (state == FINISH) setSlowSpeed(); else setFastSpeed();
      lastWeightChange = millis();
    }
  } else {
    motorOff();
  }

  // --- Display ---
  if (!inMenu && abs(w - lastDisp) >= 0.25) {
    showWeight(w); lastDisp = w;
  }

  delay(10);
}

// ---------------------------------------------------------------
float getWeight() {
  if (!scale.is_ready()) return filterBuf[filterIdx];
  float raw = scale.get_units(2);
  filterSum -= filterBuf[filterIdx];
  filterBuf[filterIdx] = raw;
  filterSum += raw;
  filterIdx = (filterIdx + 1) % 5;
  return filterSum / 5;
}

// ---------------------------------------------------------------
void startDosing() {
  showMessage("Dosing...");
  setFastSpeed();
  stepper.move(100000);
  lastWeight = getWeight();
  lastWeightChange = millis();
}

void setFastSpeed() { stepper.setSpeed(cfg.fastRPM * 200 / 60); }
void setSlowSpeed() { stepper.setSpeed(cfg.slowRPM * 200 / 60); }
void runFast()  { stepper.runSpeed(); }
void runSlow()  { stepper.runSpeed(); }

// Motor power
void motorOn()  { if (!motorPowered) { digitalWrite(ENABLE_PIN, LOW); motorPowered = true; } }
void motorOff() { if (motorPowered) { stepper.stop(); digitalWrite(ENABLE_PIN, HIGH); motorPowered = false; } }
void stopMotor(){ stepper.stop(); motorOff(); }

// Unjam
void unjamReverse() {
  showMessage("UNJAM!");
  digitalWrite(LED_BUILTIN, HIGH);
  motorOn();
  stepper.setSpeed(- (cfg.slowRPM * 200 / 60));
  for (int i = 0; i < cfg.unjamSteps; i++) { stepper.runSpeed(); delayMicroseconds(800); }
  stepper.stop(); motorOff();
  digitalWrite(LED_BUILTIN, LOW);
}

// Menu
void enterMenu() {
  inMenu = true; menuItem = 0; showMenu();
}
void showMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(F("> "));
  display.println(menuLabels[menuItem]);
  display.setCursor(10, 16);
  switch (menuItem) {
    case 0: display.print(cfg.fastRPM); display.println(F(" RPM")); break;
    case 1: display.print(cfg.slowRPM); display.println(F(" RPM")); break;
    case 2: display.print(cfg.slowThresh,1); display.println(F(" g")); break;
    case 3: display.print(cfg.unjamSteps); display.println(F(" steps")); break;
    case 4: display.print(cfg.stallTimeout); display.println(F(" ms")); break;
    case 5: display.print(cfg.calFactor,1); break;
    case 6: display.println(F("Confirm save")); break;
  }
  display.display();
}
void adjustMenu(bool up) {
  switch (menuItem) {
    case 0: cfg.fastRPM = constrain(cfg.fastRPM + (up ? 10 : -10), 60, 180); break;
    case 1: cfg.slowRPM = constrain(cfg.slowRPM + (up ? 5 : -5), 10, 60); break;
    case 2: cfg.slowThresh = constrain(cfg.slowThresh + (up ? 0.1 : -0.1), 0.5, 3.0); break;
    case 3: cfg.unjamSteps = constrain(cfg.unjamSteps + (up ? 10 : -10), 20, 100); break;
    case 4: cfg.stallTimeout = constrain(cfg.stallTimeout + (up ? 100 : -100), 300, 1500); break;
    case 5: cfg.calFactor += (up ? 1.0 : -1.0); if (cfg.calFactor < 100) cfg.calFactor = 100; break;
  }
}
void saveAndExit() {
  saveSettings();
  scale.set_scale(cfg.calFactor);
  inMenu = false;
  showTarget();
}
void saveSettings() {
  uint32_t m = MAGIC;
  EEPROM.write(ADDR_MAGIC, m & 0xFF);
  EEPROM.write(ADDR_MAGIC+1, (m >> 8) & 0xFF);
  EEPROM.write(ADDR_MAGIC+2, (m >> 16) & 0xFF);
  EEPROM.write(ADDR_MAGIC+3, (m >> 24) & 0xFF);
  EEPROM.put(ADDR_FAST, cfg.fastRPM);
  EEPROM.put(ADDR_SLOW, cfg.slowRPM);
  EEPROM.put(ADDR_THRESH, cfg.slowThresh);
  EEPROM.put(ADDR_UNJAM, cfg.unjamSteps);
  EEPROM.put(ADDR_STALL, cfg.stallTimeout);
  EEPROM.put(ADDR_CAL, cfg.calFactor);
  EEPROM.commit();
}
void loadSettings() {
  EEPROM.get(ADDR_FAST, cfg.fastRPM);
  EEPROM.get(ADDR_SLOW, cfg.slowRPM);
  EEPROM.get(ADDR_THRESH, cfg.slowThresh);
  EEPROM.get(ADDR_UNJAM, cfg.unjamSteps);
  EEPROM.get(ADDR_STALL, cfg.stallTimeout);
  EEPROM.get(ADDR_CAL, cfg.calFactor);
}

// Display
void splashScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0); display.println(F("Bean Doser v3"));
  display.setCursor(0,16); display.println(F("Hold 2s = Menu"));
  display.display(); delay(1500);
}
void showMessage(const String &msg) {
  display.clearDisplay(); display.setTextSize(2); display.setCursor(10,8); display.println(msg); display.display();
}
void showTarget() {
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0,0); display.print(F("Target: "));
  display.setTextSize(2); display.setCursor(0,12); display.print(targetWeight,1); display.print(F("g")); display.display();
}
void showWeight(float w) {
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0,0); display.print(F("Weight:"));
  display.setTextSize(2); display.setCursor(0,12);
  if (w < 0.01) display.print(F("0.00")); else display.print(w, 2);
  display.print(F("g")); display.display();
}

void encISR() { encoder.tick(); }
