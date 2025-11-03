// ---------------------------------------------------------------
//  DIY Bean Doser – NEMA-17 + HX711 + SSD1306 + Encoder
//  Based on your fast HX711 code + AccelStepper "slow finish"
// ---------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include <AccelStepper.h>
#include <RotaryEncoder.h>

// ---------------------  PINOUT  ---------------------
#define OLED_ADDR   0x3C
#define SCREEN_W    128
#define SCREEN_H    32
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

#define HX_DATA     13
#define HX_CLK      12
HX711 scale;

#define STEP_PIN     25
#define DIR_PIN      26
#define ENABLE_PIN  10   // optional, LOW = enabled
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

#define ENC_CLK      17   // DT
#define ENC_DT       16   // CLK
#define ENC_SW       27
RotaryEncoder encoder(ENC_CLK, ENC_DT, RotaryEncoder::LatchMode::TWO03);

// ---------------------  SETTINGS  ---------------------
const float CAL_FACTOR       = 420.0983;   // your value
const uint8_t AVG_READINGS   = 2;          // fast
const uint8_t FILTER_SIZE    = 5;
float filterBuf[FILTER_SIZE] = {0};
uint8_t filterIdx = 0;
float filterSum = 0;

const float DISPLAY_THRESH   = 0.25;       // g
float lastDisp = -99.0;

const float TARGET_MIN       = 5.0;        // g
const float TARGET_MAX       = 30.0;
float targetWeight = 15.0;                 // default

// Stepper speed profile (tune for your auger)
const long FAST_RPM   = 120;   // ~2 beans/s
const long SLOW_RPM   = 30;    // ~0.3 beans/s
const float SLOW_THRESHOLD = 1.5;   // g before target → switch to slow

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial); delay(100);
  Serial.println(F("\n=== DIY Bean Doser ==="));

  // ----- OLED -----
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("OLED failed"));
    for (;;) ;
  }
  display.setTextColor(SSD1306_WHITE);
  splashScreen();

  // ----- HX711 -----
  scale.begin(HX_DATA, HX_CLK);
  scale.set_gain(128);
  scale.set_scale(CAL_FACTOR);
  delay(500);
  Serial.println(F("Taring..."));
  showMessage("Taring...");
  scale.tare(15);
  showMessage("Ready");

  // ----- Stepper -----
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);            // enable driver
  stepper.setMaxSpeed(1000);                // steps/s safety
  stepper.setAcceleration(800);

  // ----- Encoder -----
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT),  encISR, CHANGE);
}

void loop() {
  static enum {IDLE, DOSING, FINISH} state = IDLE;
  static unsigned long lastBtn = 0;

  // ---- Encoder handling (target adjust) ----
  encoder.tick();
  long encPos = encoder.getPosition();
  static long lastPos = 0;
  if (encPos != lastPos) {
    float delta = (encPos > lastPos) ? 0.1 : -0.1;
    targetWeight = constrain(targetWeight + delta, TARGET_MIN, TARGET_MAX);
    lastPos = encPos;
    showTarget();
  }

  // ---- Button press → start dosing ----
  if (digitalRead(ENC_SW) == LOW && millis() - lastBtn > 300) {
    lastBtn = millis();
    if (state == IDLE) {
      state = DOSING;
      startDosing();
    } else {
      emergencyStop();
    }
  }

  // ---- Dosing state machine ----
  switch (state) {
    case DOSING:
      runFast();
      if (getWeight() >= targetWeight - SLOW_THRESHOLD) {
        setSlowSpeed();
        state = FINISH;
      }
      break;

    case FINISH:
      runSlow();
      if (getWeight() >= targetWeight) {
        stopMotor();
        showMessage("Done!");
        state = IDLE;
      }
      break;

    default:
      stepper.stop();
      break;
  }

  // ---- Live weight display (only when changed) ----
  float w = getWeight();
  if (abs(w - lastDisp) >= DISPLAY_THRESH) {
    showWeight(w);
    lastDisp = w;
  }

  delay(10);   // tight loop, HX711 still ~100 Hz
}

// ---------------------------------------------------------------
float getWeight() {
  if (!scale.is_ready()) return filterBuf[filterIdx];   // last value

  float raw = scale.get_units(AVG_READINGS);
  filterSum -= filterBuf[filterIdx];
  filterBuf[filterIdx] = raw;
  filterSum += raw;
  filterIdx = (filterIdx + 1) % FILTER_SIZE;
  return filterSum / FILTER_SIZE;
}

// ---------------------------------------------------------------
void startDosing() {
  showMessage("Dosing...");
  setFastSpeed();
  stepper.move(100000);      // run continuously forward
}

void setFastSpeed() {
  long stepsPerSec = FAST_RPM * 200 / 60;   // 200 steps/rev
  stepper.setSpeed(stepsPerSec);
}
void setSlowSpeed() {
  long stepsPerSec = SLOW_RPM * 200 / 60;
  stepper.setSpeed(stepsPerSec);
}
void runFast()  { stepper.runSpeed(); }
void runSlow()  { stepper.runSpeed(); }
void stopMotor(){ stepper.stop(); digitalWrite(ENABLE_PIN, HIGH); }

void emergencyStop() {
  stopMotor();
  showMessage("STOP");
  delay(1000);
  showTarget();
}

// ---------------------------------------------------------------
void encISR() { encoder.tick(); }

// ---------------------------------------------------------------
// ---- DISPLAY HELPERS ------------------------------------------------
void splashScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println(F("DIY Bean Doser"));
  display.setCursor(0,16);
  display.println(F("Rotate = target"));
  display.display();
  delay(1500);
}
void showMessage(const String &msg) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10,8);
  display.println(msg);
  display.display();
}
void showTarget() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(F("Target: "));
  display.setTextSize(2);
  display.setCursor(0,12);
  display.print(targetWeight,1);
  display.print(F("g"));
  display.display();
}
void showWeight(float w) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(F("Weight:"));
  display.setTextSize(2);
  display.setCursor(0,12);
  if (w < 0.01) display.print(F("0.00"));
  else          display.print(w, 2);
  display.print(F("g"));
  display.display();
}
