// ---------------------------------------------------------------
//  DIY "Smart" Bean Doser v6.0 – ESP32 + ADAPTIVE + PRESETS + AI
//  Enhanced with predictive algorithms and advanced learning
// ---------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include <AccelStepper.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <string.h>

// ---------------------  PINOUT ---------------------
#define LED_BUILTIN 2
#define OLED_ADDR   0x3C
#define SCREEN_W    128
#define SCREEN_H    32
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

#define HX_DATA     13
#define HX_CLK      12
HX711 scale;

#define STEP_PIN    25
#define DIR_PIN     26
#define ENABLE_PIN  27  // Changed from 10 to 27 for better ESP32 compatibility
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

#define ENC_CLK     17
#define ENC_DT      16
#define ENC_SW      14  // Changed from 27 to 14
RotaryEncoder encoder(ENC_CLK, ENC_DT, RotaryEncoder::LatchMode::TWO03);

// ---------------------  EEPROM LAYOUT ---------------------
#define EEPROM_SIZE 512
#define ADDR_MAGIC      0
#define ADDR_SETTINGS   4
#define ADDR_PRESETS    100
#define ADDR_LEARNING   200
#define ADDR_STATISTICS 300
#define ADDR_PROFILES   400

const uint32_t MAGIC = 0xC0FFEE42; // v6.0 magic

// ---------------------  ADVANCED SETTINGS ---------------------
struct AdvancedSettings {
  // Motor control
  int fastRPM = 150;
  int slowRPM = 40;
  int ultraSlowRPM = 15;
  float slowThreshold = 2.0;
  float ultraSlowThreshold = 0.8;
  
  // Adaptive parameters
  float baseOverrun = 0.35;
  float adaptiveGain = 0.25;
  
  // Safety & reliability
  int unjamSteps = 75;
  int stallTimeout = 600;
  int maxUnjamAttempts = 4;
  float calFactor = 420.0;
  
  // Predictive control
  float flowRatePrediction = 1.0;
  float accelerationCoeff = 0.1;
  bool enablePredictive = true;
  
  // Quality control
  float toleranceStrict = 0.05;
  float toleranceNormal = 0.15;
  float toleranceLoose = 0.3;
  int qualityMode = 1; // 0=strict, 1=normal, 2=loose
} cfg;

// ---------------------  SMART PRESETS ---------------------
struct SmartPreset {
  char name[16];
  float targetWeight;
  float learnedOverrun;
  float avgFlowRate;
  int useCount;
  float accuracy; // Running average of accuracy
  unsigned long totalTime; // Total dosing time
  bool autoOptimize;
};

SmartPreset presets[6] = {
  {"Single", 8.5, 0.35, 1.2, 0, 0.0, 0, true},
  {"Double", 17.0, 0.4, 1.2, 0, 0.0, 0, true},
  {"Triple", 25.5, 0.45, 1.2, 0, 0.0, 0, true},
  {"Light", 6.0, 0.3, 1.1, 0, 0.0, 0, true},
  {"Strong", 20.0, 0.4, 1.3, 0, 0.0, 0, true},
  {"Custom", 15.0, 0.35, 1.2, 0, 0.0, 0, true}
};

int activePreset = 0;
float targetWeight = 8.5;

// ---------------------  LEARNING SYSTEM ---------------------
struct LearningData {
  float weightHistory[20];
  float timeHistory[20];
  float flowRates[10];
  float overrunHistory[15];
  int historyIndex = 0;
  int flowIndex = 0;
  int overrunIndex = 0;
  
  // Performance metrics
  float avgAccuracy = 0.0;
  int totalDoses = 0;
  float bestAccuracy = 0.0;
  unsigned long totalDosingTime = 0;
} learning;

// ---------------------  PREDICTIVE ALGORITHMS ---------------------
struct PredictiveModel {
  float predictedOverrun = 0.35;
  float predictedFlowRate = 1.2;
  float confidenceLevel = 0.5;
  bool modelReady = false;
  
  // Kalman filter variables
  float kalmanGain = 0.1;
  float processNoise = 0.01;
  float measurementNoise = 0.05;
  float estimate = 0.35;
  float errorCovariance = 1.0;
} predictor;

// ---------------------  STATE MACHINE v6.0 ---------------------
enum DoseState {
  IDLE,
  TARE_CHECK,
  PRE_DOSE_ANALYSIS,
  DOSING_FAST,
  DOSING_SLOW,
  DOSING_ULTRA_SLOW,
  FINISHING,
  POST_DOSE_LEARNING,
  QUALITY_CHECK,
  PURGING,
  JAMMED,
  MENU,
  CALIBRATE,
  STATISTICS,
  PROFILES
};
DoseState doseState = IDLE;

// ---------------------  RUNTIME VARIABLES ---------------------
float weightBuffer[10] = {0};
uint8_t bufferIndex = 0;
float filteredWeight = 0;
float lastDisplayWeight = -99.0;

unsigned long doseStartTime = 0;
unsigned long phaseStartTime = 0;
float preOverrunWeight = 0;
float doseStartWeight = 0;

// Environmental - Temperature sensor removed

// Button handling
unsigned long lastPressTime = 0;
unsigned long lastBtnChange = 0;
bool lastBtnState = HIGH;
bool btnState = HIGH;
const unsigned long DEBOUNCE_MS = 30;
const unsigned long LONG_PRESS_MS = 1500;
const unsigned long DOUBLE_PRESS_MS = 400;

// Menu system
int menuItem = 0;
int subMenuItem = 0;
bool inSubMenu = false;
unsigned long menuLastAction = 0;
const unsigned long MENU_TIMEOUT = 20000;

// Statistics
struct Statistics {
  int totalDoses = 0;
  float avgAccuracy = 0.0;
  float bestAccuracy = 0.0;
  float worstAccuracy = 0.0;
  unsigned long totalTime = 0;
  int jamCount = 0;
} stats;

// Quality control
float lastDoseAccuracy = 0.0;
bool qualityPassed = true;

// Flow rate monitoring
float instantFlowRate = 0.0;
float avgFlowRate = 1.2;
unsigned long lastFlowUpdate = 0;

// Unjam system
int unjamCount = 0;
bool progressiveUnjam = true;

// Motor state
bool motorPowered = false;

// Forward declarations
void encISR();
void motorOn();
void motorOff();
void stopMotor();
void showMessage(const String &msg);
void showIdleScreen();
void performAdvancedTare();
void updateEnvironmentalData();
void initializePredictiveModel();
void saveAllSettings();
void loadAllSettings();

// ---------------------------------------------------------------
//  SETUP
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Smart Bean Doser v6.0 Starting..."));
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // A4988 stepper driver setup
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  // Start with motor disabled
  motorPowered = false;

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("OLED initialization failed"));
    while(1);
  }
  display.setTextColor(SSD1306_WHITE);
  showBootScreen();

  // Factory reset check
  pinMode(ENC_SW, INPUT_PULLUP);
  if (digitalRead(ENC_SW) == LOW) {
    showMessage("Hold 5s for\nFactory Reset");
    delay(5000);
    if (digitalRead(ENC_SW) == LOW) {
      factoryReset();
      showMessage("RESET COMPLETE");
      delay(2000);
    }
  }

  // Initialize EEPROM and load settings
  EEPROM.begin(EEPROM_SIZE);
  if (!checkMagic()) {
    Serial.println(F("First boot - initializing defaults"));
    saveAllSettings();
  } else {
    loadAllSettings();
  }

  // Initialize HX711 with advanced filtering
  scale.begin(HX_DATA, HX_CLK);
  scale.set_gain(128);
  scale.set_scale(cfg.calFactor);
  delay(1000);
  
  showMessage("Initializing\nScale...");
  performAdvancedTare();
  
  // Initialize stepper with advanced parameters
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1500);

  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), encISR, CHANGE);

  // Initialize predictive model
  initializePredictiveModel();
  
  // Read initial temperature
  updateEnvironmentalData();
  
  showMessage("System Ready!");
  delay(1000);
  
  // Show control instructions
  showMessage("Controls:\nDial=Weight\nPress=Dose\n2x=Preset Long=Menu");
  delay(3000);
  
  showIdleScreen();
  
  Serial.println(F("Initialization complete"));
}

// ---------------------------------------------------------------
//  MAIN LOOP
// ---------------------------------------------------------------
void loop() {
  handleButton();
  handleEncoder();
  runAdvancedStateMachine();
  updateDisplay();
  
  // Periodic learning updates
  static unsigned long lastLearningUpdate = 0;
  if (millis() - lastLearningUpdate > 5000) {
    updatePredictiveModel();
    lastLearningUpdate = millis();
  }
  
  delay(10); // Slightly longer delay to reduce flickering
}

// ---------------------------------------------------------------
//  ADVANCED STATE MACHINE
// ---------------------------------------------------------------
void runAdvancedStateMachine() {
  unsigned long now = millis();
  
  // Menu timeout
  if ((doseState == MENU || doseState == STATISTICS || doseState == PROFILES) && 
      now - menuLastAction > MENU_TIMEOUT) {
    doseState = IDLE;
    showMessage("Menu Timeout");
    delay(500);
    showIdleScreen();
  }
  
  float weight = getSmartWeight();
  
  switch(doseState) {
    case IDLE:
      motorOff();
      monitorIdleWeight(weight);
      break;

    case TARE_CHECK:
      if (performSmartTareCheck(weight)) {
        doseState = PRE_DOSE_ANALYSIS;
        doseStartTime = now;
        doseStartWeight = weight;
      }
      break;
      
    case PRE_DOSE_ANALYSIS:
      if (performPreDoseAnalysis()) {
        startSmartDosing();
        doseState = DOSING_FAST;
        phaseStartTime = now;
      }
      break;
      
    case DOSING_FAST:
      motorOn();
      runSmartMotor();
      updateFlowRate(weight, now);
      
      if (weight >= targetWeight - cfg.slowThreshold) {
        doseState = DOSING_SLOW;
        phaseStartTime = now;
        setSlowSpeed();
        Serial.println("Switching to slow phase");
      }
      
      if (checkForAdvancedStall(weight, now)) return;
      break;
      
    case DOSING_SLOW:
      motorOn();
      runSmartMotor();
      updateFlowRate(weight, now);
      
      if (weight >= targetWeight - cfg.ultraSlowThreshold) {
        doseState = DOSING_ULTRA_SLOW;
        phaseStartTime = now;
        setUltraSlowSpeed();
        Serial.println("Switching to ultra-slow phase");
      }
      
      if (checkForAdvancedStall(weight, now)) return;
      break;
      
    case DOSING_ULTRA_SLOW:
      {
        motorOn();
        runSmartMotor();
        
        float predictedFinal = weight + calculateDynamicOverrun();
        if (predictedFinal >= targetWeight) {
          stopMotor();
          preOverrunWeight = weight;
          doseState = POST_DOSE_LEARNING;
          phaseStartTime = now;
          Serial.println("Dose complete - starting learning phase");
        }
        
        if (checkForAdvancedStall(weight, now)) return;
      }
      break;
      
    case POST_DOSE_LEARNING:
      {
        if (now - phaseStartTime > 2000) { // Wait for settling
          float finalWeight = getSmartWeight();
          performPostDoseLearning(finalWeight);
          doseState = QUALITY_CHECK;
        }
      }
      break;
      
    case QUALITY_CHECK:
      performQualityCheck();
      updateStatistics();
      showDoseResults();
      delay(2500);
      doseState = IDLE;
      showIdleScreen();
      break;
      
    case PURGING:
      motorOn();
      setFastSpeed();
      runSmartMotor();
      break;
      
    case JAMMED:
      handleJammedState();
      break;
      
    case MENU:
    case STATISTICS:
    case PROFILES:
    case CALIBRATE:
      // Handled by input functions
      break;
  }
}

// ---------------------------------------------------------------
//  SMART WEIGHT MEASUREMENT
// ---------------------------------------------------------------
float getSmartWeight() {
  if (!scale.is_ready()) return filteredWeight;
  
  float raw = scale.get_units(3);
  
  // Advanced filtering with outlier rejection
  weightBuffer[bufferIndex] = raw;
  bufferIndex = (bufferIndex + 1) % 10;
  
  // Calculate median for outlier rejection
  float sorted[10];
  memcpy(sorted, weightBuffer, sizeof(weightBuffer));
  
  // Simple bubble sort for median
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9 - i; j++) {
      if (sorted[j] > sorted[j + 1]) {
        float temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }
  
  // Use median of middle 6 values
  float sum = 0;
  for (int i = 2; i < 8; i++) {
    sum += sorted[i];
  }
  
  filteredWeight = sum / 6.0;
  
  return filteredWeight;
}

// ---------------------------------------------------------------
//  PREDICTIVE ALGORITHMS
// ---------------------------------------------------------------
void initializePredictiveModel() {
  predictor.estimate = cfg.baseOverrun;
  predictor.errorCovariance = 1.0;
  predictor.modelReady = (learning.totalDoses > 5);
  Serial.println(F("Predictive model initialized"));
}

void updatePredictiveModel() {
  if (learning.totalDoses < 3) return;
  
  // Kalman filter update for overrun prediction
  float measurement = learning.overrunHistory[learning.overrunIndex];
  
  // Prediction step
  float predictedEstimate = predictor.estimate;
  float predictedCovariance = predictor.errorCovariance + predictor.processNoise;
  
  // Update step
  predictor.kalmanGain = predictedCovariance / (predictedCovariance + predictor.measurementNoise);
  predictor.estimate = predictedEstimate + predictor.kalmanGain * (measurement - predictedEstimate);
  predictor.errorCovariance = (1 - predictor.kalmanGain) * predictedCovariance;
  
  // Update confidence
  predictor.confidenceLevel = constrain(learning.totalDoses / 20.0, 0.0, 1.0);
  predictor.modelReady = (predictor.confidenceLevel > 0.3);
  
  // Update predicted overrun
  predictor.predictedOverrun = predictor.estimate;
}

float calculateDynamicOverrun() {
  if (!predictor.modelReady) {
    return presets[activePreset].learnedOverrun;
  }
  
  // Base overrun from predictive model
  float dynamicOverrun = predictor.predictedOverrun;
  
  // Adjust for flow rate
  if (instantFlowRate > 0) {
    float flowFactor = instantFlowRate / avgFlowRate;
    dynamicOverrun *= (1.0 + (flowFactor - 1.0) * cfg.adaptiveGain);
  }
  
  // Constrain to reasonable bounds
  return constrain(dynamicOverrun, 0.1, 1.5);
}

// ---------------------------------------------------------------
//  ADVANCED MOTOR CONTROL
// ---------------------------------------------------------------
void startSmartDosing() {
  unjamCount = 0;
  setFastSpeed();
  
  // Calculate initial move distance based on prediction
  float estimatedSteps = (targetWeight / avgFlowRate) * (cfg.fastRPM * 200 / 60.0);
  stepper.move(estimatedSteps * 1.5); // Add safety margin
  
  Serial.print("Starting dose for "); Serial.print(targetWeight); Serial.println("g");
}

void setFastSpeed() {
  float speed = cfg.fastRPM * 200 / 60.0;
  stepper.setSpeed(speed);
}

void setSlowSpeed() {
  float speed = cfg.slowRPM * 200 / 60.0;
  stepper.setSpeed(speed);
}

void setUltraSlowSpeed() {
  float speed = cfg.ultraSlowRPM * 200 / 60.0;
  stepper.setSpeed(speed);
}

void runSmartMotor() {
  stepper.runSpeed();
  
  // Adaptive speed control based on flow rate
  if (cfg.enablePredictive && instantFlowRate > 0) {
    float speedAdjustment = 1.0;
    
    if (instantFlowRate > avgFlowRate * 1.2) {
      speedAdjustment = 0.9; // Slow down if flowing too fast
    } else if (instantFlowRate < avgFlowRate * 0.8) {
      speedAdjustment = 1.1; // Speed up if flowing too slow
    }
    
    float currentSpeed = stepper.speed();
    stepper.setSpeed(currentSpeed * speedAdjustment);
  }
}

// ---------------------------------------------------------------
//  FLOW RATE MONITORING
// ---------------------------------------------------------------
void updateFlowRate(float weight, unsigned long now) {
  static float lastWeight = 0;
  static unsigned long lastTime = 0;
  
  if (now - lastTime > 200) { // Update every 200ms
    float deltaWeight = weight - lastWeight;
    float deltaTime = (now - lastTime) / 1000.0; // Convert to seconds
    
    if (deltaTime > 0 && deltaWeight > 0) {
      instantFlowRate = deltaWeight / deltaTime;
      
      // Update running average
      avgFlowRate = (avgFlowRate * 0.8) + (instantFlowRate * 0.2);
      
      // Store in learning data
      learning.flowRates[learning.flowIndex] = instantFlowRate;
      learning.flowIndex = (learning.flowIndex + 1) % 10;
    }
    
    lastWeight = weight;
    lastTime = now;
    lastFlowUpdate = now;
  }
}

// ---------------------------------------------------------------
//  ADVANCED STALL DETECTION
// ---------------------------------------------------------------
bool checkForAdvancedStall(float weight, unsigned long now) {
  static float stallWeight = 0;
  static unsigned long stallStartTime = 0;
  static bool stallDetected = false;
  
  // Check for weight change
  if (abs(weight - stallWeight) > 0.03) {
    stallWeight = weight;
    stallStartTime = now;
    stallDetected = false;
    unjamCount = 0; // Reset on successful flow
    return false;
  }
  
  // Check for stall timeout
  if (!stallDetected && (now - stallStartTime > cfg.stallTimeout)) {
    stallDetected = true;
    
    if (unjamCount < cfg.maxUnjamAttempts) {
      performAdvancedUnjam();
      unjamCount++;
      stallStartTime = now; // Reset timer
      stallDetected = false;
      return false;
    } else {
      doseState = JAMMED;
      return true;
    }
  }
  
  return false;
}

void performAdvancedUnjam() {
  showMessage("Smart Unjam\nSequence...");
  digitalWrite(LED_BUILTIN, HIGH);
  
  motorOn();
  
  // Progressive unjam sequence
  int baseSteps = cfg.unjamSteps;
  int unjamSteps = baseSteps * (unjamCount + 1);
  
  // Phase 1: Quick reverse
  stepper.setSpeed(-(cfg.slowRPM * 200 / 60.0));
  stepper.move(-unjamSteps);
  while (stepper.distanceToGo() != 0) {
    stepper.runSpeed();
    delay(1);
  }
  
  delay(100);
  
  // Phase 2: Vibration sequence
  for (int i = 0; i < 3; i++) {
    stepper.move(20);
    while (stepper.distanceToGo() != 0) {
      stepper.runSpeed();
    }
    delay(50);
    stepper.move(-20);
    while (stepper.distanceToGo() != 0) {
      stepper.runSpeed();
    }
    delay(50);
  }
  
  // Phase 3: Resume with current speed
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.print("Advanced unjam #"); Serial.println(unjamCount + 1);
}

// ---------------------------------------------------------------
//  LEARNING SYSTEM
// ---------------------------------------------------------------
void performPostDoseLearning(float finalWeight) {
  float actualOverrun = finalWeight - preOverrunWeight;
  float accuracy = abs(finalWeight - targetWeight) / targetWeight;
  
  // Store in learning history
  learning.overrunHistory[learning.overrunIndex] = actualOverrun;
  learning.overrunIndex = (learning.overrunIndex + 1) % 15;
  
  learning.weightHistory[learning.historyIndex] = finalWeight;
  learning.timeHistory[learning.historyIndex] = (millis() - doseStartTime) / 1000.0;
  learning.historyIndex = (learning.historyIndex + 1) % 20;
  
  // Update preset learning
  SmartPreset* preset = &presets[activePreset];
  preset->learnedOverrun = (preset->learnedOverrun * 0.7) + (actualOverrun * 0.3);
  preset->accuracy = (preset->accuracy * 0.8) + (accuracy * 0.2);
  preset->useCount++;
  preset->totalTime += (millis() - doseStartTime);
  
  // Update global learning
  learning.totalDoses++;
  learning.avgAccuracy = (learning.avgAccuracy * 0.9) + (accuracy * 0.1);
  learning.totalDosingTime += (millis() - doseStartTime);
  
  if (accuracy < learning.bestAccuracy || learning.bestAccuracy == 0) {
    learning.bestAccuracy = accuracy;
  }
  
  // Store accuracy for quality check
  lastDoseAccuracy = accuracy;
  
  Serial.print("Learning: Final="); Serial.print(finalWeight);
  Serial.print("g, Overrun="); Serial.print(actualOverrun);
  Serial.print("g, Accuracy="); Serial.print(accuracy * 100); Serial.println("%");
}

// ---------------------------------------------------------------
//  QUALITY CONTROL
// ---------------------------------------------------------------
void performQualityCheck() {
  float tolerance;
  switch (cfg.qualityMode) {
    case 0: tolerance = cfg.toleranceStrict; break;
    case 1: tolerance = cfg.toleranceNormal; break;
    case 2: tolerance = cfg.toleranceLoose; break;
    default: tolerance = cfg.toleranceNormal;
  }
  
  qualityPassed = (lastDoseAccuracy <= tolerance);
  
  if (!qualityPassed) {
    Serial.println("Quality check failed");
  }
}

// ---------------------------------------------------------------
//  ENVIRONMENTAL MONITORING
// ---------------------------------------------------------------
void updateEnvironmentalData() {
  // Temperature sensor removed - no environmental monitoring
}

// ---------------------------------------------------------------
//  BUTTON HANDLING
// ---------------------------------------------------------------
void handleButton() {
  unsigned long now = millis();
  bool btnReading = digitalRead(ENC_SW) == LOW;

  if (btnReading != lastBtnState) {
    lastBtnChange = now;
  }
  
  if ((now - lastBtnChange) > DEBOUNCE_MS) {
    if (btnReading != btnState) {
      btnState = btnReading;
      
      if (btnState) { // Pressed
        if (now - lastPressTime <= DOUBLE_PRESS_MS) {
          handleDoublePress();
        }
        lastPressTime = now;
      } else { // Released
        if (now - lastPressTime >= LONG_PRESS_MS) {
          handleLongPress();
        } else if (now - lastPressTime > DEBOUNCE_MS) {
          handleShortPress();
        }
        
        if (doseState == PURGING) {
          stopMotor();
          doseState = IDLE;
          showMessage("Purge Complete");
          delay(500);
          showIdleScreen();
        }
      }
    }
  }
  lastBtnState = btnReading;
}

void handleShortPress() {
  menuLastAction = millis();
  
  switch (doseState) {
    case IDLE:
      // Single press = Start dosing (most intuitive)
      startDoseSequence();
      break;
    case MENU:
      handleMenuSelect();
      break;
    case STATISTICS:
      handleStatsSelect();
      break;
    case PROFILES:
      handleProfileSelect();
      break;
    case CALIBRATE:
      handleCalibrationStep();
      break;
    default:
      if (doseState == DOSING_FAST || doseState == DOSING_SLOW || doseState == DOSING_ULTRA_SLOW) {
        emergencyStop();
      }
      break;
  }
}

void handleDoublePress() {
  menuLastAction = millis();
  
  switch (doseState) {
    case IDLE:
      // Double press = Cycle presets
      cyclePresets();
      break;
    default:
      // Double press in other states = quick exit to idle
      exitToIdle();
      break;
  }
}

void handleLongPress() {
  menuLastAction = millis();
  
  switch (doseState) {
    case IDLE:
      // Long press = Enter menu
      enterMainMenu();
      break;
    case MENU:
    case STATISTICS:
    case PROFILES:
    case CALIBRATE:
      // Long press in menu = Exit to idle
      exitToIdle();
      break;
    default:
      if (doseState == DOSING_FAST || doseState == DOSING_SLOW || doseState == DOSING_ULTRA_SLOW) {
        emergencyStop();
      } else {
        exitToIdle();
      }
      break;
  }
}

// ---------------------------------------------------------------
//  ENCODER HANDLING
// ---------------------------------------------------------------
void handleEncoder() {
  static unsigned long lastEncoderTime = 0;
  static long lastPos = 0;
  
  // Debounce encoder readings
  if (millis() - lastEncoderTime < 50) return;
  
  long pos = encoder.getPosition();
  
  if (pos == lastPos) return;
  
  lastEncoderTime = millis();
  menuLastAction = millis();
  int direction = (pos > lastPos) ? 1 : -1;
  
  switch (doseState) {
    case IDLE:
      if (btnState) {
        // Encoder + Button = Purge mode
        startPurge();
      } else {
        // Normal encoder = Adjust weight
        adjustPresetWeight(direction);
      }
      break;
    case MENU:
      navigateMenu(direction);
      break;
    case STATISTICS:
      navigateStats(direction);
      break;
    case PROFILES:
      navigateProfiles(direction);
      break;
  }
  
  lastPos = pos;
}

// ---------------------------------------------------------------
//  MENU SYSTEM
// ---------------------------------------------------------------
void enterMainMenu() {
  doseState = MENU;
  menuItem = 0;
  inSubMenu = false;
  showMainMenu();
}

void showMainMenu() {
  const char* menuItems[] = {
    "Motor Settings",
    "Adaptive Control", 
    "Quality Control",
    "Preset Manager",
    "Statistics",
    "Calibration",
    "System Info",
    "Factory Reset",
    "Save & Exit"
  };
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("> ");
  display.println(menuItems[menuItem]);
  
  display.setCursor(0, 16);
  display.print("Item ");
  display.print(menuItem + 1);
  display.print(" of ");
  display.println(9);
  
  display.display();
}

void handleMenuSelect() {
  switch (menuItem) {
    case 0: enterMotorSettings(); break;
    case 1: enterAdaptiveSettings(); break;
    case 2: enterQualitySettings(); break;
    case 3: enterPresetManager(); break;
    case 4: enterStatistics(); break;
    case 5: enterCalibration(); break;
    case 6: showSystemInfo(); break;
    case 7: confirmFactoryReset(); break;
    case 8: saveAndExit(); break;
  }
}

// ---------------------------------------------------------------
//  DISPLAY FUNCTIONS
// ---------------------------------------------------------------
void showBootScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("Smart Bean Doser"));
  display.println(F("v6.0 - AI Enhanced"));
  display.println(F(""));
  display.println(F("Initializing..."));
  display.display();
  delay(2000);
}

void showIdleScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(presets[activePreset].name);
  display.print(" (");
  display.print(presets[activePreset].useCount);
  display.println(")");
  
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print(presets[activePreset].targetWeight, 1);
  display.print("g");
  
  // Show button instructions
  display.setTextSize(1);
  display.setCursor(0, 26);
  display.print("Press=Dose 2x=Preset");
  
  display.display();
  lastDisplayWeight = -99.0;
}

void showMessage(const String &msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 8);
  display.println(msg);
  display.display();
}

void showDoseResults() {
  float finalWeight = getSmartWeight();
  float error = finalWeight - targetWeight;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Result: ");
  display.print(finalWeight, 2);
  display.println("g");
  
  display.setCursor(0, 10);
  display.print("Error: ");
  if (error >= 0) display.print("+");
  display.print(error, 2);
  display.println("g");
  
  display.setCursor(0, 20);
  display.print("Quality: ");
  display.println(qualityPassed ? "PASS" : "FAIL");
  
  display.display();
}

void updateDisplay() {
  // Only update display during dosing states, not in IDLE
  if (doseState == DOSING_FAST || doseState == DOSING_SLOW || doseState == DOSING_ULTRA_SLOW) {
    float w = getSmartWeight();
    if (abs(w - lastDisplayWeight) >= 0.05) {
      showCurrentWeight(w);
      lastDisplayWeight = w;
    }
  }
}

void showCurrentWeight(float weight) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Current Weight:");
  
  display.setTextSize(2);
  display.setCursor(0, 12);
  if (abs(weight) < 0.05) {
    display.print("0.00");
  } else {
    display.print(weight, 2);
  }
  display.print("g");
  
  display.display();
}

// ---------------------------------------------------------------
//  UTILITY FUNCTIONS
// ---------------------------------------------------------------
void cyclePresets() {
  activePreset = (activePreset + 1) % 6;
  targetWeight = presets[activePreset].targetWeight;
  
  // Clear any previous display state
  lastDisplayWeight = -99.0;
  showIdleScreen();
  
  Serial.print("Preset changed to: ");
  Serial.print(presets[activePreset].name);
  Serial.print(" (");
  Serial.print(targetWeight, 1);
  Serial.println("g)");
}

void adjustPresetWeight(int direction) {
  float delta = direction * 0.1;
  float newWeight = presets[activePreset].targetWeight + delta;
  newWeight = constrain(newWeight, 5.0, 50.0);
  
  // Only update if weight actually changed
  if (abs(newWeight - presets[activePreset].targetWeight) > 0.05) {
    presets[activePreset].targetWeight = newWeight;
    targetWeight = presets[activePreset].targetWeight;
    showIdleScreen();
    
    // Brief feedback
    Serial.print("Weight adjusted to: ");
    Serial.print(targetWeight, 1);
    Serial.println("g");
  }
}

void startDoseSequence() {
  targetWeight = presets[activePreset].targetWeight;
  doseState = TARE_CHECK;
  showMessage("Starting Dose...");
}

void startPurge() {
  if (doseState == IDLE) {
    doseState = PURGING;
    showMessage("PURGING...\nRotate+Hold\nRelease to stop");
    setFastSpeed();
    stepper.move(1000000);
    Serial.println("Purge mode activated - rotate encoder while holding button");
  }
}

bool performSmartTareCheck(float weight) {
  if (abs(weight) < 1.0) {
    return true; // Already tared
  } else if (abs(weight) < 3.0) {
    performAdvancedTare();
    return true;
  } else {
    showMessage("Remove beans\nand try again");
    delay(2000);
    doseState = IDLE;
    showIdleScreen();
    return false;
  }
}

void performAdvancedTare() {
  showMessage("Advanced Tare...");
  
  // Multi-point tare for better accuracy
  float readings[10];
  for (int i = 0; i < 10; i++) {
    readings[i] = scale.get_units(5);
    delay(100);
  }
  
  // Calculate average excluding outliers
  float sum = 0;
  int count = 0;
  float avg = 0;
  
  // First pass - calculate rough average
  for (int i = 0; i < 10; i++) {
    sum += readings[i];
  }
  avg = sum / 10.0;
  
  // Second pass - exclude outliers
  sum = 0;
  count = 0;
  for (int i = 0; i < 10; i++) {
    if (abs(readings[i] - avg) < 0.5) {
      sum += readings[i];
      count++;
    }
  }
  
  if (count > 5) {
    float tareValue = sum / count;
    scale.set_offset(scale.get_offset() + (tareValue * scale.get_scale()));
  } else {
    scale.tare(15); // Fallback to standard tare
  }
  
  showMessage("Tare Complete");
  delay(500);
}

bool performPreDoseAnalysis() {
  showMessage("Analyzing...");
  
  // Predict optimal parameters
  if (predictor.modelReady) {
    float predictedTime = targetWeight / predictor.predictedFlowRate;
    Serial.print("Predicted dose time: "); Serial.println(predictedTime);
  }
  
  delay(500);
  return true;
}

void handleJammedState() {
  stopMotor();
  stats.jamCount++;
  showMessage("JAMMED!\nCheck hopper\nPress to retry");
  
  // Wait for button press
  while (digitalRead(ENC_SW) == HIGH) {
    delay(50);
  }
  
  // Debounce
  delay(200);
  while (digitalRead(ENC_SW) == LOW) {
    delay(50);
  }
  
  doseState = IDLE;
  unjamCount = 0;
  showIdleScreen();
}

void emergencyStop() {
  stopMotor();
  doseState = IDLE;
  unjamCount = 0;
  showMessage("EMERGENCY STOP");
  delay(1500);
  showIdleScreen();
}

void motorOn() {
  if (!motorPowered) {
    digitalWrite(ENABLE_PIN, LOW);  // A4988: LOW = enabled
    delay(2);  // Small delay for A4988 to stabilize
    motorPowered = true;
  }
}

void motorOff() {
  if (motorPowered) {
    stepper.stop();
    delay(5);  // Allow motor to stop completely
    digitalWrite(ENABLE_PIN, HIGH);  // A4988: HIGH = disabled (saves power)
    motorPowered = false;
  }
}

void stopMotor() {
  motorOff();
}

// ---------------------------------------------------------------
//  EEPROM FUNCTIONS
// ---------------------------------------------------------------
bool checkMagic() {
  uint32_t magic;
  EEPROM.get(ADDR_MAGIC, magic);
  return magic == MAGIC;
}

void saveAllSettings() {
  EEPROM.put(ADDR_MAGIC, MAGIC);
  EEPROM.put(ADDR_SETTINGS, cfg);
  EEPROM.put(ADDR_PRESETS, presets);
  EEPROM.put(ADDR_LEARNING, learning);
  EEPROM.put(ADDR_STATISTICS, stats);
  EEPROM.commit();
  Serial.println("All settings saved");
}

void loadAllSettings() {
  EEPROM.get(ADDR_SETTINGS, cfg);
  EEPROM.get(ADDR_PRESETS, presets);
  EEPROM.get(ADDR_LEARNING, learning);
  EEPROM.get(ADDR_STATISTICS, stats);
  
  // Validate loaded data
  cfg.fastRPM = constrain(cfg.fastRPM, 60, 300);
  cfg.slowRPM = constrain(cfg.slowRPM, 10, 100);
  cfg.ultraSlowRPM = constrain(cfg.ultraSlowRPM, 5, 50);
  
  for (int i = 0; i < 6; i++) {
    presets[i].targetWeight = constrain(presets[i].targetWeight, 5.0, 50.0);
    presets[i].learnedOverrun = constrain(presets[i].learnedOverrun, 0.1, 2.0);
  }
  
  Serial.println("Settings loaded and validated");
}

void factoryReset() {
  // Reset to defaults
  cfg = AdvancedSettings();
  
  // Reset presets
  strcpy(presets[0].name, "Single");   presets[0].targetWeight = 8.5;
  strcpy(presets[1].name, "Double");   presets[1].targetWeight = 17.0;
  strcpy(presets[2].name, "Triple");   presets[2].targetWeight = 25.5;
  strcpy(presets[3].name, "Light");    presets[3].targetWeight = 6.0;
  strcpy(presets[4].name, "Strong");   presets[4].targetWeight = 20.0;
  strcpy(presets[5].name, "Custom");   presets[5].targetWeight = 15.0;
  
  // Reset learning and stats
  learning = LearningData();
  stats = Statistics();
  
  saveAllSettings();
  
  scale.set_scale(cfg.calFactor);
  initializePredictiveModel();
}

// ---------------------------------------------------------------
//  PLACEHOLDER FUNCTIONS (to be implemented)
// ---------------------------------------------------------------
void enterMotorSettings() { showMessage("Motor Settings\n(Not implemented)"); delay(1000); showMainMenu(); }
void enterAdaptiveSettings() { showMessage("Adaptive Settings\n(Not implemented)"); delay(1000); showMainMenu(); }
void enterQualitySettings() { showMessage("Quality Settings\n(Not implemented)"); delay(1000); showMainMenu(); }
void enterPresetManager() { showMessage("Preset Manager\n(Not implemented)"); delay(1000); showMainMenu(); }
void enterStatistics() { showMessage("Statistics\n(Not implemented)"); delay(1000); showMainMenu(); }
void enterCalibration() { showMessage("Calibration\n(Not implemented)"); delay(1000); showMainMenu(); }
void showSystemInfo() { showMessage("System Info\n(Not implemented)"); delay(1000); showMainMenu(); }
void confirmFactoryReset() { showMessage("Factory Reset\n(Not implemented)"); delay(1000); showMainMenu(); }
void saveAndExit() { saveAllSettings(); doseState = IDLE; showMessage("Saved!"); delay(500); showIdleScreen(); }

void navigateMenu(int direction) { menuItem = constrain(menuItem + direction, 0, 8); showMainMenu(); }
void navigateStats(int direction) { /* Not implemented */ }
void navigateProfiles(int direction) { /* Not implemented */ }
void handleStatsSelect() { /* Not implemented */ }
void handleProfileSelect() { /* Not implemented */ }
void handleCalibrationStep() { /* Not implemented */ }
void exitToIdle() { doseState = IDLE; showMessage("Exiting..."); delay(500); showIdleScreen(); }
void updateStatistics() { /* Statistics updated in learning function */ }
void monitorIdleWeight(float weight) { 
  // Ensure idle screen stays displayed
  static unsigned long lastIdleUpdate = 0;
  if (millis() - lastIdleUpdate > 5000) { // Refresh every 5 seconds
    showIdleScreen();
    lastIdleUpdate = millis();
  }
}

// ---------------------------------------------------------------
//  ISR
// ---------------------------------------------------------------
void encISR() {
  encoder.tick();
}
