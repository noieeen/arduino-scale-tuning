//
//    FILE: HX_is_ready.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: HX711 demo with OLED display - optimized for gram measurement
//     URL: https://github.com/RobTillaart/HX711

#include "HX711.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// HX711 configuration
HX711 scale;
uint8_t dataPin = 13;
uint8_t clockPin = 12;

// Measurement settings - OPTIMIZED FOR SPEED
#define NUM_READINGS 1         // Reduced from 5 for faster response
#define DISPLAY_THRESHOLD 0.1  // Lower threshold for quicker updates
#define STABILIZATION_TIME 10  // Minimal delay (was 100ms)

// Moving average filter for smoothing
#define FILTER_SIZE 3          // Reduced from 5 for faster response
float readings[FILTER_SIZE];
int readIndex = 0;
float total = 0;
float average = 0;

// Display update control
float lastDisplayedWeight = 0.0;
unsigned long lastUpdateTime = 0;
int notReadyCount = 0;
int readyCount = 0;

// Function declarations
void displayWeight(float weight, bool forceUpdate = false);
float getFilteredWeight();
void displayError(String message);

void setup() {
  Serial.begin(115200);
  delay(500);  // Reduced from 1000ms
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("HX711_LIB_VERSION: ");
  Serial.println(HX711_LIB_VERSION);
  Serial.println();

  // Initialize display first
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Initializing..."));
  display.display();

  // Initialize HX711
  Serial.println("Initializing HX711...");
  Serial.print("Data Pin: ");
  Serial.println(dataPin);
  Serial.print("Clock Pin: ");
  Serial.println(clockPin);
  
  scale.begin(dataPin, clockPin);
  
  // Wait for HX711 to stabilize - reduced delay
  delay(500);  // Reduced from 1000ms
  
  // Check if HX711 is responding - faster test
  Serial.println("Testing HX711 connection...");
  int testCount = 0;
  for (int i = 0; i < 5; i++) {  // Reduced from 10 tests
    if (scale.is_ready()) {
      testCount++;
    }
    delay(50);  // Reduced from 100ms
  }
  
  Serial.print("HX711 ready count: ");
  Serial.print(testCount);
  Serial.println("/5");
  
  if (testCount == 0) {
    Serial.println("ERROR: HX711 never ready!");
    displayError("HX711 Error!\nCheck wiring");
    for (;;);
  }
  
  // Set gain to 128
  scale.set_gain(128);
  delay(200);  // Reduced from 500ms
  
  // Calibration factor for grams
  scale.set_scale(420.0983);
  
  Serial.println("Remove weight, taring in 2s...");
  display.clearDisplay();
  display.setCursor(0, 8);
  display.println(F("Remove weight"));
  display.println(F("Taring..."));
  display.display();
  delay(2000);  // Reduced from 3000ms
  
  // Tare with fewer samples for faster startup
  Serial.println("Taring scale...");
  scale.tare(10);  // Reduced from 20
  
  Serial.println("Scale ready!");

  // Initialize moving average filter
  for (int i = 0; i < FILTER_SIZE; i++) {
    readings[i] = 0;
  }

  // Show ready screen - shorter delay
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 8);
  display.println(F("READY"));
  display.display();
  delay(500);  // Reduced from 1000ms
  
  displayWeight(0.0, true);
}

float getFilteredWeight() {
  // Subtract the last reading
  total = total - readings[readIndex];
  
  // Read new value - no retry delays for speed
  float newReading = 0;
  
  if (scale.is_ready()) {
    newReading = scale.get_units(NUM_READINGS);
  }
  
  readings[readIndex] = newReading;
  total = total + readings[readIndex];
  
  // Advance to the next position
  readIndex = (readIndex + 1) % FILTER_SIZE;
  
  // Calculate the average
  average = total / FILTER_SIZE;
  
  return average;
}

void displayWeight(float weight, bool forceUpdate) {
  // Update display more frequently
  if (!forceUpdate && abs(weight - lastDisplayedWeight) < DISPLAY_THRESHOLD) {
    return;
  }
  
  display.clearDisplay();
  
  // Title
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Weight:"));
  
  // Weight value
  display.setTextSize(2);
  display.setCursor(0, 12);
  
  // Format weight with 2 decimal places
  if (abs(weight) < 0.01) {
    display.print(F("0.00"));
  } else {
    display.print(weight, 2);
  }
  
  // Unit
  display.setTextSize(1);
  display.print(F(" g"));
  
  // Connection status indicator
  display.setCursor(100, 0);
  if (readyCount > notReadyCount) {
    display.print(F("OK"));
  } else {
    display.print(F("?"));
  }
  
  display.display();
  lastDisplayedWeight = weight;
}

void displayError(String message) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 8);
  display.println(message);
  display.display();
}

void loop() {
  if (scale.is_ready()) {
    readyCount++;
    notReadyCount = 0;
    
    // Get filtered weight reading
    float currentWeight = getFilteredWeight();
    
    // Round to 2 decimal places
    currentWeight = round(currentWeight * 100.0) / 100.0;
    
    // Handle very small values as zero
    if (abs(currentWeight) < 0.01) {
      currentWeight = 0.0;
    }
    
    // Serial output for monitoring
    Serial.print("Weight: ");
    Serial.print(currentWeight, 2);
    Serial.println(" g");
    
    // Update display
    displayWeight(currentWeight);
    
    // Minimal delay for maximum responsiveness
    delay(STABILIZATION_TIME);
    
  } else {
    // Scale not ready
    notReadyCount++;
    
    if (notReadyCount > 10) {  // Increased tolerance
      Serial.println("Scale not ready!");
      displayError("Connection\nError!");
    }
    
    delay(10);  // Very short delay
  }
}

//  -- END OF FILE --
