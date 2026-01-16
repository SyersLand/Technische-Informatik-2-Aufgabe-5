#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>

// ==================== OLED SETUP ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==================== MPU6050 SETUP ====================
Adafruit_MPU6050 mpu;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

// ==================== SERVO SETUP ====================
const int SERVO_PIN = 27;
const int SERVO_PWM_CHANNEL = 0;
const int SERVO_PWM_FREQ = 50;
const int SERVO_PWM_RESOLUTION = 12;

// ==================== PIN DEFINITIONS ====================
const int POT_PIN = 14;
const int LDR_PIN = 13;
const int BUTTON_P3 = 35;
const int BUTTON_S2 = 34;

// Traffic Light LEDs
const int RED1_PIN = 19;
const int YELLOW1_PIN = 18;
const int GREEN1_PIN = 5;
const int RED2_PIN = 16;
const int YELLOW2_PIN = 4;
const int GREEN2_PIN = 2;

// Mode Indicator LEDs (4 LED)
const int MODE_LEDS[4] = {32, 33, 25, 26};

// ==================== CONSTANTS ====================
const int MODE_COUNT = 4;
const int DAY_NIGHT_THRESHOLD = 3000;
const unsigned long BLINK_INTERVAL = 500;
const unsigned long PEDESTRIAN_DURATION = 10000;
const unsigned long ALL_RED_DURATION = 2000;
const unsigned long YELLOW_DURATION = 3000;
const unsigned long DISPLAY_UPDATE_INTERVAL = 300;
const unsigned long MPU_READ_INTERVAL = 100;
const unsigned long BARRIER_CLOSE_DELAY = 4000;

// ==================== GLOBAL VARIABLES ====================
int currentMode = 0;
bool isNightMode = false;
bool yellowState = false;
unsigned long lastBlinkTime = 0;
bool pedestrianRequest = false;
unsigned long pedestrianRequestTime = 0;
int greenPhaseDuration[4] = {5000, 7000, 10000, 15000};

// Servo variables
int currentPotValue = 0;
bool isBarrierOpen = false;
unsigned long barrierCloseTime = 0;

// Traffic State
enum TrafficState {
  GREEN1_RED2,
  YELLOW1_RED2,
  ALL_RED_TRANSITION,
  RED1_GREEN2,
  RED1_YELLOW2,
  ALL_RED_TRANSITION_2,
  PEDESTRIAN_CROSSING,
  BARRIER_MODE,
  WAIT_AFTER_BARRIER
};

TrafficState currentState = ALL_RED_TRANSITION;
unsigned long stateStartTime = 0;

// Display update timing
unsigned long lastDisplayUpdate = 0;
unsigned long lastMPURead = 0;

// ==================== FUNCTION DECLARATIONS ====================
void setupOLED();
void setupMPU6050();
void readMPU6050();
void setServoAngle(int angle);
void calibrateLDR();
void checkNightMode();
void readPotentiometer();
void checkButtons();
void emergencyStop();
void handlePedestrianRequest();
void updateModeLEDs();
void nightModeOperation();
void normalModeOperation();
void setLights(int red1, int yellow1, int green1, int red2, int yellow2, int green2);
void printState();
void updateDisplay();
void handleBarrierMode();
void handleWaitAfterBarrier();

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== TRAFFIC LIGHT WITH MPU6050 & OLED ===");
  Serial.println("Integrated System with Pedestrian Crossing");
  
  // Setup I2C
  Wire.begin(21, 22);
  
  // Setup OLED
  setupOLED();
  
  // Setup MPU6050
  setupMPU6050();
  
  // Setup LED pins
  pinMode(RED1_PIN, OUTPUT);
  pinMode(YELLOW1_PIN, OUTPUT);
  pinMode(GREEN1_PIN, OUTPUT);
  pinMode(RED2_PIN, OUTPUT);
  pinMode(YELLOW2_PIN, OUTPUT);
  pinMode(GREEN2_PIN, OUTPUT);
  
  for (int i = 0; i < MODE_COUNT; i++) {
    pinMode(MODE_LEDS[i], OUTPUT);
    digitalWrite(MODE_LEDS[i], LOW);
  }
  
  // Setup buttons
  pinMode(BUTTON_P3, INPUT_PULLUP);
  pinMode(BUTTON_S2, INPUT_PULLUP);
  
  // Setup Servo
  ledcSetup(SERVO_PWM_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, SERVO_PWM_CHANNEL);
  setServoAngle(0);
  
  // Kalibrasi
  calibrateLDR();
  
  // Inisialisasi
  setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
  digitalWrite(MODE_LEDS[0], HIGH);
  
  // Baca nilai awal potentiometer
  currentPotValue = analogRead(POT_PIN);
  setServoAngle(map(currentPotValue, 0, 4095, 0, 180));
  
  stateStartTime = millis();
  
  // Display startup message
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Integrated System");
  display.println("MPU6050 + OLED");
  display.println("Pedestrian Ready");
  display.display();
  
  delay(2000);
  
  Serial.println("System initialized");
  printState();
}

void setupOLED() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
  } else {
    Serial.println("OLED ready");
  }
}

void setupMPU6050() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// ==================== SERVO FUNCTION ====================
void setServoAngle(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  int duty = (pulseWidth * 4095) / 20000;
  ledcWrite(SERVO_PWM_CHANNEL, duty);
  
  // Update barrier state
  bool newBarrierOpen = (angle > 120);
  if (newBarrierOpen != isBarrierOpen) {
    isBarrierOpen = newBarrierOpen;
    Serial.print("Barrier: ");
    Serial.println(isBarrierOpen ? "OPEN" : "CLOSED");
    if (!isBarrierOpen) {
      barrierCloseTime = millis();
    }
  }
}

// ==================== MPU6050 READING ====================
void readMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentTime = millis();
  
  // 1. Baca MPU6050
  if (currentTime - lastMPURead >= MPU_READ_INTERVAL) {
    readMPU6050();
    lastMPURead = currentTime;
  }
  
  // 2. Handle potentiometer untuk servo
  static unsigned long lastPotRead = 0;
  if (currentTime - lastPotRead >= 50) {
    int potValue = analogRead(POT_PIN);
    if (abs(potValue - currentPotValue) > 10) {
      currentPotValue = potValue;
      setServoAngle(map(potValue, 0, 4095, 0, 180));
    }
    lastPotRead = currentTime;
  }
  
  // 3. Check night mode
  checkNightMode();
  
  // 4. Read potentiometer for mode
  readPotentiometer();
  
  // 5. Check buttons (jika bukan night mode)
  if (!isNightMode) {
    checkButtons();
  }
  
  // 6. Handle pedestrian request
  if (pedestrianRequest) {
    handlePedestrianRequest();
  }
  
  // 7. Update display
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = currentTime;
  }
  
  // 8. Main traffic logic
  if (isNightMode) {
    nightModeOperation();
  } else {
    // Check barrier mode
    if (isBarrierOpen && currentState != BARRIER_MODE && 
        currentState != PEDESTRIAN_CROSSING && currentState != WAIT_AFTER_BARRIER) {
      currentState = BARRIER_MODE;
      stateStartTime = currentTime;
      Serial.println("BARRIER OPEN - Entering barrier mode");
    }
    
    switch(currentState) {
      case BARRIER_MODE:
        handleBarrierMode();
        break;
      case WAIT_AFTER_BARRIER:
        handleWaitAfterBarrier();
        break;
      case PEDESTRIAN_CROSSING:
        // Ditangani di handlePedestrianRequest()
        break;
      default:
        normalModeOperation();
        break;
    }
  }
  
  delay(10);
}

// ==================== HELPER FUNCTIONS ====================
void setLights(int red1, int yellow1, int green1, int red2, int yellow2, int green2) {
  digitalWrite(RED1_PIN, red1);
  digitalWrite(YELLOW1_PIN, yellow1);
  digitalWrite(GREEN1_PIN, green1);
  digitalWrite(RED2_PIN, red2);
  digitalWrite(YELLOW2_PIN, yellow2);
  digitalWrite(GREEN2_PIN, green2);
}

void printState() {
  Serial.print("State: ");
  switch(currentState) {
    case GREEN1_RED2: Serial.println("GREEN 1 - RED 2"); break;
    case YELLOW1_RED2: Serial.println("YELLOW 1 - RED 2"); break;
    case ALL_RED_TRANSITION: Serial.println("ALL RED (transition)"); break;
    case RED1_GREEN2: Serial.println("RED 1 - GREEN 2"); break;
    case RED1_YELLOW2: Serial.println("RED 1 - YELLOW 2"); break;
    case ALL_RED_TRANSITION_2: Serial.println("ALL RED (transition 2)"); break;
    case PEDESTRIAN_CROSSING: Serial.println("PEDESTRIAN CROSSING"); break;
    case BARRIER_MODE: Serial.println("BARRIER MODE"); break;
    case WAIT_AFTER_BARRIER: Serial.println("WAIT AFTER BARRIER"); break;
  }
}

void calibrateLDR() {
  Serial.println("LDR Calibration...");
  delay(500);
  Serial.print("Initial LDR: ");
  Serial.println(analogRead(LDR_PIN));
}

void checkNightMode() {
  int ldrValue = analogRead(LDR_PIN);
  bool nightDetected = (ldrValue > DAY_NIGHT_THRESHOLD);
  
  if (nightDetected && !isNightMode) {
    isNightMode = true;
    pedestrianRequest = false;
    Serial.println("=== NIGHT MODE (Yellow Blink) ===");
    setLights(LOW, LOW, LOW, LOW, LOW, LOW);
    
    for (int i = 0; i < MODE_COUNT; i++) {
      digitalWrite(MODE_LEDS[i], LOW);
    }
  } 
  else if (!nightDetected && isNightMode) {
    isNightMode = false;
    Serial.println("=== DAY MODE ===");
    currentState = ALL_RED_TRANSITION;
    stateStartTime = millis();
    setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
    digitalWrite(MODE_LEDS[currentMode], HIGH);
    printState();
  }
}

void readPotentiometer() {
  int potValue = analogRead(POT_PIN);
  int newMode = potValue / 1024;
  
  if (newMode >= MODE_COUNT) {
    newMode = MODE_COUNT - 1;
  }
  
  if (newMode != currentMode) {
    currentMode = newMode;
    updateModeLEDs();
    
    Serial.print("Changed to Mode ");
    Serial.print(currentMode + 1);
    Serial.print(" (");
    Serial.print(greenPhaseDuration[currentMode] / 1000);
    Serial.println("s green)");
  }
}

void checkButtons() {
  static bool lastS2State = HIGH;
  static bool lastP3State = HIGH;
  static unsigned long lastDebounceTime = 0;
  
  unsigned long now = millis();
  if (now - lastDebounceTime < 100) return;
  lastDebounceTime = now;
  
  bool s2State = digitalRead(BUTTON_S2);
  bool p3State = digitalRead(BUTTON_P3);
  
  // Button S2 - Pedestrian
  if (s2State == LOW && lastS2State == HIGH) {
    Serial.println("Button S2: Pedestrian Request");
    if (!pedestrianRequest) {
      pedestrianRequest = true;
      pedestrianRequestTime = millis();
    }
  }
  
  // Button P3 - Emergency
  if (p3State == LOW && lastP3State == HIGH) {
    Serial.println("Button P3: Emergency Stop");
    emergencyStop();
  }
  
  lastS2State = s2State;
  lastP3State = p3State;
}

void emergencyStop() {
  Serial.println("!!! EMERGENCY - ALL RED !!!");
  setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
  delay(3000);
  
  currentState = ALL_RED_TRANSITION;
  stateStartTime = millis();
  Serial.println("Emergency cleared");
  printState();
}

void handlePedestrianRequest() {
  if (currentState != PEDESTRIAN_CROSSING) {
    currentState = PEDESTRIAN_CROSSING;
    Serial.println("=== PEDESTRIAN CROSSING ===");
    setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
    stateStartTime = millis();
  }
  
  // Cek waktu (10 detik)
  if (millis() - pedestrianRequestTime >= PEDESTRIAN_DURATION) {
    Serial.println("Pedestrian crossing finished");
    pedestrianRequest = false;
    currentState = ALL_RED_TRANSITION;
    stateStartTime = millis();
    printState();
  }
}

void updateModeLEDs() {
  for (int i = 0; i < MODE_COUNT; i++) {
    digitalWrite(MODE_LEDS[i], LOW);
  }
  digitalWrite(MODE_LEDS[currentMode], HIGH);
}

void nightModeOperation() {
  if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
    yellowState = !yellowState;
    setLights(LOW, yellowState, LOW, LOW, yellowState, LOW);
    lastBlinkTime = millis();
  }
}

void normalModeOperation() {
  if (pedestrianRequest) return;
  
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - stateStartTime;
  
  switch (currentState) {
    case GREEN1_RED2:
      setLights(LOW, LOW, HIGH, HIGH, LOW, LOW);
      
      if (elapsed >= greenPhaseDuration[currentMode]) {
        currentState = YELLOW1_RED2;
        stateStartTime = currentTime;
        Serial.println("Green phase ended, switching to YELLOW");
        printState();
      }
      break;
      
    case YELLOW1_RED2:
      setLights(LOW, HIGH, LOW, HIGH, LOW, LOW);
      
      if (elapsed >= YELLOW_DURATION) {
        currentState = ALL_RED_TRANSITION;
        stateStartTime = currentTime;
        Serial.println("Yellow ended, ALL RED for safety");
        printState();
      }
      break;
      
    case ALL_RED_TRANSITION:
      setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
      
      if (elapsed >= ALL_RED_DURATION) {
        currentState = RED1_GREEN2;
        stateStartTime = currentTime;
        Serial.println("Now Traffic Light 2 gets GREEN");
        printState();
      }
      break;
      
    case RED1_GREEN2:
      setLights(HIGH, LOW, LOW, LOW, LOW, HIGH);
      
      if (elapsed >= greenPhaseDuration[currentMode]) {
        currentState = RED1_YELLOW2;
        stateStartTime = currentTime;
        Serial.println("Green phase 2 ended, switching to YELLOW");
        printState();
      }
      break;
      
    case RED1_YELLOW2:
      setLights(HIGH, LOW, LOW, LOW, HIGH, LOW);
      
      if (elapsed >= YELLOW_DURATION) {
        currentState = ALL_RED_TRANSITION_2;
        stateStartTime = currentTime;
        Serial.println("Yellow 2 ended, ALL RED for safety");
        printState();
      }
      break;
      
    case ALL_RED_TRANSITION_2:
      setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
      
      if (elapsed >= ALL_RED_DURATION) {
        currentState = GREEN1_RED2;
        stateStartTime = currentTime;
        Serial.println("Cycle complete, back to Traffic Light 1 GREEN");
        printState();
      }
      break;
      
    case PEDESTRIAN_CROSSING:
      // Ditangani di handlePedestrianRequest()
      break;
  }
}

void handleBarrierMode() {
  setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
  
  if (!isBarrierOpen && barrierCloseTime == 0) {
    barrierCloseTime = millis();
    currentState = WAIT_AFTER_BARRIER;
    Serial.println("Barrier closed, waiting 4 seconds...");
  }
}

void handleWaitAfterBarrier() {
  setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
  
  if (millis() - barrierCloseTime >= BARRIER_CLOSE_DELAY) {
    barrierCloseTime = 0;
    currentState = ALL_RED_TRANSITION;
    stateStartTime = millis();
    Serial.println("Returning to normal traffic cycle");
  }
}

// ==================== DISPLAY FUNCTION WITH XYZ ====================
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Line 1: System Status
  display.setCursor(0, 0);
  display.print("S:");
  const char* states[] = {"G1R2","Y1R2","TR1","R1G2","R1Y2","TR2","PED","BAR","WAIT"};
  int stateIdx = currentState;
  if (stateIdx < 9) {
    display.print(states[stateIdx]);
  }
  
  display.print(" M:");
  display.print(currentMode + 1);
  display.print(" N:");
  display.print(isNightMode ? "Y" : "N");
  
  // Line 2: Potentiometer & Barrier
  display.setCursor(0, 10);
  display.print("Pot:");
  display.print(currentPotValue);
  display.print(" B:");
  display.print(isBarrierOpen ? "OPEN" : "CLOSED");
  
  // Line 3: MPU6050 Accelerometer X
  display.setCursor(0, 20);
  display.print("X:");
  display.print(accelX, 1);
  display.print(" m/s²");
  
  // Line 4: MPU6050 Accelerometer Y
  display.setCursor(0, 30);
  display.print("Y:");
  display.print(accelY, 1);
  display.print(" m/s²");
  
  // Line 5: MPU6050 Accelerometer Z
  display.setCursor(0, 40);
  display.print("Z:");
  display.print(accelZ, 1);
  display.print(" m/s²");
  
  // Line 6: Pedestrian Status & Green Time
  display.setCursor(0, 50);
  display.print("Ped:");
  if (pedestrianRequest) {
    display.print("CROSS");
    display.print((PEDESTRIAN_DURATION - (millis() - pedestrianRequestTime)) / 1000);
    display.print("s");
  } else {
    display.print("READY");
  }
  
  display.print(" G:");
  display.print(greenPhaseDuration[currentMode] / 1000);
  display.print("s");
  
  display.display();
}