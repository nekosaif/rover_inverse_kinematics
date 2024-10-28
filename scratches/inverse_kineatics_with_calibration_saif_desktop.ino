/**
 * inverse_kineatics_with_calibration.ino
 * 
 * This Arduino sketch controls a robotic arm with two actuators and a rotating base.
 * It provides functionality for calibration, movement control, and position memory using EEPROM.
 * 
 * Hardware Configuration:
 * - 3 Rotating base with potentiometer feedback
 * - Calibration buttons
 * - LED indicator for status
 * 
 * @author Original code modified and documented
 * @version 2.0
 */

#include <EEPROM.h>

// Pin Definitions
struct PinConfig {
  // Actuator 1 pins
  static const uint8_t ACT1_UP = 3;
  static const uint8_t ACT1_DOWN = 2;
  static const uint8_t ACT1_POT = A3;
  
  // Actuator 2 pins
  static const uint8_t ACT2_UP = 5;
  static const uint8_t ACT2_DOWN = 4;
  static const uint8_t ACT2_POT = A4;
  
  // Base rotation pins
  static const uint8_t BASE_RIGHT = 6;
  static const uint8_t BASE_LEFT = 7;
  static const uint8_t BASE_POT = A5;
  
  // Control pins
  static const uint8_t CALIBRATION = 8;
  static const uint8_t BASE_CALIBRATION = 9;
};

// EEPROM address mapping for potentiometer values
struct EEPROMMap {
  static const uint8_t ACT1_POT_MIN = 0;
  static const uint8_t ACT1_POT_MAX = 1;
  static const uint8_t ACT2_POT_MIN = 2;
  static const uint8_t ACT2_POT_MAX = 3;
  static const uint8_t BASE_POT_MIN = 4;
  static const uint8_t BASE_POT_MAX = 5;
};

// Configuration constants
struct Config {
  static const uint8_t EEPROM_START_ADDR = 0;
  static const uint8_t CALIBRATION_BLINK_TIME = 3;
  static const uint8_t STALL_TIME = 1;
  static const uint8_t POT_OFFSET = 5;
  static const uint8_t POT_TOLERANCE = 3;
  
  // Movement limits (in degrees)
  static const uint8_t ACT1_MIN_ANGLE = 0;
  static const uint8_t ACT1_MAX_ANGLE = 98;
  static const uint8_t ACT2_MIN_ANGLE = 35;
  static const uint8_t ACT2_MAX_ANGLE = 134;
  static const uint8_t BASE_MIN_ANGLE = 0;
  static const uint8_t BASE_MAX_ANGLE = 90;
};

// Global variables
uint8_t potValues[6] = {255, 255, 255, 255, 255, 255}; // Initialize with invalid values

/**
 * Initial setup of the controller
 */
void setup() {
  Serial.begin(9600);
  setupPins();
  
  if (!performCalibrationCheck()) {
    loadCalibrationFromEEPROM();
  }
}

/**
 * Main program loop
 */
void loop() {
  if (Serial.available() > 0) {
    processSerialCommand();
  }
}

/**
 * Configure all input and output pins
 */
void setupPins() {
  // Configure LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Configure actuator pins
  configurePinPair(PinConfig::ACT1_UP, PinConfig::ACT1_DOWN);
  configurePinPair(PinConfig::ACT2_UP, PinConfig::ACT2_DOWN);
  configurePinPair(PinConfig::BASE_RIGHT, PinConfig::BASE_LEFT);
  
  // Configure potentiometer inputs
  pinMode(PinConfig::ACT1_POT, INPUT);
  pinMode(PinConfig::ACT2_POT, INPUT);
  pinMode(PinConfig::BASE_POT, INPUT);
  
  // Configure control inputs
  pinMode(PinConfig::CALIBRATION, INPUT_PULLUP);
  pinMode(PinConfig::BASE_CALIBRATION, INPUT_PULLUP);
}

/**
 * Configure a pair of motor control pins
 */
void configurePinPair(uint8_t pinA, uint8_t pinB) {
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
}

/**
 * Check if calibration is needed during startup
 * @return true if calibration was performed, false otherwise
 */
bool performCalibrationCheck() {
  for (int i = 0; i < Config::CALIBRATION_BLINK_TIME * 2; ++i) {
    blinkLED(250);
    
    if (!digitalRead(PinConfig::CALIBRATION)) {
      indicateCalibrationStart();
      performCalibration();
      return true;
    }
  }
  return false;
}

/**
 * Load calibration values from EEPROM
 */
void loadCalibrationFromEEPROM() {
  for (int i = 0; i < 6; ++i) {
    potValues[i] = EEPROM.read(Config::EEPROM_START_ADDR + i);
    Serial.println(potValues[i]);
  }
}

/**
 * Save calibration values to EEPROM
 */
void saveCalibrationToEEPROM() {
  for (int i = 0; i < 6; ++i) {
    EEPROM.update(Config::EEPROM_START_ADDR + i, potValues[i]);
  }
}

/**
 * Process incoming serial commands
 * Format: [actuator_number][position_value]
 */
void processSerialCommand() {
  String command = Serial.readString();
  int actuator = command.substring(0, 1).toInt();
  int position = command.substring(1).toInt();
  moveToPosition(actuator, position);
}

/**
 * Move specified actuator to target position
 * @param actuator Actuator number (1-3)
 * @param targetPosition Target position in degrees
 */
void moveToPosition(uint8_t actuator, int targetPosition) {
  struct ActuatorPins {
    uint8_t up;
    uint8_t down;
    uint8_t pot;
  } pins;
  
  int mappedTarget = getMappedPosition(actuator, targetPosition, pins);
  
  while (true) {
    int currentPosition = map(analogRead(pins.pot), 0, 1023, 0, 255);
    
    if (abs(currentPosition - mappedTarget) <= Config::POT_OFFSET) {
      digitalWrite(pins.up, HIGH);
      digitalWrite(pins.down, HIGH);
      break;
    }
    
    // Pulse the appropriate direction
    if (currentPosition < mappedTarget - Config::POT_OFFSET) {
      pulsePin(pins.down, 100);
    } else if (currentPosition > mappedTarget + Config::POT_OFFSET) {
      pulsePin(pins.up, 100);
    }
  }
}

/**
 * Get mapped position and pin configuration for specified actuator
 */
int getMappedPosition(uint8_t actuator, int position, struct ActuatorPins& pins) {
  switch (actuator) {
    case 1:
      pins = {PinConfig::ACT1_UP, PinConfig::ACT1_DOWN, PinConfig::ACT1_POT};
      return map(position, Config::ACT1_MIN_ANGLE, Config::ACT1_MAX_ANGLE, 
                potValues[EEPROMMap::ACT1_POT_MAX], potValues[EEPROMMap::ACT1_POT_MIN]);
    
    case 2:
      pins = {PinConfig::ACT2_DOWN, PinConfig::ACT2_UP, PinConfig::ACT2_POT};
      return map(position, Config::ACT2_MIN_ANGLE, Config::ACT2_MAX_ANGLE,
                potValues[EEPROMMap::ACT2_POT_MIN], potValues[EEPROMMap::ACT2_POT_MAX]);
    
    case 3:
      pins = {PinConfig::BASE_RIGHT, PinConfig::BASE_LEFT, PinConfig::BASE_POT};
      return map(position, Config::BASE_MIN_ANGLE, Config::BASE_MAX_ANGLE,
                potValues[EEPROMMap::BASE_POT_MIN], potValues[EEPROMMap::BASE_POT_MAX]);
    
    default:
      return -1;
  }
}

/**
 * Perform full calibration sequence
 */
void performCalibration() {
  Serial.println("Starting calibration sequence...");
  
  // Calibrate actuators
  calibrateActuator2Max();
  delay(1000);
  calibrateActuator1Max();
  delay(1000);
  calibrateActuator1Min();
  delay(1000);
  calibrateActuator2Min();
  delay(1000);
  
  indicateActuatorCalibrationComplete();
  
  // Wait for safety confirmation
  while (digitalRead(PinConfig::CALIBRATION)) {
    delay(100);
  }
  
  // Calibrate base
  calibrateBaseMin();
  delay(1000);
  calibrateBaseMax();
  delay(1000);
  
  // Save and complete
  saveCalibrationToEEPROM();
  Serial.println("Calibration complete");
  
  // Log calibration values
  for (int i = 0; i < 6; ++i) {
    Serial.print(potValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}

/**
 * Calibration helper functions
 */
void calibrateActuator1Min() {
  calibrateEndpoint(PinConfig::ACT1_POT, PinConfig::ACT1_UP, PinConfig::ACT1_DOWN,
                   EEPROMMap::ACT1_POT_MIN, "ACT1 MIN");
}

void calibrateActuator1Max() {
  calibrateEndpoint(PinConfig::ACT1_POT, PinConfig::ACT1_DOWN, PinConfig::ACT1_UP,
                   EEPROMMap::ACT1_POT_MAX, "ACT1 MAX");
}

void calibrateActuator2Min() {
  calibrateEndpoint(PinConfig::ACT2_POT, PinConfig::ACT2_DOWN, PinConfig::ACT2_UP,
                   EEPROMMap::ACT2_POT_MIN, "ACT2 MIN");
}

void calibrateActuator2Max() {
  calibrateEndpoint(PinConfig::ACT2_POT, PinConfig::ACT2_UP, PinConfig::ACT2_DOWN,
                   EEPROMMap::ACT2_POT_MAX, "ACT2 MAX");
}

/**
 * Generic endpoint calibration function
 */
void calibrateEndpoint(uint8_t potPin, uint8_t movePin, uint8_t oppositePin,
                      uint8_t storageIndex, const char* calibrationName) {
  Serial.print("Calibrating ");
  Serial.println(calibrationName);
  
  int potValue = map(analogRead(potPin), 0, 1023, 0, 255);
  unsigned long updateTime = millis();
  
  digitalWrite(movePin, LOW);
  
  while (true) {
    if (millis() > updateTime + Config::STALL_TIME * 1000 || 
        !digitalRead(PinConfig::CALIBRATION)) {
      
      if (abs(potValue - map(analogRead(potPin), 0, 1023, 0, 255)) < Config::POT_TOLERANCE) {
        digitalWrite(movePin, HIGH);
        digitalWrite(oppositePin, HIGH);
        potValues[storageIndex] = potValue;
        return;
      }
    }
    
    int newValue = map(analogRead(potPin), 0, 1023, 0, 255);
    if (potValue != newValue) {
      updateTime = millis();
      potValue = newValue;
    }
  }
}

/**
 * Utility functions
 */
void blinkLED(int duration) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(duration);
  digitalWrite(LED_BUILTIN, LOW);
  delay(duration);
}

void pulsePin(uint8_t pin, int duration) {
  digitalWrite(pin, LOW);
  delay(duration);
  digitalWrite(pin, HIGH);
}

void indicateCalibrationStart() {
  for (int i = 0; i < 20; ++i) {
    blinkLED(50);
  }
}

void indicateActuatorCalibrationComplete() {
  for (int i = 0; i < 5; ++i) {
    blinkLED(500);
  }
}
