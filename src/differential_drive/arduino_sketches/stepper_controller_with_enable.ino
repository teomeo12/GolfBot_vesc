/*
  GolfBot Enhanced Stepper Controller with Enable Pin Logic

  This sketch controls a stepper motor using the AccelStepper library and
  provides a serial command interface for configuration and control.

  It is designed to work with the ROS2 'stepper_controller_node.py' and
  uses an ENABLE pin to energize/de-energize the motor, saving power.

  Hardware:
  - Arduino Uno
  - A4988 Stepper Driver
  - NEMA 17 Stepper Motor (or similar)

  Wiring:
  - Arduino D2 -> A4988 STEP
  - Arduino D3 -> A4988 DIR
  - Arduino D8 -> A4988 ENABLE (GND on driver if not used)
  - A4988 RESET and SLEEP pins should be tied together.
  - Power and motor connections as per A4988 datasheet.
*/

#include <AccelStepper.h>
#include <EEPROM.h>

// --- Pin Definitions ---
const int stepPin = 2;
const int dirPin = 3;
const int enablePin = 8; // A4988 ENABLE pin (active LOW)

// Create an AccelStepper instance
// Interface Type 1 = Driver (STEP/DIR pins)
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// --- Configuration Struct ---
// This struct holds all our stepper motor settings.
// We use it to easily save/load from EEPROM.
struct StepperConfig {
  long version; // To check if EEPROM data is valid
  float maxSpeed;
  float acceleration;
  long steps;
  int direction; // 1 for forward, -1 for reverse
};

StepperConfig config;
const long CONFIG_VERSION = 12345; // Magic number to identify our config in EEPROM
const int EEPROM_ADDR = 0;

// --- State Variables ---
bool motorRunning = false;

// --- Function Prototypes ---
void saveConfig();
void loadConfig();
void resetConfig();
void printConfig();
void executeCommand(String cmd);

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  // Configure the enable pin and set it HIGH to keep motor disabled
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH); // Disable driver by default (active LOW)

  // Load configuration from EEPROM or use defaults
  loadConfig();

  // Set initial motor parameters from loaded config
  stepper.setMaxSpeed(config.maxSpeed);
  stepper.setAcceleration(config.acceleration);
  stepper.setPinsInverted(config.direction == -1, false, false);

  Serial.println("==== Arduino Stepper Controller Ready ====");
  Serial.println("State: DISABLED");
  printConfig();
  Serial.println("==========================================");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    executeCommand(command);
  }

  if (motorRunning) {
    // If we have reached the target, stop running (for non-continuous moves)
    if (stepper.distanceToGo() == 0) {
        motorRunning = false;
        digitalWrite(enablePin, HIGH); // Disable motor driver
        Serial.println("INFO: Target reached. Motor disabled.");
    } else {
        stepper.run(); // Continue running
    }
  }
}

// --- Command Handling ---
void executeCommand(String cmd) {
  cmd.toUpperCase();
  Serial.println("CMD: " + cmd);

  if (cmd.startsWith("C:")) { // Configure: C:steps:speed:accel:dir
    // Example: C:1600:4000:2000:1
    int first = cmd.indexOf(':');
    int second = cmd.indexOf(':', first + 1);
    int third = cmd.indexOf(':', second + 1);
    int fourth = cmd.indexOf(':', third + 1);

    config.steps = cmd.substring(first + 1, second).toInt();
    config.maxSpeed = cmd.substring(second + 1, third).toFloat();
    config.acceleration = cmd.substring(third + 1, fourth).toFloat();
    config.direction = cmd.substring(fourth + 1).toInt();

    stepper.setMaxSpeed(config.maxSpeed);
    stepper.setAcceleration(config.acceleration);
    stepper.setPinsInverted(config.direction == -1, false, false);

    Serial.println("OK: Configuration updated.");
    printConfig();

  } else if (cmd == "R") { // Run with configured steps
    Serial.println("OK: Starting motor for " + String(config.steps) + " steps.");
    digitalWrite(enablePin, LOW); // Enable driver
    delay(10); // Small delay to ensure driver is ready
    stepper.move(config.steps * config.direction);
    motorRunning = true;

  } else if (cmd == "RCON") { // Run continuously
    Serial.println("OK: Starting motor continuously.");
    digitalWrite(enablePin, LOW); // Enable driver
    delay(10);
    // Set a very large number to simulate continuous running
    stepper.move(99999999 * config.direction);
    motorRunning = true;

  } else if (cmd == "S") { // Stop
    Serial.println("OK: Stopping motor.");
    stepper.stop();
    stepper.setCurrentPosition(0);
    motorRunning = false;
    digitalWrite(enablePin, HIGH); // Disable driver
    Serial.println("State: DISABLED");

  } else if (cmd == "P") { // Print config
    printConfig();
  } else if (cmd == "STATUS") {
    Serial.println("OK: Status");
    Serial.println("Running: " + String(motorRunning ? "Yes" : "No"));
    printConfig();
  } else if (cmd == "SAVE") {
    saveConfig();
  } else if (cmd == "LOAD") {
    loadConfig();
    // Re-apply loaded settings
    stepper.setMaxSpeed(config.maxSpeed);
    stepper.setAcceleration(config.acceleration);
    stepper.setPinsInverted(config.direction == -1, false, false);
    Serial.println("OK: Configuration loaded from EEPROM.");
  } else if (cmd == "RESET") {
    resetConfig();
    Serial.println("OK: Configuration reset to defaults.");
  } else {
    Serial.println("ERROR: Unknown command");
  }
}

// --- EEPROM and Config Functions ---
void printConfig() {
  Serial.println("---- Current Settings ----");
  Serial.println("Max Speed: " + String(config.maxSpeed));
  Serial.println("Acceleration: " + String(config.acceleration));
  Serial.println("Steps: " + String(config.steps));
  Serial.println("Direction: " + String(config.direction == 1 ? "Forward" : "Reverse"));
  Serial.println("--------------------------");
}

void saveConfig() {
  config.version = CONFIG_VERSION;
  EEPROM.put(EEPROM_ADDR, config);
  Serial.println("OK: Configuration saved to EEPROM.");
}

void loadConfig() {
  EEPROM.get(EEPROM_ADDR, config);
  if (config.version != CONFIG_VERSION) {
    Serial.println("WARN: No valid config in EEPROM. Loading defaults.");
    resetConfig();
  } else {
    Serial.println("INFO: Loaded configuration from EEPROM.");
  }
}

void resetConfig() {
  config.version = CONFIG_VERSION;
  config.maxSpeed = 4000.0;
  config.acceleration = 2000.0;
  config.steps = 800; // 1 rotation
  config.direction = 1;
  saveConfig(); // Save defaults to EEPROM
} 