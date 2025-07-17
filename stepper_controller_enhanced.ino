#include <AccelStepper.h>
#include <EEPROM.h>

// Stepper motor setup - pins 2 and 3 as in your original code
AccelStepper stepper(AccelStepper::DRIVER, 2, 3);  // StepPin D2 and DirPin D3

// Configuration structure for EEPROM storage
struct StepperConfig {
  int steps_per_command;    // Number of steps to run per 'R' command
  float max_speed;          // Maximum speed
  float acceleration;       // Acceleration
  int direction;            // 1 for forward, -1 for reverse
  bool auto_return;         // Whether to return to original position after steps
  char signature[4];        // "STP" signature to validate EEPROM data
};

// Default configuration
StepperConfig config = {
  800,      // steps_per_command (one full rotation = 800 steps for your setup)
  4000.0,   // max_speed
  2000.0,   // acceleration 
  1,        // direction (forward)
  false,    // auto_return
  {'S', 'T', 'P', '\0'}  // signature
};

// Runtime variables
bool is_running = false;
bool continuous_mode = false;  // NEW: Flag for continuous running
long target_position = 0;
long start_position = 0;
String input_buffer = "";
bool command_ready = false;

// EEPROM address for storing configuration
const int EEPROM_ADDRESS = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Enhanced Stepper Controller v1.0");
  Serial.println("Commands:");
  Serial.println("  R - Run stepper with current settings");
  Serial.println("  RCON - Run continuously until stopped");
  Serial.println("  S - Stop stepper");
  Serial.println("  C:steps:speed:accel:dir - Configure (e.g., C:1600:4000:2000:1)");
  Serial.println("  P - Print current settings");
  Serial.println("  SAVE - Save settings to EEPROM");
  Serial.println("  LOAD - Load settings from EEPROM");
  Serial.println("  RESET - Reset to factory defaults");
  Serial.println("  STATUS - Get current status");
  Serial.println("Ready!");
  
  // Try to load configuration from EEPROM
  loadConfiguration();
  
  // Apply loaded configuration
  applyStepperConfig();
  
  printConfiguration();
}

void loop() {
  // Handle serial communication
  handleSerial();
  
  // Run stepper motor
  if (is_running) {
    stepper.run();
    
    // Check if we've reached the target position
    if (stepper.distanceToGo() == 0) {
      if (continuous_mode) {
        // In continuous mode, start a new movement in the same direction
        start_position = stepper.currentPosition();
        target_position = start_position + (config.steps_per_command * config.direction);
        stepper.moveTo(target_position);
        // Don't set is_running = false, keep running continuously
      } else {
        // Normal mode - stop after reaching target
        is_running = false;
        Serial.println("DONE");
        
        // If auto-return is enabled, go back to start position
        if (config.auto_return && stepper.currentPosition() != start_position) {
          Serial.println("AUTO_RETURN");
          stepper.moveTo(start_position);
          is_running = true;
        }
      }
    }
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (input_buffer.length() > 0) {
        processCommand(input_buffer);
        input_buffer = "";
      }
    } else {
      input_buffer += c;
    }
  }
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  Serial.print("CMD: ");
  Serial.println(command);
  
  if (command == "R") {
    runStepper();
  }
  else if (command == "RCON") {
    continuous_mode = true;
    runStepper(); // Run the first step
    Serial.println("RCON_STARTED");
  }
  else if (command == "S") {
    stopStepper();
  }
  else if (command.startsWith("C:")) {
    parseConfigCommand(command);
  }
  else if (command == "P") {
    printConfiguration();
  }
  else if (command == "SAVE") {
    saveConfiguration();
  }
  else if (command == "LOAD") {
    loadConfiguration();
    applyStepperConfig();
  }
  else if (command == "RESET") {
    resetToDefaults();
  }
  else if (command == "STATUS") {
    printStatus();
  }
  else {
    Serial.println("ERROR: Unknown command");
  }
}

void runStepper() {
  if (is_running) {
    Serial.println("ERROR: Already running");
    return;
  }
  
  start_position = stepper.currentPosition();
  target_position = start_position + (config.steps_per_command * config.direction);
  
  stepper.moveTo(target_position);
  is_running = true;
  
  Serial.print("RUNNING: ");
  Serial.print(config.steps_per_command);
  Serial.print(" steps from ");
  Serial.print(start_position);
  Serial.print(" to ");
  Serial.println(target_position);
}

void stopStepper() {
  stepper.stop();
  is_running = false;
  continuous_mode = false; // Stop continuous mode
  Serial.println("STOPPED");
}

void parseConfigCommand(String command) {
  // Format: C:steps:speed:accel:direction
  // Example: C:1600:4000:2000:1
  
  int firstColon = command.indexOf(':', 2);
  int secondColon = command.indexOf(':', firstColon + 1);
  int thirdColon = command.indexOf(':', secondColon + 1);
  int fourthColon = command.indexOf(':', thirdColon + 1);
  
  if (firstColon == -1 || secondColon == -1 || thirdColon == -1) {
    Serial.println("ERROR: Invalid config format. Use C:steps:speed:accel:dir");
    return;
  }
  
  int steps = command.substring(2, firstColon).toInt();
  float speed = command.substring(firstColon + 1, secondColon).toFloat();
  float accel = command.substring(secondColon + 1, thirdColon).toFloat();
  int dir = 1;
  
  if (fourthColon != -1) {
    dir = command.substring(thirdColon + 1, fourthColon).toInt();
  } else {
    dir = command.substring(thirdColon + 1).toInt();
  }
  
  // Validate parameters
  if (steps <= 0 || speed <= 0 || accel <= 0 || abs(dir) != 1) {
    Serial.println("ERROR: Invalid parameters");
    return;
  }
  
  // Update configuration
  config.steps_per_command = steps;
  config.max_speed = speed;
  config.acceleration = accel;
  config.direction = dir;
  
  // Apply new configuration
  applyStepperConfig();
  
  Serial.println("CONFIG_UPDATED");
  printConfiguration();
}

void applyStepperConfig() {
  stepper.setMaxSpeed(config.max_speed);
  stepper.setAcceleration(config.acceleration);
}

void printConfiguration() {
  Serial.println("=== CONFIGURATION ===");
  Serial.print("Steps per command: ");
  Serial.println(config.steps_per_command);
  Serial.print("Max speed: ");
  Serial.println(config.max_speed);
  Serial.print("Acceleration: ");
  Serial.println(config.acceleration);
  Serial.print("Direction: ");
  Serial.println(config.direction == 1 ? "Forward" : "Reverse");
  Serial.print("Auto return: ");
  Serial.println(config.auto_return ? "Enabled" : "Disabled");
  Serial.println("==================");
}

void printStatus() {
  Serial.println("=== STATUS ===");
  Serial.print("Running: ");
  Serial.println(is_running ? "Yes" : "No");
  Serial.print("Current position: ");
  Serial.println(stepper.currentPosition());
  Serial.print("Target position: ");
  Serial.println(target_position);
  Serial.print("Distance to go: ");
  Serial.println(stepper.distanceToGo());
  Serial.print("Current speed: ");
  Serial.println(stepper.speed());
  Serial.println("==============");
}

void saveConfiguration() {
  EEPROM.put(EEPROM_ADDRESS, config);
  Serial.println("SAVED");
}

void loadConfiguration() {
  StepperConfig loaded_config;
  EEPROM.get(EEPROM_ADDRESS, loaded_config);
  
  // Check if EEPROM contains valid data
  if (loaded_config.signature[0] == 'S' && 
      loaded_config.signature[1] == 'T' && 
      loaded_config.signature[2] == 'P') {
    config = loaded_config;
    Serial.println("LOADED from EEPROM");
  } else {
    Serial.println("EEPROM invalid, using defaults");
  }
}

void resetToDefaults() {
  config = {
    800,      // steps_per_command
    4000.0,   // max_speed  
    2000.0,   // acceleration
    1,        // direction
    false,    // auto_return
    {'S', 'T', 'P', '\0'}
  };
  
  applyStepperConfig();
  Serial.println("RESET to defaults");
  printConfiguration();
} 