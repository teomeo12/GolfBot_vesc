/*
 * GolfBot Enhanced Controller with IMU - FIXED VERSION
 * 
 * This sketch combines:
 * 1. Stepper motor control (A4988 driver) - FIXED LOGIC
 * 2. BNO055 IMU sensor readings - FIXED FORMAT
 * 
 * Connections:
 * Stepper Driver:
 * - STEP -> D2
 * - DIR  -> D3
 * - EN   -> D4
 * 
 * BNO055:
 * - SDA -> A4
 * - SCL -> A5
 * - VIN -> 5V
 * - GND -> GND
 */

#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Pin Definitions
const int stepPin = 2;
const int dirPin = 3;
const int enablePin = 4;

// Create instances
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Configuration struct
struct StepperConfig {
    float maxSpeed;      // steps per second
    float acceleration;  // steps per second per second
    long steps;         // number of steps to move
    int direction;      // 1 or -1
    bool continuous;    // run continuously or not
};

StepperConfig config = {
    .maxSpeed = 800,      // default speed
    .acceleration = 400,  // default acceleration
    .steps = 200,        // default steps (1 revolution for most steppers)
    .direction = 1,      // default direction
    .continuous = true   // CHANGED: default to continuous for joystick control
};

// Motor state
bool motorRunning = false;

// Timing control
unsigned long lastImuUpdate = 0;
const int IMU_UPDATE_INTERVAL = 100;  // 10Hz IMU updates

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("==== GolfBot Controller with IMU - FIXED VERSION ====");
    
    // Initialize stepper with corrected logic
    stepper.setEnablePin(enablePin);
    stepper.setPinsInverted(false, false, true); // Enable pin is active-low
    stepper.setMaxSpeed(config.maxSpeed);
    stepper.setAcceleration(config.acceleration);
    stepper.disableOutputs(); // Start disabled
    
    Serial.println("Stepper initialized");
    
    // Initialize IMU
    if (!bno.begin()) {
        Serial.println("ERROR: No BNO055 detected! Check wiring.");
        // Don't halt, continue without IMU
    } else {
        delay(100);
        bno.setExtCrystalUse(true);
        Serial.println("BNO055 IMU initialized");
    }
    
    Serial.println("Ready! Send 'R' to run, 'S' to stop.");
}

void loop() {
    // Handle motor movement based on mode
    if (motorRunning) {
        if (config.continuous) {
            // For continuous movement, call runSpeed()
            stepper.runSpeed();
        } else {
            // For step movement, call run()
            stepper.run();
            // Check if movement is complete
            if (stepper.distanceToGo() == 0) {
                motorRunning = false;
                stepper.disableOutputs();
                Serial.println("Step movement completed");
            }
        }
    }
    
    // Update and send IMU data periodically
    if (millis() - lastImuUpdate >= IMU_UPDATE_INTERVAL) {
        sendImuData();
        lastImuUpdate = millis();
    }
    
    // Check for commands from ROS
    if (Serial.available()) {
        char cmd = Serial.read();
        
        // Clear any trailing whitespace
        while (Serial.available() && isspace(Serial.peek())) {
            Serial.read();
        }
        
        switch (cmd) {
            case 'R':
            case 'r':
                runStepper();
                break;
            case 'S':
            case 's':
                stopStepper();
                break;
            case 'C':
                parseConfig();
                break;
            case 'E':
                handleEnable();
                break;
            default:
                // Ignore unknown commands
                break;
        }
    }
}

void runStepper() {
    Serial.println("CMD: R received by Arduino!");
    
    if (!motorRunning) {
        stepper.enableOutputs();
        
        if (config.continuous) {
            // For continuous movement, use setSpeed and runSpeed
            stepper.setSpeed(config.maxSpeed * config.direction);
            motorRunning = true;
            Serial.println("Motor started - continuous mode");
        } else {
            // For step-by-step movement, use move
            stepper.move(config.steps * config.direction);
            motorRunning = true;
            Serial.println("Motor started - step mode");
        }
    }
    
    Serial.println("OK");
}

void stopStepper() {
    Serial.println("CMD: S received by Arduino!");
    
    if (motorRunning) {
        if (config.continuous) {
            stepper.setSpeed(0);
        } else {
            stepper.stop();
        }
        stepper.setCurrentPosition(0);
        stepper.disableOutputs();
        motorRunning = false;
        Serial.println("Motor stopped");
    }
    
    Serial.println("OK");
}

void sendImuData() {
    // Get sensor data
    sensors_event_t event;
    bno.getEvent(&event);
    
    imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    
    // Format EXACTLY as expected by ROS node: I:heading:roll:pitch:ax:ay:az:gx:gy:gz
    // Only 13 fields total (including the 'I:' prefix)
    Serial.print("I:");
    
    // Orientation (Euler angles in degrees)
    Serial.print(event.orientation.x, 2); Serial.print(":");  // Heading
    Serial.print(event.orientation.y, 2); Serial.print(":");  // Roll  
    Serial.print(event.orientation.z, 2); Serial.print(":");  // Pitch
    
    // Linear Acceleration (m/s²)
    Serial.print(linear_accel.x(), 4); Serial.print(":");
    Serial.print(linear_accel.y(), 4); Serial.print(":");
    Serial.print(linear_accel.z(), 4); Serial.print(":");
    
    // Angular Velocity (rad/s)
    Serial.print(gyro.x(), 4); Serial.print(":");
    Serial.print(gyro.y(), 4); Serial.print(":");
    Serial.print(gyro.z(), 4);
    
    Serial.println(); // End of message
}

void handleEnable() {
    while (!Serial.available()) {}  // Wait for parameter
    char state = Serial.read();
    if (state == '1') {
        stepper.enableOutputs();
        Serial.println("Stepper Enabled");
    } else if (state == '0') {
        stepper.disableOutputs();
        Serial.println("Stepper Disabled");
    }
}

void parseConfig() {
    String data = Serial.readStringUntil('\n');
    int idx = 0;
    int lastIndex = 0;
    StepperConfig newConfig = config;  // Start with current config
    
    // Parse each value
    for (int i = 0; i < data.length(); i++) {
        if (data.charAt(i) == ':') {
            String value = data.substring(lastIndex, i);
            switch (idx) {
                case 0:
                    newConfig.steps = value.toInt();
                    break;
                case 1:
                    newConfig.maxSpeed = value.toFloat();
                    break;
                case 2:
                    newConfig.acceleration = value.toFloat();
                    break;
                case 3:
                    newConfig.direction = value.toInt();
                    break;
                case 4:
                    newConfig.continuous = (value.toInt() == 1);
                    break;
            }
            lastIndex = i + 1;
            idx++;
        }
    }
    
    // Apply the new configuration
    config = newConfig;
    stepper.setMaxSpeed(config.maxSpeed);
    stepper.setAcceleration(config.acceleration);
    Serial.println("Configuration updated");
    Serial.println("OK");
} 