/*
 * =============================================================
 * == CONFIGURABLE Stepper + IMU Sketch ==
 * =============================================================
 * PURPOSE: Adds dynamic configuration via ROS to the simple, working sketch.
 *
 * FEATURES:
 * - Listens for a 'C' command on startup from ROS to set parameters.
 * - Allows changing speed/acceleration from the YAML file.
 * - Keeps the simple, direct 'R' (Run) and 'S' (Stop) commands.
 * - Includes the required IMU data stream.
 */

#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// --- Pin Definitions ---
const int stepPin = 2;
const int dirPin = 3;
const int enablePin = 8;

// --- Object Instances ---
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// --- Configuration Struct ---
// Holds the motor parameters. These will be updated by ROS.
struct StepperConfig {
    float maxSpeed;
    float acceleration;
    long steps;
    int direction;
    bool continuous;
};

// Default values, in case ROS doesn't send a config.
StepperConfig config = {
    .maxSpeed = 2000.0,
    .acceleration = 2000.0,
    .steps = 800,
    .direction = 1,
    .continuous = true
};

// --- IMU Timing ---
unsigned long lastImuUpdate = 0;
const int IMU_UPDATE_INTERVAL = 100; // 10Hz

void setup() {
    Serial.begin(115200);
    Serial.println("\n===== CONFIGURABLE Stepper + IMU Test =====");

    // --- Stepper Setup ---
    stepper.setEnablePin(enablePin);
    stepper.setPinsInverted(false, false, true);
    applyStepperConfig(); // Apply the default config initially
    stepper.disableOutputs();
    Serial.println("Stepper waiting for configuration from ROS...");

    // --- IMU Setup ---
    if (!bno.begin()) {
        Serial.println("WARNING: BNO055 not detected.");
    } else {
        bno.setExtCrystalUse(true);
        Serial.println("BNO055 IMU detected.");
    }
    Serial.println("Ready for commands.");
}

void loop() {
    stepper.run();

    if (Serial.available() > 0) {
        char cmd = Serial.read();

        if (cmd == 'R' || cmd == 'r') {
            Serial.println("CMD: 'R' received. Starting motor.");
            stepper.enableOutputs();
            if (config.continuous) {
                stepper.move(99999999L * config.direction);
            } else {
                stepper.move(config.steps * config.direction);
            }
        }

        if (cmd == 'S' || cmd == 's') {
            Serial.println("CMD: 'S' received. Stopping motor.");
            stepper.stop();
            stepper.setCurrentPosition(0);
            stepper.disableOutputs();
        }

        if (cmd == 'C' || cmd == 'c') {
            parseConfig();
        }
    }

    if (millis() - lastImuUpdate >= IMU_UPDATE_INTERVAL) {
        sendImuData();
        lastImuUpdate = millis();
    }
}

// Applies the current values from the 'config' struct to the stepper motor
void applyStepperConfig() {
    stepper.setMaxSpeed(config.maxSpeed);
    stepper.setAcceleration(config.acceleration);
}

// Parses the configuration string from ROS (e.g., "C:steps:speed:accel:dir:cont")
void parseConfig() {
    String data = Serial.readStringUntil('\n');
    StepperConfig newConfig = config; // Start with current config as a base
    int lastIndex = 1; // Start parsing after the initial ':'
    int part = 0;

    for (int i = 1; i < data.length(); i++) {
        if (data.charAt(i) == ':') {
            String value = data.substring(lastIndex, i);
            switch (part) {
                case 0: newConfig.steps = value.toInt(); break;
                case 1: newConfig.maxSpeed = value.toFloat(); break;
                case 2: newConfig.acceleration = value.toFloat(); break;
                case 3: newConfig.direction = value.toInt(); break;
            }
            lastIndex = i + 1;
            part++;
        }
    }
    // Handle the last part (the continuous flag)
    String value = data.substring(lastIndex);
    newConfig.continuous = (value.toInt() == 1);

    config = newConfig; // Save the newly parsed config
    applyStepperConfig(); // Apply the settings to the motor

    Serial.print("OK - Configured with Speed: ");
    Serial.print(config.maxSpeed);
    Serial.print(", Accel: ");
    Serial.println(config.acceleration);
}

void sendImuData() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    uint8_t cal_system, cal_gyro, cal_accel, cal_mag;
    bno.getCalibration(&cal_system, &cal_gyro, &cal_accel, &cal_mag);

    Serial.print("I:");
    Serial.print(euler.x()); Serial.print(":");
    Serial.print(euler.y()); Serial.print(":");
    Serial.print(euler.z()); Serial.print(":");
    Serial.print(linear_accel.x()); Serial.print(":");
    Serial.print(linear_accel.y()); Serial.print(":");
    Serial.print(linear_accel.z()); Serial.print(":");
    Serial.print(gyro.x()); Serial.print(":");
    Serial.print(gyro.y()); Serial.print(":");
    Serial.print(gyro.z()); Serial.print(":");
    Serial.print(cal_system); Serial.print(":");
    Serial.print(cal_gyro); Serial.print(":");
    Serial.print(cal_accel); Serial.print(":");
    Serial.println(cal_mag);
} 