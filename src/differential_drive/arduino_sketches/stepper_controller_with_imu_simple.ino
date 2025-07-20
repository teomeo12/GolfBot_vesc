/*
 * =============================================================
 * == BAREBONES Stepper + IMU Test Sketch ==
 * =============================================================
 * PURPOSE: A simplified, robust test based on user feedback.
 *
 * FEATURES:
 * - Uses hardcoded speed/acceleration that is known to work.
 * - NO configuration commands ('C').
 * - NO complex state-tracking variables.
 * - Simple, direct 'R' (Run) and 'S' (Stop) commands.
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

// --- IMU Timing ---
unsigned long lastImuUpdate = 0;
const int IMU_UPDATE_INTERVAL = 100; // Send IMU data at 10Hz

void setup() {
    Serial.begin(115200);
    Serial.println("\n===== BAREBONES Stepper + IMU Test =====");

    // --- Stepper Setup ---
    stepper.setEnablePin(enablePin);
    stepper.setPinsInverted(false, false, true); // Invert enable pin for A4988

    // Use the speed/accel you said works well. These are now hardcoded.
    stepper.setMaxSpeed(2000);
    stepper.setAcceleration(2000);

    stepper.disableOutputs(); // Start with motor disabled.
    Serial.println("Stepper configured with hardcoded speed (2000).");

    // --- IMU Setup ---
    if (!bno.begin()) {
        Serial.println("WARNING: BNO055 not detected. Continuing without IMU data.");
    } else {
        bno.setExtCrystalUse(true);
        Serial.println("BNO055 IMU detected.");
    }
    Serial.println("Ready for 'R' (Run) or 'S' (Stop) commands.");
}

void loop() {
    // THIS IS THE MOST IMPORTANT LINE. It MUST always run to move the motor.
    stepper.run();

    // --- Check for commands from ROS ---
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        // Simple 'R' command to run
        if (cmd == 'R' || cmd == 'r') {
            Serial.println("CMD: 'R' received. Starting motor.");
            stepper.enableOutputs();
            stepper.move(99999999); // Move to a very large number to run continuously
        }

        // Simple 'S' command to stop
        if (cmd == 'S' || cmd == 's') {
            Serial.println("CMD: 'S' received. Stopping motor.");
            stepper.stop(); // This decelerates to a stop based on acceleration
            stepper.setCurrentPosition(0); // Reset position counter
            stepper.disableOutputs(); // Disable driver to save power
        }
    }

    // --- Periodically send IMU data ---
    if (millis() - lastImuUpdate >= IMU_UPDATE_INTERVAL) {
        sendImuData();
        lastImuUpdate = millis();
    }
}

// This function just sends the IMU data over serial.
// It matches the 13-part format the ROS node expects.
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