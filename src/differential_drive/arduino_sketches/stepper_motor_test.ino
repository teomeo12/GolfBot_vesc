#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// --- PIN DEFINITIONS ---
// These must match your physical wiring from the Arduino to the A4988 driver
#define STEPPER_STEP_PIN    2 // Connect to STEP on A4988
#define STEPPER_DIR_PIN     3 // Connect to DIR on A4988
#define STEPPER_ENABLE_PIN  4 // Connect to ENABLE on A4988

// --- CONFIGURATION ---
const float MOTOR_SPEED = 800.0; // Steps per second. Adjust as needed. Start slow.

// --- GLOBAL OBJECTS ---
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// --- GLOBAL STATE ---
// We don't need the motor_is_running flag for this test
unsigned long lastImuUpdate = 0;
const int IMU_UPDATE_INTERVAL = 100; // ms, for 10Hz update rate

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect.
  Serial.println("--- Arduino Stepper & IMU Test Sketch (DEBUG VERSION) ---");

  // --- Stepper Setup ---
  stepper.setEnablePin(STEPPER_ENABLE_PIN);
  stepper.setPinsInverted(false, false, true); // The A4988 ENABLE pin is active-low.
  stepper.setMaxSpeed(MOTOR_SPEED * 1.2); // Set max speed slightly higher than target speed
  // We will enable and set speed on command now.

  Serial.println("Stepper Initialized. Send 'R' to run, 'S' to stop.");

  // --- IMU Setup ---
  if (!bno.begin()) {
    Serial.println("ERROR: No BNO055 IMU detected! Check wiring.");
    // We don't want to halt everything, so we'll just print an error.
  } else {
    delay(100);
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 IMU Initialized.");
  }
}

void loop() {
  // 1. Check for incoming commands from ROS
  if (Serial.available() > 0) {
    char command = Serial.read();

    // Trim leading whitespace
    while (isspace(command) && Serial.available() > 0) {
        command = Serial.read();
    }

    if (command == 'R' || command == 'r') {
      Serial.println("CMD: 'R' received. Entering blocking motor run loop.");
      
      // Forcefully enable and set speed
      stepper.enableOutputs(); 
      stepper.setSpeed(MOTOR_SPEED);

      // This is a BLOCKING loop for debugging.
      // It will run the motor and ignore IMU updates until an 'S' is received.
      while (true) {
        stepper.runSpeed(); // Continuously step the motor.

        // Check for a stop command without blocking the motor
        if (Serial.available() > 0) {
          char stop_command = Serial.read();
          if (stop_command == 'S' || stop_command == 's') {
            Serial.println("CMD: 'S' received. Stopping motor.");
            stepper.stop(); // Use stop() to decelerate to zero.
            stepper.disableOutputs(); 
            break; // Exit this while loop
          }
        }
      }
      Serial.println("Exited motor run loop. Resuming normal operation.");
    }
  }

  // 2. Periodically send IMU data back to ROS
  // Note: This will PAUSE while the motor is in its run loop above.
  if (millis() - lastImuUpdate >= IMU_UPDATE_INTERVAL) {
    lastImuUpdate = millis();
    publishImuData();
  }
}

void publishImuData() {
  sensors_event_t event;
  bno.getEvent(&event);
  
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  // Using the same 'I:' prefix as the other ROS node expects
  // The ROS node will publish this to the /imu/data topic, so we can comment it out
  // here to keep the Serial Monitor clean for debugging motor commands.
  /*
  Serial.print("I:");
  // Orientation (as Euler angles for simplicity in debugging, though Quaternion is better)
  Serial.print(event.orientation.x, 2); Serial.print(":"); // Heading
  Serial.print(event.orientation.y, 2); Serial.print(":"); // Roll
  Serial.print(event.orientation.z, 2); Serial.print(":"); // Pitch
  // Linear Acceleration
  Serial.print(linear_accel.x(), 4); Serial.print(":");
  Serial.print(linear_accel.y(), 4); Serial.print(":");
  Serial.print(linear_accel.z(), 4); Serial.print(":");
  // Angular Velocity
  Serial.print(gyro.x(), 4); Serial.print(":");
  Serial.print(gyro.y(), 4); Serial.print(":");
  Serial.print(gyro.z(), 4);
  Serial.println(); // End of message
  */
} 