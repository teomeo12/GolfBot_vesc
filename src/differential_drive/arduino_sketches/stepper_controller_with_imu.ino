/*
 * GolfBot Enhanced Controller with IMU
 * 
 * This sketch combines:
 * 1. Stepper motor control (A4988 driver)
 * 2. BNO055 IMU sensor readings
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
Adafruit_BNO055 bno = Adafruit_BNO055(55);

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
    .continuous = false   // default to non-continuous
};

// IMU data struct
struct ImuData {
    float orientation[3];  // euler angles
    float acceleration[3]; // linear acceleration
    float gyro[3];        // angular velocity
    uint8_t cal_system;   // system calibration (0-3)
    uint8_t cal_gyro;     // gyro calibration (0-3)
    uint8_t cal_accel;    // accelerometer calibration (0-3)
    uint8_t cal_mag;      // magnetometer calibration (0-3)
};

ImuData imuData;

// Timing control
unsigned long lastImuUpdate = 0;
const int IMU_UPDATE_INTERVAL = 50;  // 20Hz IMU updates

void setup() {
    Serial.begin(115200);
    
    // Initialize stepper
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);  // Disable stepper initially
    stepper.setEnablePin(enablePin);
    stepper.setPinsInverted(false, false, true); // Enable pin is inverted
    applyStepperConfig();
    
    // Initialize IMU
    if (!bno.begin()) {
        Serial.println("No BNO055 detected!");
        while (1);
    }
    
    bno.setExtCrystalUse(true);
    
    Serial.println("==== GolfBot Controller with IMU Ready ====");
    Serial.println("Commands:");
    Serial.println("  R - Run stepper with current settings");
    Serial.println("  S - Stop stepper");
    Serial.println("  C:steps:speed:accel:dir - Configure stepper");
    Serial.println("  E:0/1 - Enable/Disable stepper");
    Serial.println("  I - Request IMU data");
}

void loop() {
    // ALWAYS call stepper.run(). It's the heart of the AccelStepper library.
    // It will check if the motor needs to move and step it if it does.
    stepper.run();
    
    // Update and send IMU data periodically
    if (millis() - lastImuUpdate >= IMU_UPDATE_INTERVAL) {
        updateImuData();
        sendImuData(); // <-- ADDED THIS LINE
        lastImuUpdate = millis();
    }
    
    // Check for commands from ROS
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'R':
                runStepper();
                break;
            case 'S':
                stopStepper();
                break;
            case 'C':
                parseConfig();
                break;
            case 'E':
                handleEnable();
                break;
            // The 'I' case is no longer needed as data streams automatically
        }
    }
}

void updateImuData() {
    // Get Euler angles (in degrees)
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imuData.orientation[0] = euler.x();
    imuData.orientation[1] = euler.y();
    imuData.orientation[2] = euler.z();
    
    // Get linear acceleration (in m/s^2)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imuData.acceleration[0] = accel.x();
    imuData.acceleration[1] = accel.y();
    imuData.acceleration[2] = accel.z();
    
    // Get angular velocity (in rad/s)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imuData.gyro[0] = gyro.x();
    imuData.gyro[1] = gyro.y();
    imuData.gyro[2] = gyro.z();
    
    // Get calibration status
    bno.getCalibration(&imuData.cal_system, 
                       &imuData.cal_gyro,
                       &imuData.cal_accel, 
                       &imuData.cal_mag);
}

void sendImuData() {
    // Format: I:heading:roll:pitch:ax:ay:az:gx:gy:gz:cal_sys:cal_gyr:cal_acc:cal_mag
    Serial.print("I:");
    // Orientation
    for (int i = 0; i < 3; i++) {
        Serial.print(imuData.orientation[i]);
        Serial.print(":");
    }
    // Acceleration
    for (int i = 0; i < 3; i++) {
        Serial.print(imuData.acceleration[i]);
        Serial.print(":");
    }
    // Gyro
    for (int i = 0; i < 3; i++) {
        Serial.print(imuData.gyro[i]);
        Serial.print(":");
    }
    // Calibration
    Serial.print(imuData.cal_system);
    Serial.print(":");
    Serial.print(imuData.cal_gyro);
    Serial.print(":");
    Serial.print(imuData.cal_accel);
    Serial.print(":");
    Serial.println(imuData.cal_mag);
}

void runStepper() {
    Serial.println("CMD: R received by Arduino!");
    stepper.enableOutputs(); // Explicitly enable the motor
    if (config.continuous) {
        stepper.move(99999999L * config.direction);  // Very large number for continuous motion
    } else {
        stepper.move(config.steps * config.direction);
    }
    Serial.println("OK");
}

void stopStepper() {
    Serial.println("CMD: S received by Arduino!");
    stepper.stop();
    stepper.setCurrentPosition(0);
    stepper.disableOutputs(); // Disable motor when stopped to save power and reduce heat
    Serial.println("OK");
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
    applyStepperConfig();
    Serial.println("OK");
}

void applyStepperConfig() {
    stepper.setMaxSpeed(config.maxSpeed);
    stepper.setAcceleration(config.acceleration);
    stepper.setPinsInverted(config.direction == -1, false, false);
} 