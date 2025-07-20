/*
 * Stepper Motor HARDWARE TEST
 *
 * PURPOSE: To verify the physical wiring and power of the stepper motor.
 * This sketch has NO serial commands and NO logic. It tries to spin the
 * motor at a constant speed as soon as the Arduino boots up.
 *
 * INSTRUCTIONS:
 * 1. Disconnect the Arduino from ROS (close any nodes using the serial port).
 * 2. Upload this sketch.
 * 3. The motor should start spinning immediately after the upload completes.
 *
 * IF THE MOTOR DOES NOT SPIN WITH THIS SKETCH:
 * The problem is 100% in your hardware (wiring or power supply).
 * Check the hardware checklist below.
 */

#include <AccelStepper.h>

// Define the connections
const int stepPin = 2;
const int dirPin = 3;
const int enablePin = 8;

// Create the stepper instance
// AccelStepper::DRIVER uses 1 pin for step and 1 pin for direction.
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup()
{
  // Set the enable pin. setPinsInverted is used because the
  // A4988 enable pin is active-low. The third parameter 'true' inverts the enable pin logic.
  stepper.setEnablePin(enablePin);
  stepper.setPinsInverted(false, false, true);

  // Set a moderate speed and acceleration.
  // Let's start slow to ensure it works. 800 steps/sec is 1 rotation/sec.
  stepper.setMaxSpeed(800);
  stepper.setAcceleration(400);

  // Enable the motor driver.
  stepper.enableOutputs();

  // Tell the motor to move to a position very far away.
  // This will make it run "continuously".
  stepper.move(99999999);
}

void loop()
{
  // This is the most important function. It must be called as frequently
  // as possible. It checks if a step is due and sends the pulse.
  // We use run() here because we set acceleration. runSpeed() ignores acceleration.
  stepper.run();
} 