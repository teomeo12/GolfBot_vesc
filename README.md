# GolfBot Project: Version 2 README (Current Implementation)

## 1. Project Overview

This document outlines the **current, implemented system architecture** and development plan for the **GolfBot**, an autonomous rover for divot detection and repair.

The project's primary goal is to **autonomously detect, localize, and repair divots**.

The system is built upon three core components:
*   **Computer Vision:** To detect and segment divots from a live video stream.
*   **RTK-Enhanced Navigation:** To provide high-precision global and local positioning for accurate navigation and state estimation.
*   **Mechanical Dispensing:** To deposit a controlled amount of sand/seed mixture to repair the detected divot.

The GolfBot operates in two distinct modes:
1.  **Manual Mode:** Full teleoperation control via a joystick for transport, testing, and manual intervention.
2.  **Autonomous Mode:** A fully automated "search-and-repair" sequence for unattended operation within a defined area.

---

## 2. Hardware Architecture

The system integrates the following hardware components:

*   **Controller:** Seeed Studio J4102 Carrier Board with an **NVIDIA Jetson Orin Nano 16GB** module.
*   **Motor Control:** 2x **VESC Flipsky** motor controllers, connected via USB Serial, for differential drive control.
*   **Positioning:** **ArduSimple simpleRTK2B Kit** (Rover module), providing RTK-corrected GPS data via USB.
*   **Dispenser & IMU:** An **Arduino Uno R3** board connected to:
    *   A **BNO055 IMU** for supplementary orientation data.
    *   An **A4988 stepper motor driver** to control the dispensing mechanism.
    *   Connected via USB Serial.
*   **Vision:** An **Intel RealSense D435i** depth camera, providing color, depth, and internal IMU data via USB.
*   **Manual Control:** A **Logitech F710 Joystick** connected via its wireless USB dongle.

---

## 3. Current ROS 2 Software Architecture (As Implemented)

This section details the system based on the nodes and topics found in the `differential_drive` package.

### Mode Arbitration

*   Currently, the `logitech_gamepad_node.py` provides manual control by publishing directly to `/cmd_vel`. The `stepper_imu_node.py` handles motor commands based on joystick input.
*   **The final mode arbitration logic (toggling `/is_autonomous_mode`) has not been implemented yet.**

### Component Breakdown (Current Implementation)

#### Component 1: Navigation & Control

*   **`logitech_gamepad_node.py` (Manual Control)**
    *   **Filename:** `src/differential_drive/differential_drive/logitech_gamepad.py`
    *   **Subscribes:** `/joy`
    *   **Publishes:** `/cmd_vel`
    *   **Function:** Translates joystick axes movements into `Twist` messages for robot motion.

*   **`ublox_gps_node` (External Package)**
    *   **Publishes:** `/fix`
    *   **Function:** Interfaces with the ArduSimple Rover to provide global position.

*   **`gps_monitor_node.py` (RTK Status Monitor)**
    *   **Filename:** `src/differential_drive/differential_drive/gps_monitor_node.py`
    *   **Subscribes:** `/fix`
    *   **Function:** A utility node to monitor and print the status of the RTK GPS fix.

*   **`stepper_imu_node.py` (IMU & Stepper Interface)**
    *   **Filename:** `src/differential_drive/differential_drive/stepper_imu_node.py`
    *   **Subscribes:** `/joy` (for manual start/stop commands)
    *   **Publishes:** `/imu/data`, `/stepper/status`
    *   **Function:** Reads IMU data from the Arduino and publishes it. Listens for joystick buttons to send simple 'R' (Run) and 'S' (Stop) serial commands to the Arduino for the dispenser.

#### Component 2: Computer Vision

*   **`camera_node.py` (Camera Driver)**
    *   **Filename:** `src/differential_drive/differential_drive/camera_node.py`
    *   **Publishes:** `/camera/color/image_raw`, `/camera/depth/image_raw`
    *   **Function:** Manually initializes the RealSense D435i camera using the `pyrealsense2` library and streams its color and depth images to ROS 2 topics.

*   **`divot_detection_intel.py` (YOLO Detection Node)**
    *   **Filename:** `src/differential_drive/differential_drive/divot_detection_intel.py` (Launched as `divot_detector`)
    *   **Subscribes:** `/camera/color/image_raw`
    *   **Publishes:** `/camera/divot_detection/image_raw` (annotated image), `/camera/divot_detection/details` (string with detection info)
    *   **Function:** Loads a YOLO model and runs inference on images from `camera_node.py`.

#### Component 3: Mechanical Dispensing System

*   **`stepper_imu_node.py` (Stepper part)**
    *   **Filename:** `src/differential_drive/differential_drive/stepper_imu_node.py`
    *   **Subscribes:** `/joy` (listens for 'A' and 'B' buttons)
    *   **Function:** Sends a single-character command ('R' or 'S') to the Arduino via serial to start or stop the dispenser motor. The logic for dispensing a specific *amount* is not yet implemented.

---

## 4. Autonomous Workflow (Final Goal)

The complete, end-to-end autonomous operation will be as follows:
1.  **Initialization:** The robot is placed at a starting point, and all nodes are launched. The `robot_localization` node converges to provide an accurate state estimate.
2.  **Autonomous Start:** The operator presses the "Y" button to engage autonomous mode.
3.  **Searching:** The `mission_control_node` begins executing a pre-defined search pattern (e.g., a lawnmower pattern) within a defined operational area, using the filtered odometry for navigation.
4.  **Detection:** The `divot_detection_node` continuously scans the camera feed. Upon finding a divot with high confidence, it publishes the detection data.
5.  **Approach & Align:** The `mission_control_node` transitions to the "APPROACHING" state. It uses the detection data and its current position to navigate and align the dispenser mechanism directly over the divot.
6.  **Repair:** Once aligned, the node transitions to the "REPAIRING" state, commands the dispenser to release the repair material, and waits for completion.
7.  **Resume:** The node transitions back to the "SEARCHING" state and resumes the search pattern from where it left off.

---

## 5. Phased Development Roadmap & Implementation Plan

This project will be developed in manageable milestones to ensure steady progress and robust testing.

### **Milestone 1: Manual Control & Hardware Verification**

*   **Goal:** Ensure all hardware components can be individually controlled and monitored from ROS 2. This validates wiring, drivers, and basic communication.
*   **Tasks:**
    1.  The `logitech_gamepad_node` can successfully drive the robot via `/cmd_vel`.
    2.  A joystick button can publish a test message to `/dispense_sand` to manually trigger the dispenser motor.
    3.  All sensor topics (`/fix`, `/imu/data`, `/camera/color/image_raw`) are active and can be visualized in RViz.
*   **Status:** ✅ **This milestone is largely complete.**

### **Milestone 2: "Simple Auto" - The Divot Alignment & Repair Test**

*   **Goal:** Implement and test the core "sense-transform-act" pipeline in a controlled environment, removing all complex navigation. This is the most critical proof-of-concept.
*   **Workflow:**
    1.  The user manually drives the robot to place a divot clearly in the camera's view.
    2.  The user presses the 'Y' button to begin the test.
    3.  `logitech_gamepad_node` publishes `true` to `/is_autonomous_mode`.
    4.  A new, simple **`align_and_repair_node.py`** subscribes to this signal and activates.
    5.  `divot_detection_node` identifies the divot's pixel coordinates.
    6.  The `align_and_repair_node` uses the camera intrinsic parameters and depth data to perform a **pixel-to-robot coordinate transformation**, calculating the divot's `(x, y)` position in meters relative to the `base_link` frame.
    7.  The node takes exclusive control of `/cmd_vel` and executes a simple **visual servoing routine** to turn and drive forward until the dispenser (located a known 90cm behind the camera) is centered over the calculated divot coordinates.
    8.  Once aligned, it stops the robot (`Twist` message with all zeros) and publishes a command to `/dispense_sand` to run the stepper motor for a fixed test duration (e.g., 2 seconds).
    9.  After dispensing, it publishes `false` to `/is_autonomous_mode` and returns full control to the user.
*   **Development Tasks for this Milestone:**
    *   [ ] Create the `align_and_repair_node.py` file.
    *   [ ] Implement the pixel-to-`base_link` coordinate transformation logic using `tf2`.
    *   [ ] Implement the basic alignment and driving control logic.
    *   [ ] Modify the `logitech_gamepad_node` to handle mode switching as described.

### **Milestone 3: "Full Auto" - Search Pattern Integration**

*   **Goal:** Expand the "Simple Auto" node into a full `mission_control_node` capable of autonomous searching within a small, defined area.
*   **Tasks:**
    1.  Evolve the `align_and_repair_node` into a formal state machine (e.g., `SEARCHING`, `APPROACHING`, `REPAIRING`).
    2.  Implement a simple **2x2 meter "mower pattern"** search algorithm that executes while in the `SEARCHING` state.
    3.  When the 'Y' button is pressed, the node should capture its current RTK-fused position from `/odometry/filtered` and use that as the origin for its local search grid.
    4.  After a repair is complete, the node must be able to resume the search pattern from where it left off.

### **Milestone 4: (Future) System Refinements & NAV2 Integration**

*   **Goal:** Increase the system's robustness, intelligence, and operational scale.
*   **Tasks:**
    1.  Calibrate the dispenser to deliver a precise volume of material based on the divot's area calculated from the vision system's segmentation mask.
    2.  Implement a system to log the GPS coordinates of repaired divots to a file or database to avoid re-servicing and track work.
    3.  Replace the simple mower pattern with the full **ROS 2 Navigation Stack (NAV2)** for intelligent, map-based path planning and obstacle avoidance across larger, more complex areas. 