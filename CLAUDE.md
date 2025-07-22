# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Development Commands

### Building the Project
```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select differential_drive

# Build with debug information
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Running the System
```bash
# Source the environment
source install/setup.bash

# Launch the full system with Logitech gamepad (default)
ros2 launch differential_drive diff_drive.launch.py

# Launch with Sony gamepad
ros2 launch differential_drive diff_drive.launch.py gamepad_type:=sony

# Run individual nodes for testing
ros2 run differential_drive camera_node
ros2 run differential_drive divot_detector
ros2 run differential_drive stepper_imu_node
```

### Testing Commands
```bash
# Run Python linting tests
cd src/differential_drive && python -m pytest test/test_flake8.py
cd src/differential_drive && python -m pytest test/test_pep257.py

# Check topics and nodes
ros2 topic list
ros2 node list
ros2 topic echo /camera/color/image_raw
ros2 topic echo /fix  # GPS data
```

## Architecture Overview

### System Type
This is a **ROS 2 autonomous golf divot detection and repair robot** built for the Jetson Orin Nano platform. The system operates in manual and autonomous modes.

### Core Hardware Integration
- **Motor Control**: 2x VESC motor controllers via USB serial for differential drive
- **Navigation**: ArduSimple RTK GPS module for high-precision positioning  
- **Vision**: Intel RealSense D435i depth camera for divot detection
- **Dispensing**: Arduino Uno with BNO055 IMU and A4988 stepper motor driver
- **Control**: Logitech F710 joystick for manual operation

### Package Structure
- **`differential_drive`**: Main Python package containing all robot nodes
- **`vesc_driver`**: C++ motor controller interface
- **`ublox/ublox_gps`**: RTK GPS driver and message definitions
- **`vesc_interfaces`**: Custom ROS 2 message definitions

### Key Python Nodes (`src/differential_drive/differential_drive/`)
- `camera_node.py`: Intel RealSense D435i camera driver using pyrealsense2
- `divot_detection_intel.py`: YOLO-based divot detection (uses `1600s_aug_100ep.pt` model)
- `logitech_gamepad.py`: Manual control node translating `/joy` to `/cmd_vel`
- `stepper_imu_node.py`: Arduino serial interface for IMU data and dispenser control
- `velocity_control.py`: Differential drive controller converting `/cmd_vel` to VESC commands
- `gps_monitor_node.py`: RTK GPS status monitoring utility

### Arduino Integration
Arduino sketches in `arduino_sketches/` interface with:
- BNO055 IMU for orientation data
- A4988 stepper driver for sand/seed dispensing mechanism
- Serial commands: 'R' (run dispenser), 'S' (stop dispenser)

### Launch Configuration
The main launch file `diff_drive.launch.py` starts:
- Left and right VESC driver nodes with separate configs
- Differential drive velocity controller
- Joy node and gamepad interpreter (Logitech or Sony)
- RTK GPS node with zed_f9p_simple.yaml config  
- Camera and divot detection nodes
- Stepper/IMU node for dispensing control

### Current Development State
- **Milestone 1 Complete**: Manual control and hardware verification working
- **Milestone 2 Planned**: Simple autonomous divot alignment and repair
- **Milestone 3 Planned**: GUI Development
- **Milestone 4 Planned**: Full autonomous search pattern integration
- Mode switching between manual/autonomous via gamepad not yet implemented

### Development Milestones

#### Milestone 2: "Simple Auto" - The Divot Alignment & Repair Test
- **Goal**: Implement and test the core "sense-transform-act" pipeline in a controlled environment
- **Tasks**: Create `align_and_repair_node.py` for visual servoing and divot repair

#### Milestone 3: GUI Development
- **Goal**: Create a user-friendly interface for monitoring and control, replacing separate pop-up windows
- **Technology Choice**: Recommend using PyQt or Tkinter with the rclpy library for creating the ROS 2 GUI node
- **Development Tasks**:
  - Create the `golfbot_gui_node.py`
  - Implement a widget to display the live video from `/camera/color/image_raw` (using cv_bridge)
  - Add status labels to display data from `/fix` (e.g., Latitude, Longitude, Covariance)
  - Create an "Enable Auto Mode" button that publishes `true` to the `/is_autonomous_mode` topic. This button will serve the same function as the 'Y' button on the joystick
  - (Stretch Goal) Overlay bounding boxes from `/divot_detections` onto the video feed

#### Milestone 4: "Full Auto" - Search Pattern Integration
- **Goal**: Expand autonomous capability into full mission control with search patterns
- **Tasks**: Implement state machine for autonomous searching within defined areas

### Key Topics
- `/cmd_vel`: Robot velocity commands (geometry_msgs/Twist)
- `/fix`: RTK GPS position data (sensor_msgs/NavSatFix)
- `/camera/color/image_raw`: Color camera feed
- `/camera/divot_detection/image_raw`: Annotated divot detection results
- `/imu/data`: IMU orientation from Arduino
- `left_vesc/commands/motor/speed`, `right_vesc/commands/motor/speed`: Individual motor commands

### Configuration Files
- `config/left_vesc_config.yaml`, `config/right_vesc_config.yaml`: Motor controller settings
- `config/stepper_imu_config.yaml`: Arduino serial port and dispensing parameters
- YOLO model file: `1600s_aug_100ep.pt` (divot detection weights)