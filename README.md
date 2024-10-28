# 3-DOF Robotic Arm Control System

A complete control system for a 3-DOF robotic arm with inverse kinematics calculation and Arduino-based motor control. The system consists of two main components:
1. Arduino firmware for motor control and calibration
2. Python inverse kinematics calculator and control interface

## System Overview

### Hardware Components
- 2 Linear actuators with potentiometer feedback
- 1 Rotating base with potentiometer feedback
- Arduino board (e.g., Arduino Uno/Nano)
- Power supply for actuators
- Calibration buttons
- Status LED

### Features
- Real-time position control
- Automated calibration system
- Inverse kinematics calculation
- Position feedback with potentiometers
- EEPROM-based calibration storage
- Serial communication interface
- Comprehensive error handling
- Safety limits and validations

## Getting Started

### Prerequisites
- Arduino IDE (1.8.x or later)
- Python 3.7+
- Required Python packages:
  ```
  pyserial>=3.5
  typing>=3.7.4
  ```

### Installation

1. **Arduino Firmware**
   ```bash
   # Clone the repository
   git clone https://github.com/yourusername/robotic-arm-control.git
   
   # Open inverse_kinematics_with_calibration.ino in Arduino IDE
   # Upload to your Arduino board
   ```

2. **Python Controller**
   ```bash
   # Install required packages
   pip install -r requirements.txt
   
   # Run the control interface
   python ik.py
   ```

## Hardware Setup

### Pin Configuration

#### Arduino Connections
```
// Actuator 1
Pin 2: ACT1_DOWN
Pin 3: ACT1_UP
Pin A3: ACT1_POT

// Actuator 2
Pin 4: ACT2_DOWN
Pin 5: ACT2_UP
Pin A4: ACT2_POT

// Base Rotation
Pin 6: BASE_RIGHT
Pin 7: BASE_LEFT
Pin A5: BASE_POT

// Control
Pin 8: CALIBRATION
Pin 9: BASE_CALIBRATION
```

### Calibration Process

1. Power on the system
2. Wait for the LED to start blinking
3. Press and hold the calibration button
4. Follow the calibration sequence:
   - Actuator 2 maximum position
   - Actuator 1 maximum position
   - Actuator 1 minimum position
   - Actuator 2 minimum position
   - Base minimum position
   - Base maximum position
5. Release calibration button when complete

## Usage

### Python Control Interface

```python
from robotic_arm import RoboticArmController, ArmDimensions

# Define arm dimensions
arm_dims = ArmDimensions(l1=22, l2=16, l3=0, d2=17)

# Initialize controller
controller = RoboticArmController('COM17')  # Use appropriate port

# Move to position
controller.move_to_position(x=10, y=5, z=15, arm_dims=arm_dims)
```

### Manual Control via Serial

Send commands in the format:
```
[actuator_number][position_value]
```

Example:
```
1090    # Move actuator 1 to 90 degrees
2045    # Move actuator 2 to 45 degrees
3180    # Rotate base to 180 degrees
```

## Coordinate System

- Origin: Base of robot
- X-axis: Forward direction
- Y-axis: Left direction
- Z-axis: Upward direction

### Workspace Limits
- Base rotation (θ1): 0° to 90°
- Actuator 1 (θ2): 0° to 98°
- Actuator 2 (θ3): 35° to 134°

## Safety Features

- Stall detection
- Position feedback validation
- Workspace boundary checking
- Calibration verification
- Error logging
- Emergency stop capability

## Configuration

### Arduino Configuration
```cpp
// Modify these constants in the firmware as needed
static const uint8_t CALIBRATION_BLINK_TIME = 3;
static const uint8_t STALL_TIME = 1;
static const uint8_t POT_OFFSET = 5;
static const uint8_t POT_TOLERANCE = 3;
```

### Python Configuration
```python
# Adjust arm dimensions to match your hardware
arm_dims = ArmDimensions(
    l1=20,  # Length of first arm segment
    l2=14,  # Length of second arm segment
    l3=0,   # End-effector length
    d2=18   # Base height
)
```
