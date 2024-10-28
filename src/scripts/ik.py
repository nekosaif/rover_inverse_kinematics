"""
Inverse Kinematics Calculator for Robotic Arm Control

This module provides functionality to calculate inverse kinematics for a 3-DOF robotic arm.
It converts target end-effector coordinates (x, y, z) into joint angles (θ1, θ2, θ3).

The arm consists of:
- A rotating base (θ1)
- Two arms with lengths l1 and l2
- An optional end-effector extension l3
- A base height d2

Coordinate System:
- Origin is at the base of the robot
- X-axis points forward
- Y-axis points left
- Z-axis points up
"""

from typing import Tuple, Optional
import math
import serial
import logging
from dataclasses import dataclass

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

@dataclass
class ArmDimensions:
    """Stores the physical dimensions of the robotic arm."""
    l1: float  # Length of first arm segment
    l2: float  # Length of second arm segment
    l3: float  # Length of end-effector (optional)
    d2: float  # Height of base from ground

class InverseKinematicsError(Exception):
    """Custom exception for inverse kinematics calculation errors."""
    pass

def validate_input_coordinates(x: float, y: float, z: float, arm_dims: ArmDimensions) -> None:
    """
    Validates if the target coordinates are within the arm's reachable workspace.
    
    Args:
        x: Target x-coordinate
        y: Target y-coordinate
        z: Target z-coordinate
        arm_dims: Robot arm dimensions
        
    Raises:
        InverseKinematicsError: If coordinates are invalid or unreachable
    """
    # Calculate maximum reach
    max_reach = arm_dims.l1 + arm_dims.l2 + arm_dims.l3
    
    # Calculate distance to target
    target_distance = math.sqrt(x*x + y*y + (z - arm_dims.d2)**2)
    
    if target_distance > max_reach:
        raise InverseKinematicsError(
            f"Target position ({x}, {y}, {z}) is beyond maximum reach of {max_reach} units"
        )
    
    if z < 0:
        raise InverseKinematicsError("Target z-coordinate cannot be below ground level")

def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    arm_dims: Optional[ArmDimensions] = None,
    is_service_mode: bool = False
) -> Tuple[float, float, float]:
    """
    Calculates inverse kinematics for the robotic arm.
    
    Args:
        x: Target x-coordinate
        y: Target y-coordinate
        z: Target z-coordinate
        arm_dims: Robot arm dimensions (optional, uses default if not provided)
        is_service_mode: Flag for service mode operation (default: False)
    
    Returns:
        Tuple[float, float, float]: Joint angles (θ1, θ2, θ3) in degrees
        
    Raises:
        InverseKinematicsError: If calculation fails or target is unreachable
    """
    # Use default dimensions if not provided
    if arm_dims is None:
        arm_dims = ArmDimensions(l1=20, l2=14, l3=0, d2=18)
    
    try:
        # Validate input coordinates
        validate_input_coordinates(x, y, z, arm_dims)
        
        # Calculate intermediate values
        d1 = math.sqrt(x*x + y*y) - arm_dims.l3
        d6 = math.sqrt(d1*d1 + (z - arm_dims.d2)**2)
        
        # Calculate joint angles
        θ1 = math.degrees(math.acos(x/(d1 + arm_dims.l3)))
        θ6 = math.acos((arm_dims.l1**2 + d6**2 - arm_dims.l2**2) / (2*arm_dims.l1*d6))
        θ7 = math.acos(d1/d6)
        θ2 = math.degrees(θ6 + θ7)
        θ3 = math.degrees(math.acos(
            (arm_dims.l1**2 + arm_dims.l2**2 - d6**2) / (2*arm_dims.l1*arm_dims.l2)
        ))
        
        return θ1, θ2, θ3
        
    except (ValueError, ZeroDivisionError) as e:
        raise InverseKinematicsError(f"Inverse kinematics calculation failed: {str(e)}")

class RoboticArmController:
    """Controls the robotic arm via serial communication."""
    
    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 0.1):
        """
        Initialize the robotic arm controller.
        
        Args:
            port: Serial port identifier
            baudrate: Serial communication speed
            timeout: Serial timeout in seconds
        """
        try:
            self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            logging.info(f"Connected to robotic arm on port {port}")
        except serial.SerialException as e:
            logging.error(f"Failed to connect to port {port}: {str(e)}")
            raise

    def move_to_position(self, x: float, y: float, z: float, arm_dims: ArmDimensions) -> None:
        """
        Move the robotic arm to the specified position.
        
        Args:
            x: Target x-coordinate
            y: Target y-coordinate
            z: Target z-coordinate
            arm_dims: Robot arm dimensions
        """
        try:
            θ1, θ2, θ3 = inverse_kinematics(x, y, z, arm_dims)
            
            # Convert angles to integers and format command string
            command = f'1{int(θ2):03d}2{int(θ3):03d}3{int(θ1):03d}'
            
            # Send command to arduino
            self.arduino.write(bytes(command, 'utf-8'))
            logging.info(f"Sent command: {command}")
            
        except InverseKinematicsError as e:
            logging.error(f"Movement failed: {str(e)}")
            raise
        except Exception as e:
            logging.error(f"Unexpected error during movement: {str(e)}")
            raise

def main():
    """Main function for interactive control of the robotic arm."""
    # Define arm dimensions
    arm_dims = ArmDimensions(l1=22, l2=16, l3=0, d2=17)
    
    try:
        # Initialize controller
        controller = RoboticArmController('COM17')
        
        # Interactive control loop
        while True:
            try:
                # Get user input
                inp = input('Enter x y z (or empty to exit): ')
                if not inp:
                    break
                
                # Parse coordinates
                x, y, z = map(float, inp.split())
                print(f"Target position: x={x}, y={y}, z={z}")
                
                # Move arm to position
                controller.move_to_position(x, y, z, arm_dims)
                
            except ValueError:
                logging.error("Invalid input format. Please enter three numbers separated by spaces.")
            except InverseKinematicsError as e:
                logging.error(str(e))
            except Exception as e:
                logging.error(f"Unexpected error: {str(e)}")
                
    except KeyboardInterrupt:
        logging.info("Program terminated by user")
    finally:
        logging.info("Shutting down")

if __name__ == '__main__':
    main()
