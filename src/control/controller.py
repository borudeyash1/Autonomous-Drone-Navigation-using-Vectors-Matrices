import numpy as np
from typing import Dict
import time

class Controller:
    def __init__(self, kp: float = 5.0, ki: float = 0.2, kd: float = 2.0):
        """
        Initialize the PID controller with more aggressive gains.
        Args:
            kp: Proportional gain (increased from 2.0)
            ki: Integral gain (increased from 0.1)
            kd: Derivative gain (increased from 1.0)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Initialize error terms
        self.prev_error = np.zeros(3)
        self.integral = np.zeros(3)
        self.last_error = np.zeros(3)
        self.last_time = time.time()
        
        # Control limits
        self.max_thrust = 20.0  # N
        self.max_torque = 5.0   # Nm
    
    def compute_control(self, current_state: Dict, target_state: Dict, dt: float) -> np.ndarray:
        """
        Compute control input using enhanced PID control for proper drone movement.
        
        Args:
            current_state: Current state of the drone
            target_state: Target state to reach
            dt: Time step in seconds
            
        Returns:
            Control input [thrust, roll, pitch, yaw]
        """
        # Position error
        pos_error = target_state['position'] - current_state['position']
        error_magnitude = np.linalg.norm(pos_error)
        
        # Adaptive gains based on error magnitude
        if error_magnitude > 3.0:
            kp = 8.0
            kd = 3.0
        elif error_magnitude > 1.0:
            kp = 5.0
            kd = 2.0
        else:
            kp = 3.0
            kd = 1.0
        
        # Update integral term with anti-windup
        self.integral += pos_error * dt
        self.integral = np.clip(self.integral, -2.0, 2.0)  # Anti-windup
        
        # Compute derivative term
        if dt > 0:
            derivative = (pos_error - self.prev_error) / dt
        else:
            derivative = np.zeros(3)
        
        # Compute control output
        control = (
            kp * pos_error +  # Proportional term
            self.ki * self.integral +  # Integral term
            kd * derivative  # Derivative term
        )
        
        # Update previous error
        self.prev_error = pos_error
        
        # Convert to thrust and orientation
        # Base thrust to hover
        base_thrust = 9.81  # Gravity compensation
        
        # Add vertical control
        thrust = base_thrust + np.clip(control[2], -5.0, 10.0)
        thrust = np.clip(thrust, 5.0, 25.0)  # Reasonable thrust limits
        
        # Convert horizontal forces to roll/pitch
        roll = np.clip(-control[1] / max(thrust, 1.0), -0.3, 0.3)
        pitch = np.clip(control[0] / max(thrust, 1.0), -0.3, 0.3)
        
        # Yaw control to face target
        if error_magnitude > 0.1:
            target_direction = np.arctan2(pos_error[1], pos_error[0])
            current_yaw = current_state['orientation'][2]
            yaw_error = target_direction - current_yaw
            
            # Normalize yaw error to [-π, π]
            while yaw_error > np.pi:
                yaw_error -= 2 * np.pi
            while yaw_error < -np.pi:
                yaw_error += 2 * np.pi
            
            yaw = np.clip(yaw_error * 2.0, -0.5, 0.5)
        else:
            yaw = 0.0
        
        return np.array([thrust, roll, pitch, yaw])
    
    def reset(self):
        """Reset the controller state."""
        self.prev_error = np.zeros(3)
        self.integral = np.zeros(3)
        self.last_error = np.zeros(3)
        self.last_time = time.time() 