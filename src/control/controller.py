import numpy as np
from typing import Dict

class Controller:
    def __init__(self, kp: float = 1.0, ki: float = 0.1, kd: float = 0.5):
        """
        Initialize the PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Initialize error terms
        self.prev_error = np.zeros(3)
        self.integral = np.zeros(3)
        
        # Control limits
        self.max_thrust = 20.0  # N
        self.max_torque = 5.0   # Nm
    
    def compute_control(self, current_state: Dict, target_state: Dict, dt: float) -> np.ndarray:
        """
        Compute control input using PID control.
        
        Args:
            current_state: Current state of the drone
            target_state: Target state to reach
            dt: Time step in seconds
            
        Returns:
            Control input [thrust, roll, pitch, yaw]
        """
        # Position error
        pos_error = target_state['position'] - current_state['position']
        
        # Velocity error
        vel_error = target_state.get('velocity', np.zeros(3)) - current_state['velocity']
        
        # Update integral term
        self.integral += pos_error * dt
        
        # Compute derivative term
        derivative = (pos_error - self.prev_error) / dt
        
        # Compute control output
        control = (
            self.kp * pos_error +  # Proportional term
            self.ki * self.integral +  # Integral term
            self.kd * derivative  # Derivative term
        )
        
        # Update previous error
        self.prev_error = pos_error
        
        # Convert to thrust and torques
        thrust = np.clip(control[2] + 9.81, 0, self.max_thrust)  # Add gravity compensation
        roll_torque = np.clip(control[0], -self.max_torque, self.max_torque)
        pitch_torque = np.clip(control[1], -self.max_torque, self.max_torque)
        
        # Yaw control (optional)
        yaw_torque = 0.0
        
        return np.array([thrust, roll_torque, pitch_torque, yaw_torque])
    
    def reset(self):
        """Reset the controller state."""
        self.prev_error = np.zeros(3)
        self.integral = np.zeros(3) 