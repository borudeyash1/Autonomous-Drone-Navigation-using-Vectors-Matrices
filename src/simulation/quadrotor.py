import numpy as np
import pybullet as p
import pybullet_data
from typing import Tuple, List, Dict

class Quadrotor:
    def __init__(self, client: int, color: List[float] = None):
        """
        Initialize a quadrotor.
        
        Args:
            client: PyBullet client ID
            color: RGBA color for the drone body [r,g,b,a]
        """
        self.client = client
        self.color = color if color is not None else [0.2, 0.4, 1.0, 1]  # Default blue
        self.body_id = self._load_model()
        self.prop_ids = self._create_props()
        self._setup_constraints()
        
    def _load_model(self) -> int:
        """Load the quadrotor model."""
        # Create quadrotor body
        body_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05])
        visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05], rgbaColor=self.color)
        body_id = p.createMultiBody(baseMass=1.0,
                                  baseCollisionShapeIndex=body_id,
                                  baseVisualShapeIndex=visual_id,
                                  basePosition=[0, 0, 0.5])
        return body_id
    
    def _create_props(self) -> List[int]:
        """Create the propellers."""
        prop_ids = []
        prop_positions = [
            [0.15, 0.15, 0],  # Front right
            [-0.15, 0.15, 0],  # Front left
            [0.15, -0.15, 0],  # Back right
            [-0.15, -0.15, 0]  # Back left
        ]
        
        for pos in prop_positions:
            prop_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=0.02)
            visual_id = p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.02, rgbaColor=[0.8, 0.8, 0.8, 1])
            prop_id = p.createMultiBody(baseMass=0.1,
                                      baseCollisionShapeIndex=prop_id,
                                      baseVisualShapeIndex=visual_id,
                                      basePosition=pos)
            prop_ids.append(prop_id)
            
            # Create fixed constraint between prop and body
            p.createConstraint(self.body_id, -1, prop_id, -1,
                             p.JOINT_FIXED, [0, 0, 0],
                             [0, 0, 0], pos)
        
        return prop_ids
    
    def _setup_constraints(self):
        """Setup constraints for the quadrotor."""
        # Add damping to prevent excessive movement
        p.changeDynamics(self.body_id, -1, linearDamping=0.04, angularDamping=0.04)
        for prop_id in self.prop_ids:
            p.changeDynamics(prop_id, -1, linearDamping=0.04, angularDamping=0.04)
    
    def apply_control(self, control_input: np.ndarray, dt: float):
        """
        Apply control input to the quadrotor.
        
        Args:
            control_input: Control input [thrust, roll, pitch, yaw]
            dt: Time step in seconds
        """
        # Convert control input to forces and torques
        thrust = control_input[0]
        roll_torque = control_input[1]
        pitch_torque = control_input[2]
        yaw_torque = control_input[3]
        
        # Apply forces and torques
        p.applyExternalForce(self.body_id, -1, [0, 0, thrust], [0, 0, 0], p.WORLD_FRAME)
        p.applyExternalTorque(self.body_id, -1, [roll_torque, pitch_torque, yaw_torque], p.WORLD_FRAME)
    
    def get_state(self) -> Dict:
        """Get the current state of the quadrotor."""
        pos, orn = p.getBasePositionAndOrientation(self.body_id)
        vel, ang_vel = p.getBaseVelocity(self.body_id)
        
        return {
            'position': np.array(pos),
            'orientation': np.array(p.getEulerFromQuaternion(orn)),
            'velocity': np.array(vel),
            'angular_velocity': np.array(ang_vel)
        }
    
    def set_position(self, position: np.ndarray):
        """Set the position of the quadrotor."""
        p.resetBasePositionAndOrientation(self.body_id, position, [0, 0, 0, 1])
    
    def reset(self, position: np.ndarray = None):
        """Reset the quadrotor to its initial state."""
        if position is None:
            position = [0, 0, 0.5]
        self.set_position(position)
        p.resetBaseVelocity(self.body_id, [0, 0, 0], [0, 0, 0])
    
    def update_visuals(self):
        """Update the visual appearance of the quadrotor."""
        # Update prop rotation based on thrust
        state = self.get_state()
        thrust = state['velocity'][2]  # Use vertical velocity as proxy for thrust
        for prop_id in self.prop_ids:
            p.resetBasePositionAndOrientation(
                prop_id,
                p.getBasePositionAndOrientation(prop_id)[0],
                p.getQuaternionFromEuler([0, 0, thrust * 10])
            ) 