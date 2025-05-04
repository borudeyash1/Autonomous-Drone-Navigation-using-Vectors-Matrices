import numpy as np
import pybullet as p
from typing import List, Dict, Tuple
from .quadrotor import Quadrotor
from control.controller import Controller

class DroneFleet:
    def __init__(self, client: int, num_drones: int = 1):
        """
        Initialize a fleet of drones.
        
        Args:
            client: PyBullet client ID
            num_drones: Number of drones to create
        """
        self.client = client
        self.drones: List[Quadrotor] = []
        self.drone_states: List[dict] = []
        self.drone_paths: List[List[Tuple[float, float, float]]] = []
        self.drone_controllers: List[Controller] = []
        self.drone_colors = [
            [0.2, 0.4, 1.0, 1],  # Blue
            [1.0, 0.2, 0.2, 1],  # Red
            [0.2, 1.0, 0.2, 1],  # Green
            [1.0, 1.0, 0.2, 1],  # Yellow
            [1.0, 0.2, 1.0, 1],  # Purple
        ]
        
        # Initialize drones
        for i in range(num_drones):
            # Create drone with unique color
            color = self.drone_colors[i % len(self.drone_colors)]
            drone = Quadrotor(client, color=color)
            self.drones.append(drone)
            self.drone_states.append(drone.get_state())
            self.drone_paths.append([])
            self.drone_controllers.append(Controller())
    
    def add_drone(self, position: np.ndarray = None) -> int:
        """
        Add a new drone to the fleet.
        
        Args:
            position: Initial position of the drone
            
        Returns:
            Index of the new drone
        """
        color = self.drone_colors[len(self.drones) % len(self.drone_colors)]
        drone = Quadrotor(self.client, color=color)
        if position is not None:
            drone.set_position(position)
        self.drones.append(drone)
        self.drone_states.append(drone.get_state())
        self.drone_paths.append([])
        self.drone_controllers.append(Controller())
        return len(self.drones) - 1
    
    def remove_drone(self, drone_idx: int):
        """
        Remove a drone from the fleet.
        
        Args:
            drone_idx: Index of the drone to remove
        """
        if 0 <= drone_idx < len(self.drones):
            # Remove the drone's model from the simulation
            self.drones[drone_idx].reset(position=[1000, 1000, 1000])  # Move far away
            del self.drones[drone_idx]
            del self.drone_states[drone_idx]
            del self.drone_paths[drone_idx]
            del self.drone_controllers[drone_idx]
    
    def set_drone_path(self, drone_idx: int, path: List[Tuple[float, float, float]]):
        """
        Set the path for a specific drone.
        
        Args:
            drone_idx: Index of the drone
            path: List of (x,y,z) waypoints
        """
        if 0 <= drone_idx < len(self.drones):
            self.drone_paths[drone_idx] = path
    
    def step(self, dt: float):
        """
        Step the simulation for all drones.
        
        Args:
            dt: Time step in seconds
        """
        for i, (drone, controller) in enumerate(zip(self.drones, self.drone_controllers)):
            if self.drone_paths[i]:
                # Get current and target states
                current_state = drone.get_state()
                target_pos = np.array(self.drone_paths[i][0])
                target_state = {
                    'position': target_pos,
                    'velocity': np.zeros(3),
                    'orientation': np.zeros(3)
                }
                
                # Compute control input
                control_input = controller.compute_control(current_state, target_state, dt)
                
                # Apply control
                drone.apply_control(control_input, dt)
                
                # Update visuals
                drone.update_visuals()
                
                # Check if waypoint reached
                if np.linalg.norm(current_state['position'] - target_pos) < 0.1:
                    self.drone_paths[i].pop(0)
    
    def get_drone_states(self) -> List[dict]:
        """Get the current states of all drones."""
        self.drone_states = [drone.get_state() for drone in self.drones]
        return self.drone_states
    
    def reset(self):
        """Reset all drones to their initial states."""
        for drone in self.drones:
            drone.reset()
        self.drone_states = [drone.get_state() for drone in self.drones]
        self.drone_paths = [[] for _ in self.drones]
    
    def close(self):
        """Clean up resources."""
        for drone in self.drones:
            drone.reset(position=[1000, 1000, 1000])  # Move far away 
    
    def set_drone_position(self, drone_idx, position, orientation=None):
        if 0 <= drone_idx < len(self.drones):
            self.drones[drone_idx].set_position(position) 