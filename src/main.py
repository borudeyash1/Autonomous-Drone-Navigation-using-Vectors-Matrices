import numpy as np
import cv2
import time
import keyboard
import sys
import pybullet as p
from simulation.simulator import Simulator
from perception.obstacle_detector import ObstacleDetector
from planning.path_planner import PathPlanner
from control.controller import Controller
from ui.map_ui import MapUI
from typing import List, Dict, Tuple

def create_status_window():
    """Create a window to display drone status."""
    cv2.namedWindow('Drone Status', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Drone Status', 400, 300)

def update_status_display(states: List[Dict], mode: str = "Autonomous"):
    """Update the status display with current drone states."""
    # Create a black background
    status_image = np.zeros((300, 400, 3), dtype=np.uint8)
    
    # Add status information
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    font_thickness = 1
    line_height = 30
    y_pos = 30
    
    # Mode indicator
    mode_color = (0, 255, 0) if mode == "Manual" else (0, 165, 255)  # Green for Manual, Orange for Autonomous
    cv2.putText(status_image, f"Mode: {mode}", (10, y_pos), font, font_scale, mode_color, font_thickness)
    y_pos += line_height
    
    # Display information for each drone
    for i, state in enumerate(states):
        pos = state['position']
        vel = state['velocity']
        
        # Drone number
        cv2.putText(status_image, f"Drone {i+1}:", (10, y_pos), font, font_scale, (255, 255, 255), font_thickness)
        y_pos += line_height
        
        # Position information
        cv2.putText(status_image, f"X: {pos[0]:+.2f} m", (20, y_pos), font, font_scale, (255, 255, 255), font_thickness)
        y_pos += line_height
        cv2.putText(status_image, f"Y: {pos[1]:+.2f} m", (20, y_pos), font, font_scale, (255, 255, 255), font_thickness)
        y_pos += line_height
        cv2.putText(status_image, f"Z: {pos[2]:+.2f} m", (20, y_pos), font, font_scale, (255, 255, 255), font_thickness)
        y_pos += line_height
        
        # Velocity information
        cv2.putText(status_image, f"VX: {vel[0]:+.2f} m/s", (20, y_pos), font, font_scale, (255, 255, 255), font_thickness)
        y_pos += line_height
        cv2.putText(status_image, f"VY: {vel[1]:+.2f} m/s", (20, y_pos), font, font_scale, (255, 255, 255), font_thickness)
        y_pos += line_height
        cv2.putText(status_image, f"VZ: {vel[2]:+.2f} m/s", (20, y_pos), font, font_scale, (255, 255, 255), font_thickness)
        y_pos += line_height
        
        # Add spacing between drones
        y_pos += line_height
    
    # Display the status window
    cv2.imshow('Drone Status', status_image)
    cv2.waitKey(1)

def create_parameter_ui():
    """Create a window for parameter adjustment."""
    cv2.namedWindow('Parameters', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Parameters', 400, 200)
    
    # Create trackbars for parameters
    cv2.createTrackbar('Thrust', 'Parameters', 50, 100, lambda x: None)
    cv2.createTrackbar('Roll', 'Parameters', 50, 100, lambda x: None)
    cv2.createTrackbar('Pitch', 'Parameters', 50, 100, lambda x: None)
    cv2.createTrackbar('Yaw', 'Parameters', 50, 100, lambda x: None)

def get_parameter_values():
    """Get the current values of the parameters."""
    # Map sliders to realistic ranges
    # Thrust: [0, 100] -> [0, 20] N
    # Roll/Pitch/Yaw: [0, 100] -> [-0.5, 0.5] radians
    thrust_slider = cv2.getTrackbarPos('Thrust', 'Parameters')
    roll_slider = cv2.getTrackbarPos('Roll', 'Parameters')
    pitch_slider = cv2.getTrackbarPos('Pitch', 'Parameters')
    yaw_slider = cv2.getTrackbarPos('Yaw', 'Parameters')
    thrust = (thrust_slider / 100.0) * 20.0  # 0 to 20 N
    roll = (roll_slider / 100.0 - 0.5) * 1.0  # -0.5 to 0.5 rad
    pitch = (pitch_slider / 100.0 - 0.5) * 1.0  # -0.5 to 0.5 rad
    yaw = (yaw_slider / 100.0 - 0.5) * 1.0  # -0.5 to 0.5 rad
    return np.array([thrust, roll, pitch, yaw])

def animate_drones(simulator: Simulator, paths: List[List[Tuple[float, float, float]]], 
                  dynamic_obstacles: List[Dict] = None, no_fly_zones: List[Dict] = None, map_ui=None):
    """
    Animate the drones along their paths.
    
    Args:
        simulator: Simulator instance
        paths: List of paths for each drone
        dynamic_obstacles: List of dynamic obstacles
        no_fly_zones: List of no-fly zones
        map_ui: MapUI instance
    """
    # Create status and parameter windows
    create_status_window()
    create_parameter_ui()
    
    # Initialize controllers for each drone
    controllers = [Controller() for _ in range(len(paths))]
    
    # Main simulation loop
    running = True
    mode = "Autonomous"
    last_mode = mode
    
    while running:
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            running = False
        elif key == ord('m'):
            mode = "Manual" if mode == "Autonomous" else "Autonomous"
            print(f"[INFO] Switched to {mode} mode.")
            if mode == "Manual":
                print("[INFO] Manual mode: Use the sliders in the 'Parameters' window to control Thrust, Roll, Pitch, and Yaw.")
            else:
                print("[INFO] Autonomous mode: Drones follow their planned paths.")
        
        if mode != last_mode:
            last_mode = mode
        
        # Get current states
        states = simulator.fleet.get_drone_states()
        
        # Update status display
        update_status_display(states, mode)
        
        # Control inputs for each drone
        control_inputs = []
        
        for i, (state, path, controller) in enumerate(zip(states, paths, controllers)):
            if mode == "Manual":
                # Manual control
                control_input = get_parameter_values()
            else:
                # Autonomous control
                if path:
                    target_pos = np.array(path[0])
                    control_input = controller.compute_control(state, {'position': target_pos}, 1/240)
                    
                    # Check if waypoint reached
                    if np.linalg.norm(state['position'] - target_pos) < 0.1:
                        path.pop(0)
                else:
                    # Hover in place if no path
                    control_input = np.array([9.81, 0, 0, 0])
            
            control_inputs.append(control_input)
        
        # Step the simulation
        simulator.step(control_inputs, 1/240)
        
        # Update dynamic obstacles and no-fly zones
        if dynamic_obstacles:
            for obstacle in dynamic_obstacles:
                obstacle['position'] += obstacle['velocity'] * (1/240)
                if obstacle['position'][0] > 10 or obstacle['position'][0] < -10:
                    obstacle['velocity'][0] *= -1
                if obstacle['position'][1] > 10 or obstacle['position'][1] < -10:
                    obstacle['velocity'][1] *= -1
        
        if no_fly_zones:
            for zone in no_fly_zones:
                zone['position'] += zone['velocity'] * (1/240)
                if zone['position'][0] > 10 or zone['position'][0] < -10:
                    zone['velocity'][0] *= -1
                if zone['position'][1] > 10 or zone['position'][1] < -10:
                    zone['velocity'][1] *= -1
        
        # Keep 2D map visible and updated
        if map_ui is not None:
            map_ui.draw()
    
    # Clean up
    cv2.destroyAllWindows()

def main():
    # Create and show the interactive map UI
    map_ui = MapUI(grid_size=20, cell_size=30)
    grid, start, end, obstacles, obstacle_sizes, dynamic_obstacles, no_fly_zones = map_ui.wait_for_setup()
    # Do NOT close map_ui here!

    # Initialize simulation with 3 drones
    simulator = Simulator(num_drones=3)
    
    # Initialize path planner
    planner = PathPlanner()
    
    # Convert grid start/end to world coordinates
    start_world = planner.grid_to_world(start, z=1.0)
    end_world = planner.grid_to_world(end, z=1.0)
    start_positions = [start_world for _ in range(3)]
    end_positions = [end_world for _ in range(3)]
    
    # Plan paths for each drone
    paths = []
    for start, end in zip(start_positions, end_positions):
        path = planner.plan_path(start, end)
        if path:
            paths.append(path)
        else:
            print(f"Warning: Could not find path from {start} to {end}")
            paths.append([])
    
    # Set paths in the simulator
    for i, path in enumerate(paths):
        simulator.fleet.set_drone_path(i, path)
    
    # Add 3D static obstacles
    for obs in obstacles:
        size = obstacle_sizes.get(obs, (1, 1))
        world_pos = planner.grid_to_world(obs, z=0.5)
        box_size = np.array([size[0], size[1], 1.0])
        simulator.add_obstacle(world_pos, box_size, obstacle_type="box", color=(0.2, 0.2, 0.2, 1))

    # Add 3D dynamic obstacles (blue)
    for dyn in dynamic_obstacles:
        world_pos = planner.grid_to_world(dyn, z=0.5)
        simulator.add_obstacle(world_pos, np.array([1.0, 1.0, 1.0]), obstacle_type="box", color=(0, 0, 1, 1))

    # Add 3D no-fly zones (red, semi-transparent)
    for nfz in no_fly_zones:
        world_pos = planner.grid_to_world(nfz, z=0.5)
        simulator.add_obstacle(world_pos, np.array([1.0, 1.0, 1.0]), obstacle_type="box", color=(1, 0, 0, 0.7))

    # Center the camera on the first drone's start position (if using GUI)
    if hasattr(simulator, 'set_camera_target'):
        simulator.set_camera_target(start_positions[0])
    
    # Animate the drones, pass map_ui
    animate_drones(simulator, paths, dynamic_obstacles, no_fly_zones, map_ui=map_ui)
    
    # Clean up
    map_ui.close()
    simulator.close()

if __name__ == "__main__":
    main() 