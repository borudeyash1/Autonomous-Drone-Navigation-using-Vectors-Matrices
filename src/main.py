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
import math
import random

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
                  dynamic_obstacles: List[Dict] = None, no_fly_zones: List[Dict] = None, map_ui=None, end_world=None, planner=None, obstacles=None):
    """
    Animate the drones along their paths.
    
    Args:
        simulator: Simulator instance
        paths: List of paths for each drone
        dynamic_obstacles: List of dynamic obstacles
        no_fly_zones: List of no-fly zones
        map_ui: MapUI instance
        end_world: End world coordinates
        planner: PathPlanner instance
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
    pressed_keys = set()
    camera_keys = {ord('w'), ord('a'), ord('s'), ord('d'), 2490368, 2621440, 2424832, 2555904}
    last_time = time.time()
    greedy_mode = False
    
    sim_dt = 1/240
    render_dt = 1/60
    last_render = time.time()
    stuck_timer = 0
    stuck_threshold = 1.0  # seconds
    stuck_distance = 0.5
    last_pos = None
    
    while running:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Handle keyboard input more efficiently
        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            pressed_keys.add(key)
        # Remove released keys
        for k in list(pressed_keys):
            if k != key and k not in [ord('q'), ord('m'), ord('c'), ord('g'), 9]:
                pressed_keys.remove(k)
        
        if key == ord('q'):
            running = False
        elif key == ord('m'):
            mode = "Manual" if mode == "Autonomous" else "Autonomous"
            print(f"[INFO] Switched to {mode} mode.")
        elif key == ord('c') or key == 9:  # 'c' or Tab
            simulator.toggle_camera_mode()
            print(f"[INFO] Camera mode: {'FPP' if simulator.fpp_mode else 'Free View'} (press Tab or 'c' to toggle)")
        elif key == ord('g'):
            greedy_mode = not greedy_mode
            print(f"[INFO] Pathfinding mode: {'Greedy' if greedy_mode else 'A*'} (press 'g' to toggle)")
        
        # Continuous camera movement in Free View with higher speed
        if not simulator.fpp_mode:
            for k in pressed_keys:
                if k in camera_keys:
                    simulator.move_free_camera(k, dt)
        
        # Get current states
        states = simulator.fleet.get_drone_states()
        
        # Run multiple simulation steps per render frame
        sim_steps = int(max(1, dt / sim_dt))
        for _ in range(sim_steps):
            # Control inputs for each drone
            control_inputs = []
            for i, (state, path, controller) in enumerate(zip(states, paths, controllers)):
                if mode == "Manual":
                    control_input = get_parameter_values()
                else:
                    # Check if direct line to endpoint is clear (2D)
                    pos2d = np.array([state['position'][0], state['position'][1]])
                    end2d = np.array([end_world[0], end_world[1]])
                    grid_pos = planner.world_to_grid(np.array([pos2d[0], pos2d[1], 0]))
                    grid_end = planner.world_to_grid(np.array([end2d[0], end2d[1], 0]))
                    direct_clear = True
                    for gx, gy in bresenham_line(grid_pos[0], grid_pos[1], grid_end[0], grid_end[1]):
                        if (gx, gy) in obstacles:
                            direct_clear = False
                            break
                    if direct_clear:
                        target_pos = np.array([end_world[0], end_world[1], end_world[2]])
                    else:
                        # Follow next waypoint in 2D path
                        if path:
                            target_pos = path[0]
                            if np.linalg.norm(state['position'] - target_pos) < 0.5:
                                path.pop(0)
                        else:
                            target_pos = np.array([end_world[0], end_world[1], end_world[2]])
                    control_input = controller.compute_control(state, {'position': target_pos}, sim_dt)
                control_inputs.append(control_input)
            simulator.step(control_inputs, mode)
        
        # Check if drone is stuck (not progressing)
        if last_pos is not None:
            dist = np.linalg.norm(states[0]['position'] - last_pos)
            if dist < stuck_distance:
                stuck_timer += dt
            else:
                stuck_timer = 0
            if stuck_timer > stuck_threshold:
                print("[INFO] Drone stuck, replanning...")
                # Replan path
                path = planner.plan_path_3d(states[0]['position'], end_world, buildings)
                paths[0] = path
                simulator.fleet.set_drone_path(0, path)
                stuck_timer = 0
        last_pos = np.copy(states[0]['position'])
        
        # Render at 60 FPS
        if current_time - last_render >= render_dt:
            # Get current states
            states = simulator.fleet.get_drone_states()
            # Update status display
            update_status_display(states, mode)
            # Update 2D path tracking for each drone
            if map_ui is not None:
                for i, state in enumerate(states):
                    pos = state['position']
                    grid_size = map_ui.grid_size
                    world_size = 10.0
                    gx = int(round((pos[0] + world_size/2) / world_size * (grid_size-1)))
                    gy = int(round((pos[1] + world_size/2) / world_size * (grid_size-1)))
                    gx = max(0, min(grid_size-1, gx))
                    gy = max(0, min(grid_size-1, gy))
                    map_ui.update_drone_path(i, gx, gy)
            # Keep 2D map visible and updated
            if map_ui is not None:
                map_ui.draw()
            last_render = current_time
    
    # Clean up
    cv2.destroyAllWindows()

def main():
    # Create and show the interactive map UI
    map_ui = MapUI(grid_size=20, cell_size=30)
    grid, start, end, obstacles, obstacle_sizes, dynamic_obstacles, no_fly_zones = map_ui.wait_for_setup()
    # Do NOT close map_ui here!

    # Initialize simulation with 1 drone
    simulator = Simulator(num_drones=1)
    
    # Initialize path planner
    planner = PathPlanner()
    
    # Convert grid start/end to world coordinates
    start_world = planner.grid_to_world(start, z=1.0)
    # Remove dynamic obstacles from environment setup
    buildings = []
    for obs in obstacles:
        size = obstacle_sizes.get(obs, (1, 1))
        world_pos = planner.grid_to_world(obs, z=0.5)
        bw = size[0] * (10.0 / map_ui.grid_size)
        bl = size[1] * (10.0 / map_ui.grid_size)
        bh = random.uniform(2.0, 5.0)
        buildings.append((world_pos[0], world_pos[1], bh/2, bw, bl, bh))
        simulator.add_building(np.array([world_pos[0], world_pos[1], bh/2]), np.array([bw, bl, bh]), color=(0.3, 0.3, 0.7, 1))
    # End point can be on ground or on top of a building
    if random.random() < 0.5 or not buildings:
        end_world = planner.grid_to_world(end, z=1.0)
    else:
        b = random.choice(buildings)
        end_world = np.array([b[0], b[1], b[2]+b[5]/2])
    start_positions = [start_world]
    end_positions = [end_world]
    print(f"[ENV] Buildings: {buildings}")
    print(f"[ENV] Start: {start_world}, End: {end_world}")
    # Plan 2D path using A* on the grid
    path2d = planner.plan_path(start_world, end_world)
    # Convert 2D path to 3D: set z=1.0 for all waypoints except endpoint
    path3d = []
    for wp in path2d[:-1]:
        path3d.append(np.array([wp[0], wp[1], 1.0]))
    # For endpoint, use its actual z (could be on a building)
    if path2d:
        path3d.append(np.array([end_world[0], end_world[1], end_world[2]]))
    paths = [path3d]
    simulator.fleet.set_drone_path(0, path3d)
    
    # Add 3D static obstacles
    for obs in obstacles:
        size = obstacle_sizes.get(obs, (1, 1))
        world_pos = planner.grid_to_world(obs, z=0.5)
        box_size = np.array([size[0], size[1], 1.0])
        simulator.add_obstacle(world_pos, box_size, obstacle_type="box", color=(0.2, 0.2, 0.2, 1))

    # Add 3D no-fly zones (red, semi-transparent)
    for nfz in no_fly_zones:
        world_pos = planner.grid_to_world(nfz, z=0.5)
        simulator.add_obstacle(world_pos, np.array([1.0, 1.0, 1.0]), obstacle_type="box", color=(1, 0, 0, 0.7))

    # Add 3D endpoint marker
    grid_size = map_ui.grid_size
    world_size = 10.0  # Should match planner/simulator
    if hasattr(simulator, 'draw_goal_marker') and end is not None:
        simulator.draw_goal_marker(end[:2], grid_size, world_size)

    # Center the camera on the first drone's start position (if using GUI)
    if hasattr(simulator, 'set_camera_target'):
        simulator.set_camera_target(start_positions[0])
    
    # Animate the drones, pass map_ui and end_world
    animate_drones(simulator, paths, dynamic_obstacles, no_fly_zones, map_ui=map_ui, end_world=end_world, planner=planner, obstacles=obstacles)
    
    # Clean up
    map_ui.close()
    simulator.close()

# Add Bresenham's line algorithm for grid raycasting

def bresenham_line(x0, y0, x1, y1):
    """Yield grid cells on a line from (x0, y0) to (x1, y1)."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            yield (x, y)
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            yield (x, y)
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    yield (x, y)

# Helper: 3D Bresenham's line for grid
def bresenham_line_3d(x0, y0, z0, x1, y1, z1):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    dz = abs(z1 - z0)
    x, y, z = x0, y0, z0
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    sz = 1 if z1 > z0 else -1
    if dx >= dy and dx >= dz:
        yd, zd = dy - dx // 2, dz - dx // 2
        while x != x1:
            yield (x, y, z)
            if yd >= 0:
                y += sy
                yd -= dx
            if zd >= 0:
                z += sz
                zd -= dx
            x += sx
            yd += dy
            zd += dz
    elif dy >= dx and dy >= dz:
        xd, zd = dx - dy // 2, dz - dy // 2
        while y != y1:
            yield (x, y, z)
            if xd >= 0:
                x += sx
                xd -= dy
            if zd >= 0:
                z += sz
                zd -= dy
            y += sy
            xd += dx
            zd += dz
    else:
        xd, yd = dx - dz // 2, dy - dz // 2
        while z != z1:
            yield (x, y, z)
            if xd >= 0:
                x += sx
                xd -= dz
            if yd >= 0:
                y += sy
                yd -= dz
            z += sz
            xd += dx
            yd += dy
    yield (x, y, z)

# Direct line check for 3D obstacles
def is_direct_path_clear_3d(start, end, buildings, planner):
    g0 = planner.world_to_grid_3d(start)
    g1 = planner.world_to_grid_3d(end)
    for gx, gy, gz in bresenham_line_3d(g0[0], g0[1], g0[2], g1[0], g1[1], g1[2]):
        for b in buildings:
            bx, by, bz, bw, bl, bh = b
            if (bx-bw/2 <= planner.grid_to_world_3d((gx,gy,gz))[0] <= bx+bw/2 and
                by-bl/2 <= planner.grid_to_world_3d((gx,gy,gz))[1] <= by+bl/2 and
                bz-bh/2 <= planner.grid_to_world_3d((gx,gy,gz))[2] <= bz+bh/2):
                return False
    return True

if __name__ == "__main__":
    main() 