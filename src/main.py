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
    
    # Create trackbars for parameters with default value 50
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
                  dynamic_obstacles: List[Dict] = None, no_fly_zones: List[Dict] = None, 
                  map_ui=None, end_world=None, planner=None, obstacles=None, buildings=None):
    """
    Animate the drones along their paths with clear path visualization.
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
    stuck_threshold = 1.0
    stuck_distance = 0.5
    last_pos = None
    
    while running:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            pressed_keys.add(key)
        for k in list(pressed_keys):
            if k != key and k not in [ord('q'), ord('m'), ord('c'), ord('g'), 9]:
                pressed_keys.remove(k)
        
        if key == ord('q'):
            running = False
        elif key == ord('m'):
            mode = "Manual" if mode == "Autonomous" else "Autonomous"
            print(f"[INFO] Switched to {mode} mode.")
        elif key == ord('c') or key == 9:
            simulator.toggle_camera_mode()
        elif key == ord('g'):
            greedy_mode = not greedy_mode
            print(f"[INFO] Pathfinding mode: {'Greedy' if greedy_mode else 'A*'}")
        
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
                    # Always aim for the endpoint if it's visible
                    pos2d = np.array([state['position'][0], state['position'][1]])
                    end2d = np.array([end_world[0], end_world[1]])
                    grid_pos = planner.world_to_grid(np.array([pos2d[0], pos2d[1], 0]))
                    grid_end = planner.world_to_grid(np.array([end2d[0], end2d[1], 0]))
                    
                    # Check if direct path to endpoint is clear
                    direct_clear = True
                    for gx, gy in bresenham_line(grid_pos[0], grid_pos[1], grid_end[0], grid_end[1]):
                        if (gx, gy) in obstacles:
                            direct_clear = False
                            break
                    
                    if direct_clear:
                        target_pos = np.array([end_world[0], end_world[1], end_world[2]])
                        # Update path visualization for direct path
                        if map_ui is not None:
                            map_ui.clear_path()
                            map_ui.add_path_point(grid_pos[0], grid_pos[1])
                            map_ui.add_path_point(grid_end[0], grid_end[1])
                            map_ui.draw()
                    else:
                        # Follow next waypoint in path
                        if path:
                            target_pos = path[0]
                            if np.linalg.norm(state['position'] - target_pos) < 0.5:
                                path.pop(0)
                                # Update path visualization
                                if map_ui is not None:
                                    map_ui.clear_path()
                                    for wp in path:
                                        grid_wp = planner.world_to_grid(wp)
                                        map_ui.add_path_point(grid_wp[0], grid_wp[1])
                                    map_ui.draw()
                        else:
                            target_pos = np.array([end_world[0], end_world[1], end_world[2]])
                    
                    # Aggressive control inputs
                    k = 5.0  # Increased from 3.0
                    thrust = 25.0  # Increased from 20.0
                    
                    # Calculate position error
                    pos_error = target_pos - state['position']
                    
                    # Calculate desired orientation
                    pitch = np.clip(k * pos_error[1] / max(1.0, np.linalg.norm(pos_error)), -1.0, 1.0)
                    roll = np.clip(k * pos_error[0] / max(1.0, np.linalg.norm(pos_error)), -1.0, 1.0)
                    
                    # Calculate yaw to face target
                    delta = target_pos[:2] - state['position'][:2]
                    desired_yaw = np.arctan2(delta[1], delta[0])
                    yaw = desired_yaw
                    
                    # Apply additional thrust if far from target
                    if np.linalg.norm(pos_error) > 2.0:
                        thrust *= 1.5
                    
                    # Add gravity compensation
                    thrust += 9.81  # Compensate for gravity
                    
                    control_input = np.array([thrust, roll, pitch, yaw])
                    print(f"[DEBUG] Drone pos: {state['position']}, Target: {target_pos}")
                    print(f"[DEBUG] Control input: {control_input}")
                    
                    if np.linalg.norm(state['position'] - target_pos) < 0.5 and np.allclose(target_pos, [end_world[0], end_world[1], end_world[2]], atol=0.1):
                        print("[INFO] Drone reached the endpoint!")
                
                control_inputs.append(control_input)
            simulator.step(control_inputs, mode)
        
        # Check if drone is stuck
        if last_pos is not None:
            dist = np.linalg.norm(states[0]['position'] - last_pos)
            if dist < stuck_distance:
                stuck_timer += dt
            else:
                stuck_timer = 0
            if stuck_timer > stuck_threshold:
                print("[INFO] Drone stuck, replanning...")
                # Replan path with more aggressive parameters
                path = planner.plan_path_3d(states[0]['position'], end_world, buildings)
                paths[0] = path
                simulator.fleet.set_drone_path(0, path)
                # Update path visualization
                if map_ui is not None:
                    map_ui.clear_path()
                    for wp in path:
                        grid_wp = planner.world_to_grid(wp)
                        map_ui.add_path_point(grid_wp[0], grid_wp[1])
                    map_ui.draw()
                stuck_timer = 0
        last_pos = np.copy(states[0]['position'])
        
        # Render at 60 FPS
        if current_time - last_render >= render_dt:
            # Update displays and UI
            states = simulator.fleet.get_drone_states()
            update_status_display(states, mode)
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
                    map_ui.set_drone_pos((gx, gy))
                map_ui.draw()
            last_render = current_time
    
    # Clean up
    cv2.destroyAllWindows()

def main():
    # Reset and clean up
    print("[RESET] Resetting simulation and map UI...")
    map_ui = MapUI(grid_size=20, cell_size=30)
    grid, start, end, obstacles, obstacle_sizes, dynamic_obstacles, no_fly_zones = map_ui.wait_for_setup()
    
    simulator = Simulator(num_drones=1)
    planner = PathPlanner()
    start_world = planner.grid_to_world(start, z=1.0)
    
    # Create buildings and obstacles
    buildings = []
    for obs in obstacles:
        size = obstacle_sizes.get(obs, (1, 1))
        world_pos = planner.grid_to_world(obs, z=0.5)
        bw = size[0] * (10.0 / map_ui.grid_size)
        bl = size[1] * (10.0 / map_ui.grid_size)
        bh = 3.0
        buildings.append((world_pos[0], world_pos[1], bh/2, bw, bl, bh))
        simulator.add_building(np.array([world_pos[0], world_pos[1], bh/2]), np.array([bw, bl, bh]), color=(0.3, 0.3, 0.7, 1))
    end_world = planner.grid_to_world(end, z=1.0)
    print(f"[ENV] Start: {start_world}, End: {end_world}")
    # Plan path (8-connected A*)
    path2d = planner.plan_path(start_world, end_world)
    print("[PATH] Planned 2D path waypoints:")
    for wp in path2d:
        print(f"  {wp}")
    path3d = [np.array([wp[0], wp[1], 1.0]) for wp in path2d[:-1]]
    if path2d:
        path3d.append(np.array([end_world[0], end_world[1], end_world[2]]))
    print("[PATH] Planned 3D path waypoints:")
    for wp in path3d:
        print(f"  {wp}")
    # Visualize path in 2D map
    map_ui.clear_path()
    for wp in path2d:
        grid_pos = planner.world_to_grid(wp)
        map_ui.add_path_point(grid_pos[0], grid_pos[1])
    map_ui.draw()
    # Set up drone paths
    paths = [path3d]
    simulator.fleet.set_drone_path(0, path3d)
    # Add 3D endpoint marker (red)
    if hasattr(simulator, 'draw_goal_marker') and end is not None:
        simulator.draw_goal_marker(end[:2], map_ui.grid_size, 10.0)
    # Animate drones
    animate_drones(simulator, paths, dynamic_obstacles, no_fly_zones, map_ui=map_ui, end_world=end_world, planner=planner, obstacles=obstacles, buildings=buildings)
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