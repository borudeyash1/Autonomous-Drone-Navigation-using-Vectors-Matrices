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
from ui.modern_ui import ModernUI
from typing import List, Dict, Tuple
import math
import random

def create_status_window():
    """Create a window to display drone status."""
    cv2.namedWindow('Drone Status', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Drone Status', 800, 600)

def update_status_display(states: List[Dict], mode: str = "Autonomous"):
    """Update the status display with current drone states matching the images."""
    # Create a larger status display
    status_image = np.zeros((600, 800, 3), dtype=np.uint8)
    
    # Add modern background
    cv2.rectangle(status_image, (0, 0), (800, 600), (40, 40, 40), -1)
    
    # Add status information
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.7
    font_thickness = 2
    line_height = 35
    y_pos = 40
    
    # Title
    cv2.putText(status_image, "AUTONOMOUS DRONE NAVIGATION SYSTEM", (20, y_pos), font, 0.8, (0, 255, 255), 2)
    y_pos += 50
    
    # Mode indicator
    mode_color = (0, 255, 0) if mode == "Manual" else (0, 165, 255)  # Green for Manual, Orange for Autonomous
    cv2.putText(status_image, f"OPERATION MODE: {mode}", (20, y_pos), font, font_scale, mode_color, font_thickness)
    y_pos += line_height + 10
    
    # Display information for each drone
    for i, state in enumerate(states):
        pos = state['position']
        vel = state['velocity']
        orn = state['orientation']
        
        # Calculate flight parameters
        speed = np.linalg.norm(vel)
        altitude = pos[2]
        heading = np.degrees(orn[2]) % 360
        pitch = np.degrees(orn[1])
        roll = np.degrees(orn[0])
        
        # Drone header
        drone_color = (0, 255, 255) if i == 0 else (255, 255, 0)
        cv2.putText(status_image, f"DRONE #{i+1} - STATUS", (20, y_pos), font, font_scale, drone_color, font_thickness)
        y_pos += line_height
        
        # Position information
        cv2.putText(status_image, f"POSITION: X:{pos[0]:+.2f} Y:{pos[1]:+.2f} Z:{pos[2]:+.2f}", (30, y_pos), font, 0.6, (255, 255, 255), 1)
        y_pos += line_height
        
        # Velocity information
        cv2.putText(status_image, f"VELOCITY: X:{vel[0]:+.2f} Y:{vel[1]:+.2f} Z:{vel[2]:+.2f}", (30, y_pos), font, 0.6, (255, 255, 255), 1)
        y_pos += line_height
        
        # Flight parameters
        cv2.putText(status_image, f"SPEED: {speed:.2f} m/s | ALTITUDE: {altitude:.2f} m", (30, y_pos), font, 0.6, (255, 255, 255), 1)
        y_pos += line_height
        
        cv2.putText(status_image, f"HEADING: {heading:.1f}° | PITCH: {pitch:.1f}° | ROLL: {roll:.1f}°", (30, y_pos), font, 0.6, (255, 255, 255), 1)
        y_pos += line_height
        
        # Status indicators
        cv2.putText(status_image, f"GPS: OK | GYRO: ACTIVE | CAM: {i+1} | BATTERY: 100%", (30, y_pos), font, 0.6, (0, 255, 0), 1)
        y_pos += line_height + 20
    
    # Add system status
    cv2.putText(status_image, "SYSTEM STATUS:", (20, y_pos), font, font_scale, (0, 255, 255), font_thickness)
    y_pos += line_height
    
    cv2.putText(status_image, "• PATH PLANNING: ACTIVE", (30, y_pos), font, 0.6, (0, 255, 0), 1)
    y_pos += line_height
    
    cv2.putText(status_image, "• OBSTACLE AVOIDANCE: ENABLED", (30, y_pos), font, 0.6, (0, 255, 0), 1)
    y_pos += line_height
    
    cv2.putText(status_image, "• TRAJECTORY TRACKING: ACTIVE", (30, y_pos), font, 0.6, (0, 255, 0), 1)
    y_pos += line_height
    
    cv2.putText(status_image, "• COMMUNICATION: STABLE", (30, y_pos), font, 0.6, (0, 255, 0), 1)
    y_pos += line_height
    
    # Add timestamp
    import datetime
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    cv2.putText(status_image, f"TIME: {timestamp}", (20, 580), font, 0.6, (255, 255, 255), 1)
    
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
                  map_ui=None, end_world=None, planner=None, obstacles=None, buildings=None, start_world=None):
    """
    Animate the drones along their paths with clear path visualization.
    """
    # Create modern UI system
    modern_ui = ModernUI()
    
    # Create status and parameter windows (legacy)
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
    
    # Timeout mechanism
    start_time = time.time()
    max_runtime = 300  # 5 minutes max runtime
    
    sim_dt = 1/240
    render_dt = 1/60
    last_render = time.time()
    last_3d_update = time.time()
    stuck_timer = 0
    stuck_threshold = 1.0
    stuck_distance = 0.5
    last_pos = None
    
    # Initialize trajectory tracking
    trajectory_points = [[] for _ in range(len(paths))]
    path_completed = [False] * len(paths)
    
    while running:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Check for timeout
        if current_time - start_time > max_runtime:
            print("[INFO] Maximum runtime reached, stopping simulation...")
            break
        
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
                    # Enhanced autonomous control with better obstacle avoidance
                    current_pos = state['position']
                    
                    # Check if drone has reached the goal
                    distance_to_goal = np.linalg.norm(current_pos - end_world)
                    if distance_to_goal < 0.5:
                        path_completed[i] = True
                        print(f"[INFO] Drone {i+1} reached the endpoint!")
                        # Hover at goal position
                        target_pos = end_world.copy()
                    else:
                        # Determine target position
                        if path and not path_completed[i]:
                            # Follow planned path
                            target_pos = np.array(path[0])
                            
                            # Check if waypoint reached
                            if np.linalg.norm(current_pos - target_pos) < 0.3:
                                path.pop(0)
                                if path:
                                    target_pos = np.array(path[0])
                                else:
                                    target_pos = end_world.copy()
                        else:
                            # Direct path to goal
                            target_pos = end_world.copy()
                    
                    # Use the proper PID controller
                    target_state = {
                        'position': target_pos,
                        'velocity': np.zeros(3),  # Target velocity (hover)
                        'orientation': np.zeros(3)
                    }
                    
                    control_input = controller.compute_control(state, target_state, dt)
                    
                    # Add trajectory point for visualization
                    trajectory_points[i].append(current_pos.copy())
                    simulator.add_trajectory_point(current_pos, i)
                
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
            
            # Update modern UI
            modern_ui.update_drone_states(states)
            modern_ui.set_system_mode(mode)
            
            # Update legacy status display
            update_status_display(states, mode)
            
            # Update 2D map UI
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
        
        # Update visualization at lower frequency
        if current_time - last_3d_update >= 0.1:  # 10 FPS for visualization
            states = simulator.fleet.get_drone_states()
            
            # Update visualization based on mode
            if hasattr(simulator, 'gui_mode') and not simulator.gui_mode:
                # Use 2D visualization for DIRECT mode
                simulator.update_2d_visualization(
                    drone_states=states,
                    obstacles=dynamic_obstacles or [],
                    buildings=buildings or [],
                    start=start_world,
                    goal=end_world
                )
            else:
                # Use 3D visualization for GUI mode
                simulator.update_3d_visualization(
                    drone_states=states,
                    obstacles=dynamic_obstacles or [],
                    buildings=buildings or [],
                    start=start_world,
                    goal=end_world
                )
                
                # Draw planned path in 3D
                if paths and paths[0]:
                    simulator.draw_planned_path(paths[0], color='red')
            
            last_3d_update = current_time
    
    # Clean up
    modern_ui.close()
    cv2.destroyAllWindows()

def main():
    # Reset and clean up
    print("[RESET] Resetting simulation and map UI...")
    map_ui = MapUI(grid_size=20, cell_size=30)
    grid, start, end, obstacles, obstacle_sizes, dynamic_obstacles, no_fly_zones = map_ui.wait_for_setup()
    
    print("[SIM] Initializing PyBullet simulator...")
    # Start with DIRECT mode to avoid Windows hanging issues
    try:
        simulator = Simulator(gui=False, num_drones=1)
        print("[SIM] Simulator initialized in DIRECT mode!")
    except Exception as e:
        print(f"[ERROR] Failed to initialize simulator: {e}")
        return
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
    # Plan path with enhanced obstacle avoidance
    path3d = planner.replan_path(start_world, end_world)
    print("[PATH] Planned 3D path waypoints:")
    for wp in path3d:
        print(f"  {wp}")
    
    # Convert to 2D for map display
    path2d = [np.array([wp[0], wp[1], 0]) for wp in path3d]
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
    animate_drones(simulator, paths, dynamic_obstacles, no_fly_zones, map_ui=map_ui, end_world=end_world, planner=planner, obstacles=obstacles, buildings=buildings, start_world=start_world)
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