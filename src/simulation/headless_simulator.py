import numpy as np
import cv2
import time
from typing import Tuple, Optional, List, Dict
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class HeadlessSimulator:
    def __init__(self, num_drones: int = 1):
        """
        Initialize a headless simulator without PyBullet.
        
        Args:
            num_drones: Number of drones to create (default: 1)
        """
        print("[SIM] Initializing headless simulator...")
        
        # Drone states
        self.drone_states = []
        for i in range(num_drones):
            self.drone_states.append({
                'position': np.array([0.0, 0.0, 1.0]),
                'velocity': np.array([0.0, 0.0, 0.0]),
                'orientation': np.array([0.0, 0.0, 0.0])
            })
        
        # Camera parameters
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fov = 90
        self.camera_aspect = self.camera_width / self.camera_height
        self.camera_near = 0.1
        self.camera_far = 100.0
        
        # Trajectory visualization
        self.trajectory_lines = []
        self.planned_path_lines = []
        self.goal_marker = None
        self.start_marker = None
        
        # Create OpenCV windows
        cv2.namedWindow('FPP View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('FPP View', self.camera_width, self.camera_height)
        cv2.namedWindow('Drone Status', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Drone Status', 400, 300)
        
        # Create 2D top-down view
        cv2.namedWindow('2D Simulation View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('2D Simulation View', 600, 600)
        
        # Create matplotlib figure for 3D visualization
        self.fig_3d = None
        self.ax_3d = None
        self.setup_3d_visualization()
        
        # Camera control parameters
        self.camera_yaw = 0.0
        self.camera_pitch = 0.0
        self.fpp_mode = True
        
        print("[SIM] Headless simulator initialized successfully!")
    
    def setup_3d_visualization(self):
        """Setup matplotlib 3D visualization."""
        try:
            plt.ion()  # Turn on interactive mode
            self.fig_3d = plt.figure(figsize=(12, 8))
            self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
            self.ax_3d.set_xlabel('X')
            self.ax_3d.set_ylabel('Y')
            self.ax_3d.set_zlabel('Z')
            self.ax_3d.set_title('3D Drone Trajectory Visualization')
            self.ax_3d.grid(True)
            self.ax_3d.set_xlim([-10, 10])
            self.ax_3d.set_ylim([-10, 10])
            self.ax_3d.set_zlim([0, 5])
        except Exception as e:
            print(f"Warning: Could not setup 3D visualization: {e}")
            self.fig_3d = None
            self.ax_3d = None
    
    def add_trajectory_point(self, position: np.ndarray, drone_id: int = 0):
        """Add a point to the drone's trajectory."""
        if drone_id >= len(self.trajectory_lines):
            self.trajectory_lines.append([])
        
        self.trajectory_lines[drone_id].append(position.copy())
    
    def draw_planned_path(self, path: List[np.ndarray], color: str = 'red'):
        """Draw the planned path in 3D."""
        if not path or len(path) < 2 or self.ax_3d is None:
            return
        
        # Convert path to arrays
        x_coords = [p[0] for p in path]
        y_coords = [p[1] for p in path]
        z_coords = [p[2] for p in path]
        
        # Draw path line
        self.ax_3d.plot(x_coords, y_coords, z_coords, color=color, linewidth=3, alpha=0.8, label='Planned Path')
        
        # Draw waypoints
        self.ax_3d.scatter(x_coords, y_coords, z_coords, color=color, s=50, alpha=0.8)
    
    def update_2d_visualization(self, drone_states: List[Dict], obstacles: List[Dict] = None, buildings: List[Tuple] = None, start: np.ndarray = None, goal: np.ndarray = None):
        """Update the 2D top-down visualization."""
        # Create 2D visualization
        img = np.ones((600, 600, 3), dtype=np.uint8) * 50  # Dark background
        
        # Draw grid
        for i in range(0, 600, 30):
            cv2.line(img, (i, 0), (i, 600), (100, 100, 100), 1)
            cv2.line(img, (0, i), (600, i), (100, 100, 100), 1)
        
        # Draw obstacles (buildings)
        if buildings:
            for building in buildings:
                x, y, z, w, l, h = building
                # Convert world coordinates to image coordinates
                img_x = int((x + 5) / 10 * 600)  # Map from [-5,5] to [0,600]
                img_y = int((y + 5) / 10 * 600)
                img_w = int(w / 10 * 600)
                img_l = int(l / 10 * 600)
                
                # Draw building as rectangle
                cv2.rectangle(img, (img_x - img_w//2, img_y - img_l//2), 
                            (img_x + img_w//2, img_y + img_l//2), (0, 0, 255), -1)
        
        # Draw start and goal
        if start is not None:
            start_x = int((start[0] + 5) / 10 * 600)
            start_y = int((start[1] + 5) / 10 * 600)
            cv2.circle(img, (start_x, start_y), 15, (0, 255, 0), -1)
            cv2.putText(img, "START", (start_x - 20, start_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if goal is not None:
            goal_x = int((goal[0] + 5) / 10 * 600)
            goal_y = int((goal[1] + 5) / 10 * 600)
            cv2.circle(img, (goal_x, goal_y), 15, (0, 0, 255), -1)
            cv2.putText(img, "GOAL", (goal_x - 15, goal_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw drone positions
        for i, state in enumerate(drone_states):
            pos = state['position']
            drone_x = int((pos[0] + 5) / 10 * 600)
            drone_y = int((pos[1] + 5) / 10 * 600)
            
            colors = [(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 255), (0, 165, 255)]
            color = colors[i % len(colors)]
            
            # Draw drone as triangle
            pts = np.array([[drone_x, drone_y - 10], [drone_x - 8, drone_y + 8], [drone_x + 8, drone_y + 8]], np.int32)
            cv2.fillPoly(img, [pts], color)
            cv2.putText(img, f"Drone {i+1}", (drone_x - 20, drone_y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Draw trajectories
        for i, trajectory in enumerate(self.trajectory_lines):
            if len(trajectory) > 1:
                colors = [(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 255), (0, 165, 255)]
                color = colors[i % len(colors)]
                
                for j in range(1, len(trajectory)):
                    pt1 = trajectory[j-1]
                    pt2 = trajectory[j]
                    x1 = int((pt1[0] + 5) / 10 * 600)
                    y1 = int((pt1[1] + 5) / 10 * 600)
                    x2 = int((pt2[0] + 5) / 10 * 600)
                    y2 = int((pt2[1] + 5) / 10 * 600)
                    cv2.line(img, (x1, y1), (x2, y2), color, 2)
        
        # Add title
        cv2.putText(img, "2D Drone Simulation (Top-Down View)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Display the image
        cv2.imshow('2D Simulation View', img)
        cv2.waitKey(1)
    
    def update_3d_visualization(self, drone_states: List[Dict], obstacles: List[Dict] = None, buildings: List[Tuple] = None, start: np.ndarray = None, goal: np.ndarray = None):
        """Update the 3D visualization."""
        if self.ax_3d is None:
            return
        
        # Clear previous plot
        self.ax_3d.clear()
        
        # Set up the plot
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_title('3D Drone Trajectory Visualization')
        self.ax_3d.grid(True)
        self.ax_3d.set_xlim([-10, 10])
        self.ax_3d.set_ylim([-10, 10])
        self.ax_3d.set_zlim([0, 5])
        
        # Draw obstacles
        if buildings:
            for building in buildings:
                x, y, z, w, l, h = building
                # Create a box for the building
                self.ax_3d.bar3d(x-w/2, y-l/2, 0, w, l, h, alpha=0.7, color='black')
        
        # Draw start and goal
        if start is not None and goal is not None:
            # Start marker (green circle)
            self.ax_3d.scatter([start[0]], [start[1]], [start[2]], color='green', s=200, marker='o', label='Start')
            # Goal marker (red star)
            self.ax_3d.scatter([goal[0]], [goal[1]], [goal[2]], color='red', s=200, marker='*', label='Goal')
        
        # Draw trajectories for all drones
        for i in range(len(self.trajectory_lines)):
            if i < len(self.trajectory_lines) and len(self.trajectory_lines[i]) > 1:
                trajectory = self.trajectory_lines[i]
                x_coords = [p[0] for p in trajectory]
                y_coords = [p[1] for p in trajectory]
                z_coords = [p[2] for p in trajectory]
                
                colors = ['red', 'blue', 'green', 'yellow', 'purple']
                color = colors[i % len(colors)]
                self.ax_3d.plot(x_coords, y_coords, z_coords, color=color, linewidth=2, alpha=0.6, linestyle='--', label=f'Drone {i+1} Trajectory')
        
        # Draw current drone positions
        for i, state in enumerate(drone_states):
            pos = state['position']
            colors = ['red', 'blue', 'green', 'yellow', 'purple']
            color = colors[i % len(colors)]
            self.ax_3d.scatter([pos[0]], [pos[1]], [pos[2]], color=color, s=100, marker='^', label=f'Drone {i+1}')
        
        # Add legend
        self.ax_3d.legend()
        
        # Update the plot
        plt.draw()
        plt.pause(0.01)
    
    def step(self, control_inputs: List[np.ndarray], mode: str = "Autonomous") -> Tuple[np.ndarray, np.ndarray]:
        """Step simulation and update visualization."""
        try:
            # Update drone states based on control inputs
            for i, (state, control) in enumerate(zip(self.drone_states, control_inputs)):
                # Simple physics simulation
                dt = 1/240  # 240 Hz simulation
                
                # Extract control inputs
                thrust, roll, pitch, yaw = control
                
                # Update position based on velocity
                state['position'] += state['velocity'] * dt
                
                # Update velocity based on thrust and orientation
                # Convert thrust to acceleration
                acceleration = np.array([
                    np.sin(yaw) * np.cos(pitch) * thrust,
                    np.cos(yaw) * np.cos(pitch) * thrust,
                    np.sin(pitch) * thrust - 9.81  # Gravity
                ])
                
                state['velocity'] += acceleration * dt
                
                # Update orientation
                state['orientation'] += np.array([roll, pitch, yaw]) * dt
                
                # Add some damping
                state['velocity'] *= 0.99
                
                # Keep drone above ground
                if state['position'][2] < 0.1:
                    state['position'][2] = 0.1
                    state['velocity'][2] = max(0, state['velocity'][2])
            
            # Get camera images (simulated)
            rgb_image, depth_image = self._get_camera_images()
            
            # Add HUD overlay to RGB image
            rgb_image = self._add_hud_overlay(rgb_image)
            
            # Update displays
            cv2.imshow('FPP View', rgb_image)
            self.update_status_display(mode)
            cv2.waitKey(1)
            
            return rgb_image, depth_image
        except Exception as e:
            print(f"Error in simulation step: {e}")
            return np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8), np.zeros((self.camera_height, self.camera_width))
    
    def _get_camera_images(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get simulated camera images."""
        # Create a simple simulated camera view
        rgb_image = np.ones((self.camera_height, self.camera_width, 3), dtype=np.uint8) * 100
        
        # Add some visual elements
        cv2.circle(rgb_image, (self.camera_width//2, self.camera_height//2), 50, (0, 255, 0), 2)
        cv2.putText(rgb_image, "Simulated FPV View", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        depth_image = np.ones((self.camera_height, self.camera_width), dtype=np.float32) * 10.0
        
        return rgb_image, depth_image
    
    def _add_hud_overlay(self, image: np.ndarray) -> np.ndarray:
        """Add HUD overlay to camera view."""
        overlay = image.copy()
        h, w = overlay.shape[:2]
        
        # Create HUD background
        bar_height = 80
        cv2.rectangle(overlay, (0, 0), (w, bar_height), (20, 20, 20), -1)
        alpha_bar = 0.8
        image = cv2.addWeighted(overlay, alpha_bar, image, 1 - alpha_bar, 0)
        
        # Draw crosshair
        cx, cy = w // 2, h // 2
        cv2.drawMarker(image, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=30, thickness=3)
        
        # Add state information
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 0.7
        font_thickness = 2
        
        for i, state in enumerate(self.drone_states):
            pos = state['position']
            vel = state['velocity']
            orn = state['orientation']
            
            speed = np.linalg.norm(vel)
            altitude = pos[2]
            heading = np.degrees(orn[2]) % 360
            pitch = np.degrees(orn[1])
            
            y0 = 25 + i*35
            cv2.putText(image, f"DRONE #{i+1} - FPV CAMERA", (10, y0), font, font_scale, (255, 255, 0), font_thickness)
            
            pos_text = f"POS X:{pos[0]:+.2f} Y:{pos[1]:+.2f} Z:{pos[2]:+.2f}"
            cv2.putText(image, pos_text, (10, y0+20), font, 0.6, (0, 255, 255), 1)
            vel_text = f"VEL: X:{vel[0]:+.2f} Y:{vel[1]:+.2f} Z:{vel[2]:+.2f}"
            cv2.putText(image, vel_text, (10, y0+35), font, 0.6, (0, 255, 255), 1)
        
        return image
    
    def update_status_display(self, mode: str = "Autonomous"):
        """Update the status display window."""
        states = self.drone_states
        text = "Drone States:\n"
        
        for i, state in enumerate(states):
            pos = state['position']
            vel = state['velocity']
            text += f"Drone {i}:\n"
            text += f"  Position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]\n"
            text += f"  Velocity: [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}]\n"
        
        # Create status image
        status_image = np.zeros((300, 400, 3), dtype=np.uint8)
        cv2.putText(status_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow('Drone Status', status_image)
        cv2.waitKey(1)
    
    def get_drone_states(self) -> List[Dict]:
        """Get current drone states."""
        return self.drone_states
    
    def set_drone_position(self, drone_idx: int, position: np.ndarray, orientation: np.ndarray = None):
        """Set drone position."""
        if 0 <= drone_idx < len(self.drone_states):
            self.drone_states[drone_idx]['position'] = position.copy()
            if orientation is not None:
                self.drone_states[drone_idx]['orientation'] = orientation.copy()
    
    def close(self):
        """Clean up resources."""
        try:
            cv2.destroyAllWindows()
            if self.fig_3d is not None:
                plt.close(self.fig_3d)
        except Exception as e:
            print(f"Error during cleanup: {e}")
