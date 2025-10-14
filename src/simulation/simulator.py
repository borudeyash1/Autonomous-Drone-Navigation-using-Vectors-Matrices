import pybullet as p
import pybullet_data
import numpy as np
import time
from typing import Tuple, Optional, List, Dict
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from .drone_fleet import DroneFleet

class Simulator:
    def __init__(self, gui: bool = True, num_drones: int = 1):  # Default to GUI mode for better visualization
        """
        Initialize the PyBullet simulation environment.
        
        Args:
            gui: Whether to show the GUI (default: True)
            num_drones: Number of drones to create (default: 1)
        """
        # Connect to physics server - start with DIRECT mode for reliability
        print("[SIM] Connecting to PyBullet...")
        try:
            if gui:
                print("[SIM] Attempting GUI mode...")
                self.client = p.connect(p.GUI)
                print("[SIM] PyBullet GUI connected successfully!")
            else:
                print("[SIM] Using DIRECT mode...")
                self.client = p.connect(p.DIRECT)
                print("[SIM] PyBullet DIRECT connected successfully!")
        except Exception as e:
            print(f"[ERROR] Failed to connect to PyBullet GUI: {e}")
            print("[INFO] Falling back to DIRECT mode...")
            self.client = p.connect(p.DIRECT)
            gui = False  # Disable GUI features
            print("[SIM] PyBullet DIRECT connected successfully!")
            
            # Configure GUI appearance for modern look (only if GUI is enabled)
            if gui:
                p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
                p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
                p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
                p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
                p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
                p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            
            # Set camera parameters for external view (only if GUI is enabled)
            if gui:
                p.resetDebugVisualizerCamera(
                    cameraDistance=15.0,
                    cameraYaw=45,
                    cameraPitch=-30,
                    cameraTargetPosition=[0, 0, 2]
                )
                
                # Set rendering options
                p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
                p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
            
            # Set background color to a modern dark theme
            print("[SIM] Setting up physics engine...")
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setPhysicsEngineParameter(numSolverIterations=10)
            p.resetSimulation()  # Remove the deformable world parameter
            p.setGravity(0, 0, -9.81)
            
            # Configure ground plane appearance with enhanced materials
            print("[SIM] Loading ground plane...")
            try:
                self.plane_id = p.loadURDF("plane.urdf")
                if gui:
                    p.changeVisualShape(
                        self.plane_id, 
                        -1, 
                        rgbaColor=[0.3, 0.3, 0.3, 1],  # Dark gray ground
                        specularColor=[0.4, 0.4, 0.4]
                    )
                print("[SIM] Ground plane loaded successfully!")
            except Exception as e:
                print(f"[ERROR] Failed to load ground plane: {e}")
                self.plane_id = None
        else:
            self.client = p.connect(p.DIRECT)
        
        # Set simulation parameters
        p.setTimeStep(1/240)  # 240 Hz simulation
        p.setRealTimeSimulation(0)  # Step simulation manually
        
        # Set gravity
        p.setGravity(0, 0, -9.81)
        
        # Load ground plane
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Initialize drone fleet
        print("[SIM] Initializing drone fleet...")
        try:
            self.fleet = DroneFleet(self.client, num_drones)
            print("[SIM] Drone fleet initialized successfully!")
        except Exception as e:
            print(f"[ERROR] Failed to initialize drone fleet: {e}")
            raise
        
        # Camera parameters
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fov = 90
        self.camera_aspect = self.camera_width / self.camera_height
        self.camera_near = 0.1
        self.camera_far = 100.0
        
        # Camera matrix for obstacle detection
        self.camera_matrix = np.array([
            [self.camera_width/2, 0, self.camera_width/2],
            [0, self.camera_height/2, self.camera_height/2],
            [0, 0, 1]
        ])
        
        # List of obstacle IDs
        self.obstacle_ids = []
        
        # Trajectory visualization
        self.trajectory_lines = []
        self.planned_path_lines = []
        self.goal_marker = None
        self.start_marker = None
        
        # Create OpenCV windows
        cv2.namedWindow('FPP View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('FPP View', self.camera_width, self.camera_height)
        cv2.setMouseCallback('FPP View', self._mouse_event)
        cv2.namedWindow('Drone Status', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Drone Status', 400, 300)
        
        # Create 2D top-down view for DIRECT mode
        self.gui_mode = gui
        if not gui:
            cv2.namedWindow('2D Simulation View', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('2D Simulation View', 600, 600)
        
        # Create matplotlib figure for 3D visualization
        self.fig_3d = None
        self.ax_3d = None
        self.setup_3d_visualization()
        
        # Additional camera control parameters
        self.camera_yaw = 0.0
        self.camera_pitch = 0.0
        self._mouse_down = False
        self._last_mouse_x = 0
        self._last_mouse_y = 0
        
        # Create status display
        self.status_display = p.addUserDebugText(
            "Initializing...",
            [0, 0, 2],
            textColorRGB=[1, 1, 1],
            textSize=1.5
        )
        
        self.fpp_mode = True  # True: FPP, False: Free View
        self.free_cam_pos = np.array([0.0, -5.0, 2.0])
        self.free_cam_yaw = 0.0
        self.free_cam_pitch = -10.0
        self._free_mouse_down = False
        self._last_free_mouse_x = 0
        self._last_free_mouse_y = 0
    
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
            # Create new trajectory line for this drone
            colors = ['red', 'blue', 'green', 'yellow', 'purple']
            color = colors[drone_id % len(colors)]
            self.trajectory_lines.append([])
        
        self.trajectory_lines[drone_id].append(position.copy())
    
    def draw_planned_path(self, path: List[np.ndarray], color: str = 'red'):
        """Draw the planned path in 3D."""
        if not path or len(path) < 2:
            return
        
        # Convert path to arrays
        x_coords = [p[0] for p in path]
        y_coords = [p[1] for p in path]
        z_coords = [p[2] for p in path]
        
        # Draw path line
        self.ax_3d.plot(x_coords, y_coords, z_coords, color=color, linewidth=3, alpha=0.8, label='Planned Path')
        
        # Draw waypoints
        self.ax_3d.scatter(x_coords, y_coords, z_coords, color=color, s=50, alpha=0.8)
    
    def draw_actual_trajectory(self, drone_id: int = 0):
        """Draw the actual trajectory of a drone."""
        if drone_id >= len(self.trajectory_lines) or not self.trajectory_lines[drone_id]:
            return
        
        trajectory = self.trajectory_lines[drone_id]
        if len(trajectory) < 2:
            return
        
        # Convert trajectory to arrays
        x_coords = [p[0] for p in trajectory]
        y_coords = [p[1] for p in trajectory]
        z_coords = [p[2] for p in trajectory]
        
        # Draw trajectory line
        colors = ['red', 'blue', 'green', 'yellow', 'purple']
        color = colors[drone_id % len(colors)]
        self.ax_3d.plot(x_coords, y_coords, z_coords, color=color, linewidth=2, alpha=0.6, linestyle='--', label=f'Drone {drone_id+1} Trajectory')
    
    def draw_obstacles_3d(self, obstacles: List[Dict], buildings: List[Tuple]):
        """Draw obstacles in 3D visualization."""
        # Draw static obstacles (buildings)
        for building in buildings:
            x, y, z, w, l, h = building
            # Create a box for the building
            self.ax_3d.bar3d(x-w/2, y-l/2, 0, w, l, h, alpha=0.7, color='black')
        
        # Draw dynamic obstacles
        for obs in obstacles:
            # Handle both dict and tuple formats
            if isinstance(obs, dict):
                pos = obs.get('position', [0, 0, 0])
                radius = obs.get('radius', 0.5)
            else:
                # Assume it's a tuple with position
                pos = obs if len(obs) >= 3 else [0, 0, 0]
                radius = 0.5
            
            # Create a sphere for dynamic obstacle
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            x_sphere = pos[0] + radius * np.outer(np.cos(u), np.sin(v))
            y_sphere = pos[1] + radius * np.outer(np.sin(u), np.sin(v))
            z_sphere = pos[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
            self.ax_3d.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.6, color='blue')
    
    def draw_start_goal_markers(self, start: np.ndarray, goal: np.ndarray):
        """Draw start and goal markers."""
        # Start marker (green circle)
        self.ax_3d.scatter([start[0]], [start[1]], [start[2]], color='green', s=200, marker='o', label='Start')
        
        # Goal marker (red star)
        self.ax_3d.scatter([goal[0]], [goal[1]], [goal[2]], color='red', s=200, marker='*', label='Goal')
    
    def update_2d_visualization(self, drone_states: List[Dict], obstacles: List[Dict] = None, buildings: List[Tuple] = None, start: np.ndarray = None, goal: np.ndarray = None):
        """Update the 2D top-down visualization for DIRECT mode."""
        if self.gui_mode:
            return  # Only for DIRECT mode
        
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
        if obstacles:
            self.draw_obstacles_3d(obstacles, buildings or [])
        
        # Draw start and goal
        if start is not None and goal is not None:
            self.draw_start_goal_markers(start, goal)
        
        # Draw trajectories for all drones
        for i in range(len(self.trajectory_lines)):
            self.draw_actual_trajectory(i)
        
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
    
    def _mouse_event(self, event, x, y, flags, param):
        if self.fpp_mode:
            if event == cv2.EVENT_LBUTTONDOWN:
                self._mouse_down = True
                self._last_mouse_x = x
                self._last_mouse_y = y
            elif event == cv2.EVENT_LBUTTONUP:
                self._mouse_down = False
            elif event == cv2.EVENT_MOUSEMOVE and self._mouse_down:
                dx = x - self._last_mouse_x
                dy = y - self._last_mouse_y
                self._last_mouse_x = x
                self._last_mouse_y = y
                self.camera_yaw += dx * 0.3  # Sensitivity
                self.camera_pitch += dy * 0.3
                self.camera_pitch = np.clip(self.camera_pitch, -89, 89)
        else:
            if event == cv2.EVENT_RBUTTONDOWN:
                self._free_mouse_down = True
                self._last_free_mouse_x = x
                self._last_free_mouse_y = y
            elif event == cv2.EVENT_RBUTTONUP:
                self._free_mouse_down = False
            elif event == cv2.EVENT_MOUSEMOVE and self._free_mouse_down:
                dx = x - self._last_free_mouse_x
                dy = y - self._last_free_mouse_y
                self._last_free_mouse_x = x
                self._last_free_mouse_y = y
                self.free_cam_yaw += dx * 0.3
                self.free_cam_pitch += dy * 0.3
                self.free_cam_pitch = np.clip(self.free_cam_pitch, -89, 89)
    
    def toggle_camera_mode(self):
        self.fpp_mode = not self.fpp_mode
        if self.fpp_mode:
            cv2.setMouseCallback('FPP View', self._mouse_event)
        else:
            cv2.setMouseCallback('FPP View', self._mouse_event)
    
    def move_free_camera(self, key, dt):
        """Move the free camera based on keyboard input."""
        move_speed = 10.0  # Increased from 5.0
        rotate_speed = 2.0  # Increased from 1.0
        
        if key == ord('w'):
            self.free_cam_pos += np.array([0, move_speed * dt, 0])
        elif key == ord('s'):
            self.free_cam_pos += np.array([0, -move_speed * dt, 0])
        elif key == ord('a'):
            self.free_cam_pos += np.array([-move_speed * dt, 0, 0])
        elif key == ord('d'):
            self.free_cam_pos += np.array([move_speed * dt, 0, 0])
        elif key == 2490368:  # Up arrow
            self.free_cam_pos += np.array([0, 0, move_speed * dt])
        elif key == 2621440:  # Down arrow
            self.free_cam_pos += np.array([0, 0, -move_speed * dt])
        elif key == 2424832:  # Left arrow
            self.free_cam_yaw += rotate_speed * dt
        elif key == 2555904:  # Right arrow
            self.free_cam_yaw -= rotate_speed * dt
            
        # Update camera view
        self._update_camera_view()
        
    def _update_camera_view(self):
        """Update the camera view based on current mode and position."""
        if self.fpp_mode:
            # Use drone's position and orientation for FPP view
            drone_state = self.fleet.get_drone_states()[0]
            pos = drone_state['position']
            orn = drone_state['orientation']
            yaw, pitch, roll = p.getEulerFromQuaternion(orn)
            
            # Set camera position slightly in front of drone
            cam_pos = pos + np.array([
                -np.sin(yaw) * 0.5,
                np.cos(yaw) * 0.5,
                0.1
            ])
            
            # Set camera target in front of drone
            target_pos = pos + np.array([
                -np.sin(yaw) * 2.0,
                np.cos(yaw) * 2.0,
                0.0
            ])
            
            p.resetDebugVisualizerCamera(
                cameraDistance=0.1,
                cameraYaw=np.degrees(yaw),
                cameraPitch=np.degrees(pitch),
                cameraTargetPosition=target_pos
            )
        else:
            # Use free camera position and orientation
            p.resetDebugVisualizerCamera(
                cameraDistance=0.1,
                cameraYaw=np.degrees(self.free_cam_yaw),
                cameraPitch=np.degrees(self.free_cam_pitch),
                cameraTargetPosition=self.free_cam_pos
            )
    
    def add_obstacle(self, position: np.ndarray, size: np.ndarray, 
                    obstacle_type: str = "box", color: Tuple[float, float, float, float] = (0.5, 0.5, 0.5, 1)):
        """Add an obstacle with modern appearance."""
        if obstacle_type == "box":
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=size/2,
                rgbaColor=color,
                specularColor=[0.4, 0.4, 0.4],
                visualFramePosition=[0, 0, 0]
            )
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=size/2
            )
        elif obstacle_type == "cylinder":
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=size[0]/2,
                length=size[2],
                rgbaColor=color,
                specularColor=[0.4, 0.4, 0.4],
                visualFramePosition=[0, 0, 0]
            )
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=size[0]/2,
                height=size[2]
            )
        else:
            raise ValueError(f"Unknown obstacle type: {obstacle_type}")
        
        # Create the obstacle with modern materials
        obstacle_id = p.createMultiBody(
            baseMass=0,  # Static obstacle
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position,
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        self.obstacle_ids.append(obstacle_id)
        return obstacle_id
    
    def add_random_obstacles(self, num_obstacles: int, min_size: float = 0.5, 
                           max_size: float = 2.0, min_height: float = 1.0,
                           max_height: float = 5.0, area_size: float = 20.0):
        """Add random obstacles with modern color scheme."""
        # Modern color palette
        colors = [
            (0.2, 0.6, 0.8, 1.0),  # Blue
            (0.8, 0.4, 0.2, 1.0),  # Orange
            (0.2, 0.8, 0.4, 1.0),  # Green
            (0.8, 0.2, 0.6, 1.0),  # Purple
            (0.6, 0.8, 0.2, 1.0),  # Yellow-green
        ]
        
        for i in range(num_obstacles):
            # Random position
            x = np.random.uniform(-area_size/2, area_size/2)
            y = np.random.uniform(-area_size/2, area_size/2)
            z = np.random.uniform(min_height/2, max_height/2)
            position = np.array([x, y, z])
            
            # Random size
            width = np.random.uniform(min_size, max_size)
            length = np.random.uniform(min_size, max_size)
            height = np.random.uniform(min_height, max_height)
            size = np.array([width, length, height])
            
            # Use modern color palette
            color = colors[i % len(colors)]
            
            # Add obstacle
            self.add_obstacle(position, size, "box", color)
    
    def step(self, control_inputs: List[np.ndarray], mode: str = "Autonomous") -> Tuple[np.ndarray, np.ndarray]:
        """Step simulation and update visualization."""
        try:
            # Step the drone fleet
            self.fleet.step(1/240)
            
            # Step simulation
            p.stepSimulation()
            
            # Get camera images
            rgb_image, depth_image = self._get_camera_images()
            
            # Debug: Print mean pixel value
            print(f"[DEBUG] Camera mean pixel value: {np.mean(rgb_image):.2f}")
            
            # If the image is all black, show a test image for debugging
            if np.mean(rgb_image) < 1.0:
                print("[DEBUG] Camera image is all black. Displaying test image.")
                rgb_image = np.ones((self.camera_height, self.camera_width, 3), dtype=np.uint8) * 255
            
            # Add HUD overlay to RGB image
            rgb_image = self._add_hud_overlay(rgb_image)
            
            # Update displays
            if hasattr(self, 'client') and p.isConnected(self.client):
                cv2.imshow('FPP View', rgb_image)
                self.update_status_display(mode)
                cv2.waitKey(1)
            
            # For DIRECT mode, also update 2D visualization
            if not self.gui_mode:
                states = self.fleet.get_drone_states()
                self.update_2d_visualization(states)
            
            return rgb_image, depth_image
        except Exception as e:
            print(f"Error in simulation step: {e}")
            return np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8), np.zeros((self.camera_height, self.camera_width))
    
    def _add_hud_overlay(self, image: np.ndarray) -> np.ndarray:
        """Add modern HUD overlay to camera view matching the images."""
        # Convert image to uint8 if needed
        if image.dtype != np.uint8:
            image = (image * 255).astype(np.uint8)
        
        overlay = image.copy()
        h, w = overlay.shape[:2]
        
        # Create modern HUD background
        # Top bar for drone info
        bar_height = 80
        cv2.rectangle(overlay, (0, 0), (w, bar_height), (20, 20, 20), -1)
        alpha_bar = 0.8
        image = cv2.addWeighted(overlay, alpha_bar, image, 1 - alpha_bar, 0)
        
        # Draw crosshair at center (yellow crosshair like in images)
        cx, cy = w // 2, h // 2
        cv2.drawMarker(image, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=30, thickness=3)
        
        # Add horizon line
        horizon_y = h // 2
        cv2.line(image, (0, horizon_y), (w, horizon_y), (0, 255, 255), 2)
        
        # Add state information with modern styling
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 0.7
        font_thickness = 2
        states = self.fleet.get_drone_states()
        
        for i, state in enumerate(states):
            pos = state['position']
            vel = state['velocity']
            orn = state['orientation']
            
            # Calculate flight parameters
            speed = np.linalg.norm(vel)
            altitude = pos[2]
            heading = np.degrees(orn[2]) % 360
            pitch = np.degrees(orn[1])
            
            # Drone identifier
            y0 = 25 + i*35
            cv2.putText(image, f"DRONE #{i+1} - FPV CAMERA", (10, y0), font, font_scale, (255, 255, 0), font_thickness)
            
            # Position and velocity
            pos_text = f"POS X:{pos[0]:+.2f} Y:{pos[1]:+.2f} Z:{pos[2]:+.2f}"
            cv2.putText(image, pos_text, (10, y0+20), font, 0.6, (0, 255, 255), 1)
            vel_text = f"VEL: X:{vel[0]:+.2f} Y:{vel[1]:+.2f} Z:{vel[2]:+.2f}"
            cv2.putText(image, vel_text, (10, y0+35), font, 0.6, (0, 255, 255), 1)
            
            # Status indicators
            status_text = f"CAM:{i+1} | GPS:OK | GYRO:ACTIVE"
            cv2.putText(image, status_text, (w-200, y0), font, 0.5, (0, 255, 0), 1)
        
        # Add telemetry panel (black box with white text like in images)
        panel_width = 200
        panel_height = 150
        panel_x = w - panel_width - 10
        panel_y = bar_height + 10
        
        # Draw telemetry panel background
        cv2.rectangle(image, (panel_x, panel_y), (panel_x + panel_width, panel_y + panel_height), (0, 0, 0), -1)
        cv2.rectangle(image, (panel_x, panel_y), (panel_x + panel_width, panel_y + panel_height), (255, 255, 255), 2)
        
        # Add telemetry data
        if states:
            state = states[0]  # Use first drone for telemetry
            pos = state['position']
            vel = state['velocity']
            orn = state['orientation']
            
            speed = np.linalg.norm(vel)
            altitude = pos[2]
            heading = np.degrees(orn[2]) % 360
            pitch = np.degrees(orn[1])
            
            # Calculate thrust percentage (simplified)
            thrust_percent = min(100, max(0, (np.linalg.norm(vel) + altitude) * 10))
            
            telemetry_data = [
                f"THRUST: {thrust_percent:.1f}%",
                f"SPEED: {speed:.2f} m/s",
                f"ALTITUDE: {altitude:.2f} m",
                f"HEADING: {heading:.1f}°",
                f"PITCH: {pitch:.1f}°",
                f"BATTERY: 100%",
                f"SIGNAL: 60%"
            ]
            
            for j, data in enumerate(telemetry_data):
                cv2.putText(image, data, (panel_x + 5, panel_y + 20 + j*18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add summary data at bottom
        summary_y = h - 30
        summary_text = "BAT: 100% SIG: 60% LAT: 3ms"
        cv2.putText(image, summary_text, (10, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return image
    
    def _get_camera_images(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get enhanced camera images."""
        states = self.fleet.get_drone_states()
        if self.fpp_mode and len(states) > 0:
            # FPP: Use drone's position and orientation
            pos = states[0]['position']
            orn = states[0]['orientation']
            yaw = orn[2]  # yaw from euler angles
            pitch = orn[1]
            # Camera eye: slightly above and behind the drone
            eye = pos + np.array([
                -np.sin(yaw) * 0.3,
                np.cos(yaw) * 0.3,
                0.15
            ])
            # Camera target: look forward from drone
            target = pos + np.array([
                np.sin(yaw) * 2.0,
                np.cos(yaw) * 2.0,
                0.0
            ])
            up = [0, 0, 1]
        elif not self.fpp_mode:
            # Free camera
            eye = self.free_cam_pos
            yaw = np.deg2rad(self.free_cam_yaw)
            pitch = np.deg2rad(self.free_cam_pitch)
            look_dir = np.array([
                np.sin(yaw) * np.cos(pitch),
                np.cos(yaw) * np.cos(pitch),
                np.sin(pitch)
            ])
            target = eye + look_dir
            up = [0, 0, 1]
        else:
            # fallback
            eye = [0, 0, 2]
            target = [0, 0, 0]
            up = [0, 0, 1]
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=eye,
            cameraTargetPosition=target,
            cameraUpVector=up
        )
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=self.camera_fov,
            aspect=self.camera_aspect,
            nearVal=self.camera_near,
            farVal=self.camera_far
        )
        width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=self.camera_width,
            height=self.camera_height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            flags=p.ER_NO_SEGMENTATION_MASK
        )
        rgb_image = np.reshape(rgb_img, (height, width, 4))[:, :, :3]
        depth_image = np.reshape(depth_img, (height, width))
        rgb_image = (rgb_image * 255).astype(np.uint8)
        print(f"[DEBUG] rgb_image shape: {rgb_image.shape}, dtype: {rgb_image.dtype}")
        return rgb_image, depth_image
    
    def update_status_display(self, mode: str = "Autonomous"):
        """Update the status display window with drone information."""
        states = self.fleet.get_drone_states()
        text = "Drone States:\n"
        
        for i, state in enumerate(states):
            pos = state['position']
            vel = state['velocity']
            text += f"Drone {i}:\n"
            text += f"  Position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]\n"
            text += f"  Velocity: [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}]\n"
        
        p.removeUserDebugItem(self.status_display)
        self.status_display = p.addUserDebugText(
            text,
            [0, 0, 2],
            textColorRGB=[1, 1, 1],
            textSize=1.5
        )
    
    def get_drone_state(self, drone_idx: int = 0) -> dict:
        """Get the current state of a specific drone."""
        states = self.fleet.get_drone_states()
        if 0 <= drone_idx < len(states):
            return states[drone_idx]
        return None
    
    def reset(self):
        """Reset the simulation with enhanced visuals."""
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        
        # Reset ground plane with modern appearance
        self.plane_id = p.loadURDF("plane.urdf")
        p.changeVisualShape(
            self.plane_id, 
            -1, 
            rgbaColor=[0.2, 0.2, 0.2, 1],  # Dark gray ground
            specularColor=[0.4, 0.4, 0.4]
        )
        
        # Reset drone fleet
        self.fleet.reset()
        
        # Recreate obstacles
        for obstacle_id in self.obstacle_ids:
            p.removeBody(obstacle_id)
        self.obstacle_ids = []
    
    def close(self):
        """Clean up resources."""
        try:
            cv2.destroyAllWindows()
            self.fleet.close()
            if hasattr(self, 'client') and p.isConnected(self.client):
                p.disconnect(self.client)
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    def remove_obstacle(self, obstacle_id):
        if obstacle_id in self.obstacle_ids:
            p.removeBody(obstacle_id)
            self.obstacle_ids.remove(obstacle_id)
    
    def set_camera_target(self, target_pos):
        # Only works if GUI is enabled, but we keep the method for future GUI support
        self._camera_target = target_pos 

    def draw_3d_path(self, path, grid_size, world_size=10.0):
        # Remove previous path markers if any
        if hasattr(self, '_path_marker_ids'):
            for mid in self._path_marker_ids:
                p.removeBody(mid)
        self._path_marker_ids = []
        for wp in path:
            pos3d = self._grid_to_world(wp, grid_size, world_size)
            marker_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=0.12),
                baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.12, rgbaColor=[1,1,0,0.7]),
                basePosition=pos3d
            )
            self._path_marker_ids.append(marker_id)

    def draw_goal_marker(self, goal, grid_size, world_size=10.0):
        if hasattr(self, '_goal_marker_id') and self._goal_marker_id is not None:
            p.removeBody(self._goal_marker_id)
        pos3d = self._grid_to_world(goal, grid_size, world_size)
        self._goal_marker_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=0.18),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.18, rgbaColor=[1,0,0,1]),
            basePosition=pos3d
        )

    def _grid_to_world(self, grid_pos, grid_size, world_size=10.0, z=1.0):
        gx, gy = grid_pos
        x = (gx / (grid_size-1)) * world_size - world_size/2
        y = (gy / (grid_size-1)) * world_size - world_size/2
        return [x, y, z]

    def set_drone_position(self, drone_idx, position, orientation=None):
        """Set the position (and optionally orientation) of a drone immediately."""
        self.fleet.set_drone_position(drone_idx, position, orientation)

    def add_building(self, position: np.ndarray, size: np.ndarray, color=(0.5, 0.5, 0.5, 1)):
        """Add a 3D building (cuboid) to the environment."""
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=size/2,
            rgbaColor=color,
            specularColor=[0.6, 0.6, 0.6],
            visualFramePosition=[0, 0, 0]
        )
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=size/2
        )
        building_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position,
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        self.obstacle_ids.append(building_id)
        return building_id 