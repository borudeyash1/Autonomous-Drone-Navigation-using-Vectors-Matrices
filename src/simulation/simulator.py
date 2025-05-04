import pybullet as p
import pybullet_data
import numpy as np
import time
from typing import Tuple, Optional, List, Dict
import cv2
from .drone_fleet import DroneFleet

class Simulator:
    def __init__(self, gui: bool = False, num_drones: int = 1):  # Default to DIRECT mode
        """
        Initialize the PyBullet simulation environment.
        
        Args:
            gui: Whether to show the GUI (default: False)
            num_drones: Number of drones to create (default: 1)
        """
        # Connect to physics server
        if gui:
            self.client = p.connect(p.GUI)
            # Configure GUI appearance
            p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            
            # Set camera parameters for external view
            p.resetDebugVisualizerCamera(
                cameraDistance=10.0,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0]
            )
            
            # Set rendering options
            p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
            
            # Set background color to a modern dark theme
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setPhysicsEngineParameter(numSolverIterations=10)
            p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
            p.setGravity(0, 0, -9.81)
            
            # Configure ground plane appearance with enhanced materials
            self.plane_id = p.loadURDF("plane.urdf")
            p.changeVisualShape(
                self.plane_id, 
                -1, 
                rgbaColor=[0.2, 0.2, 0.2, 1],  # Dark gray ground
                specularColor=[0.4, 0.4, 0.4]
            )
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
        self.fleet = DroneFleet(self.client, num_drones)
        
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
        
        # Create OpenCV windows
        cv2.namedWindow('FPP View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('FPP View', self.camera_width, self.camera_height)
        cv2.setMouseCallback('FPP View', self._mouse_event)
        cv2.namedWindow('Drone Status', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Drone Status', 400, 300)
        
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
            
            return rgb_image, depth_image
        except Exception as e:
            print(f"Error in simulation step: {e}")
            return np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8), np.zeros((self.camera_height, self.camera_width))
    
    def _add_hud_overlay(self, image: np.ndarray) -> np.ndarray:
        """Add modern HUD overlay to camera view."""
        # Convert image to uint8 if needed
        if image.dtype != np.uint8:
            image = (image * 255).astype(np.uint8)
        overlay = image.copy()
        h, w = overlay.shape[:2]
        # Draw semi-transparent bar at the top
        bar_height = 60
        cv2.rectangle(overlay, (0, 0), (w, bar_height), (30, 30, 30), -1)
        alpha_bar = 0.6
        image = cv2.addWeighted(overlay, alpha_bar, image, 1 - alpha_bar, 0)
        # Draw crosshair at center
        cx, cy = w // 2, h // 2
        cv2.drawMarker(image, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        # Add state information with modern font and styling
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 0.7
        font_thickness = 2
        states = self.fleet.get_drone_states()
        for i, state in enumerate(states):
            pos = state['position']
            vel = state['velocity']
            y0 = 25 + i*30
            cv2.putText(image, f"Drone {i+1}", (10, y0), font, font_scale, (255, 255, 0), font_thickness)
            pos_text = f"POS: X:{pos[0]:+.2f} Y:{pos[1]:+.2f} Z:{pos[2]:+.2f}"
            cv2.putText(image, pos_text, (120, y0), font, 0.6, (0, 255, 255), 1)
            vel_text = f"VEL: X:{vel[0]:+.2f} Y:{vel[1]:+.2f} Z:{vel[2]:+.2f}"
            cv2.putText(image, vel_text, (120, y0+18), font, 0.6, (0, 255, 255), 1)
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
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.18, rgbaColor=[0,1,0,1]),
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