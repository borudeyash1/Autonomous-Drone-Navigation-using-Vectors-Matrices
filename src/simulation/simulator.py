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
        cv2.namedWindow('Drone Camera View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Drone Camera View', self.camera_width, self.camera_height)
        cv2.setMouseCallback('Drone Camera View', self._mouse_event)
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
    
    def _mouse_event(self, event, x, y, flags, param):
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
                cv2.imshow('Drone Camera View', rgb_image)
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
        
        # Create a copy of the image
        overlay = image.copy()
        
        # Get drone states
        states = self.fleet.get_drone_states()
        
        # Add state information with modern font and styling
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_thickness = 1
        
        # Display information for each drone
        for i, state in enumerate(states):
            pos = state['position']
            vel = state['velocity']
            
            # Drone number
            cv2.putText(overlay, f"Drone {i+1}:", (10, 20 + i*30), font, font_scale, (255, 255, 255), font_thickness)
            
            # Position information
            pos_text = f"POS: X:{pos[0]:+.2f} Y:{pos[1]:+.2f} Z:{pos[2]:+.2f}"
            cv2.putText(overlay, pos_text, (20, 20 + i*30), font, font_scale, (0, 255, 255), font_thickness)
            
            # Velocity information
            vel_text = f"VEL: X:{vel[0]:+.2f} Y:{vel[1]:+.2f} Z:{vel[2]:+.2f}"
            cv2.putText(overlay, vel_text, (20, 40 + i*30), font, font_scale, (0, 255, 255), font_thickness)
        
        # Blend the overlay with the original image
        alpha = 0.8
        image = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)
        
        return image
    
    def _get_camera_images(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get enhanced camera images."""
        states = self.fleet.get_drone_states()
        if len(states) == 0:
            # fallback to default
            eye = [0, 0, 2]
            target = [0, 0, 0]
        else:
            # Use the first drone's position
            pos = states[0]['position']
            # Camera slightly above and behind the drone, looking forward
            eye = [pos[0], pos[1] - 2, pos[2] + 1]
            target = [pos[0], pos[1], pos[2]]

        view_matrix = p.computeViewMatrix(
            cameraEyePosition=eye,
            cameraTargetPosition=target,
            cameraUpVector=[0, 0, 1]
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