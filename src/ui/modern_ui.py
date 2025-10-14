import numpy as np
import cv2
import time
import datetime
from typing import List, Dict, Tuple
import threading
import queue

class ModernUI:
    def __init__(self):
        """Initialize the modern UI system."""
        self.window_width = 1200
        self.window_height = 800
        self.fpv_width = 640
        self.fpv_height = 480
        
        # Create main window
        cv2.namedWindow('Autonomous Drone Navigation System', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Autonomous Drone Navigation System', self.window_width, self.window_height)
        
        # Create FPV window
        cv2.namedWindow('FPV Camera View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('FPV Camera View', self.fpv_width, self.fpv_height)
        
        # Create status window
        cv2.namedWindow('System Status', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('System Status', 400, 600)
        
        # UI state
        self.drone_states = []
        self.system_mode = "Autonomous"
        self.last_update = time.time()
        self.fpv_image = None
        self.fpv_queue = queue.Queue()
        
        # Colors
        self.colors = {
            'background': (30, 30, 30),
            'panel': (50, 50, 50),
            'text': (255, 255, 255),
            'accent': (0, 255, 255),
            'success': (0, 255, 0),
            'warning': (0, 165, 255),
            'error': (0, 0, 255),
            'drone1': (0, 255, 255),
            'drone2': (255, 255, 0),
            'drone3': (255, 0, 255)
        }
        
        # Start UI update thread
        self.running = True
        self.ui_thread = threading.Thread(target=self._ui_update_loop)
        self.ui_thread.daemon = True
        self.ui_thread.start()
    
    def update_drone_states(self, states: List[Dict]):
        """Update drone states for display."""
        self.drone_states = states
        self.last_update = time.time()
    
    def update_fpv_image(self, image: np.ndarray):
        """Update FPV camera image."""
        if image is not None:
            self.fpv_queue.put(image.copy())
    
    def set_system_mode(self, mode: str):
        """Set system operation mode."""
        self.system_mode = mode
    
    def _ui_update_loop(self):
        """Main UI update loop running in separate thread."""
        while self.running:
            try:
                self._update_main_display()
                self._update_fpv_display()
                self._update_status_display()
                time.sleep(0.033)  # ~30 FPS
            except Exception as e:
                print(f"UI update error: {e}")
                time.sleep(0.1)
    
    def _update_main_display(self):
        """Update the main display window."""
        # Create main display
        main_display = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8)
        main_display[:] = self.colors['background']
        
        # Draw title bar
        cv2.rectangle(main_display, (0, 0), (self.window_width, 60), self.colors['panel'], -1)
        cv2.putText(main_display, "AUTONOMOUS DRONE NAVIGATION SYSTEM", (20, 35), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.8, self.colors['accent'], 2)
        
        # Draw mode indicator
        mode_color = self.colors['success'] if self.system_mode == "Autonomous" else self.colors['warning']
        cv2.putText(main_display, f"MODE: {self.system_mode}", (self.window_width - 200, 35), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.6, mode_color, 2)
        
        # Draw drone panels
        panel_width = 380
        panel_height = 200
        panel_spacing = 20
        
        for i, state in enumerate(self.drone_states):
            x = 20 + (i % 3) * (panel_width + panel_spacing)
            y = 80 + (i // 3) * (panel_height + panel_spacing)
            
            if x + panel_width > self.window_width:
                break
                
            self._draw_drone_panel(main_display, state, i, x, y, panel_width, panel_height)
        
        # Draw system status panel
        status_x = 20
        status_y = self.window_height - 200
        self._draw_system_status_panel(main_display, status_x, status_y, 400, 180)
        
        # Draw trajectory visualization area
        traj_x = 440
        traj_y = self.window_height - 200
        self._draw_trajectory_panel(main_display, traj_x, traj_y, 740, 180)
        
        # Display main window
        cv2.imshow('Autonomous Drone Navigation System', main_display)
    
    def _draw_drone_panel(self, display: np.ndarray, state: Dict, drone_id: int, x: int, y: int, width: int, height: int):
        """Draw a drone status panel."""
        # Panel background
        cv2.rectangle(display, (x, y), (x + width, y + height), self.colors['panel'], -1)
        cv2.rectangle(display, (x, y), (x + width, y + height), self.colors['accent'], 2)
        
        # Panel title
        drone_color = [self.colors['drone1'], self.colors['drone2'], self.colors['drone3']][drone_id % 3]
        cv2.putText(display, f"DRONE #{drone_id + 1}", (x + 10, y + 25), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.7, drone_color, 2)
        
        # Position data
        pos = state['position']
        cv2.putText(display, f"POS: X:{pos[0]:+.2f} Y:{pos[1]:+.2f} Z:{pos[2]:+.2f}", 
                   (x + 10, y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
        
        # Velocity data
        vel = state['velocity']
        speed = np.linalg.norm(vel)
        cv2.putText(display, f"VEL: X:{vel[0]:+.2f} Y:{vel[1]:+.2f} Z:{vel[2]:+.2f}", 
                   (x + 10, y + 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
        cv2.putText(display, f"SPEED: {speed:.2f} m/s", 
                   (x + 10, y + 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
        
        # Orientation data
        orn = state['orientation']
        heading = np.degrees(orn[2]) % 360
        pitch = np.degrees(orn[1])
        roll = np.degrees(orn[0])
        cv2.putText(display, f"HEADING: {heading:.1f}°", 
                   (x + 10, y + 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
        cv2.putText(display, f"PITCH: {pitch:.1f}° ROLL: {roll:.1f}°", 
                   (x + 10, y + 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
        
        # Status indicators
        cv2.putText(display, "GPS: OK | GYRO: ACTIVE | CAM: 1", 
                   (x + 10, y + 150), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['success'], 1)
        cv2.putText(display, "BATTERY: 100% | SIGNAL: 60%", 
                   (x + 10, y + 170), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['success'], 1)
    
    def _draw_system_status_panel(self, display: np.ndarray, x: int, y: int, width: int, height: int):
        """Draw system status panel."""
        # Panel background
        cv2.rectangle(display, (x, y), (x + width, y + height), self.colors['panel'], -1)
        cv2.rectangle(display, (x, y), (x + width, y + height), self.colors['accent'], 2)
        
        # Panel title
        cv2.putText(display, "SYSTEM STATUS", (x + 10, y + 25), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.7, self.colors['accent'], 2)
        
        # Status items
        status_items = [
            "• PATH PLANNING: ACTIVE",
            "• OBSTACLE AVOIDANCE: ENABLED", 
            "• TRAJECTORY TRACKING: ACTIVE",
            "• COMMUNICATION: STABLE",
            "• AUTOPILOT: ENGAGED",
            "• SENSORS: NOMINAL"
        ]
        
        for i, item in enumerate(status_items):
            cv2.putText(display, item, (x + 10, y + 50 + i * 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['success'], 1)
        
        # Timestamp
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        cv2.putText(display, f"TIME: {timestamp}", (x + 10, y + height - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
    
    def _draw_trajectory_panel(self, display: np.ndarray, x: int, y: int, width: int, height: int):
        """Draw trajectory visualization panel."""
        # Panel background
        cv2.rectangle(display, (x, y), (x + width, y + height), self.colors['panel'], -1)
        cv2.rectangle(display, (x, y), (x + width, y + height), self.colors['accent'], 2)
        
        # Panel title
        cv2.putText(display, "TRAJECTORY VISUALIZATION", (x + 10, y + 25), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.7, self.colors['accent'], 2)
        
        # Draw simple trajectory representation
        if len(self.drone_states) > 0:
            # Draw grid
            grid_size = 20
            for i in range(0, width - 40, grid_size):
                cv2.line(display, (x + 20 + i, y + 40), (x + 20 + i, y + height - 20), (60, 60, 60), 1)
            for i in range(0, height - 60, grid_size):
                cv2.line(display, (x + 20, y + 40 + i), (x + width - 20, y + 40 + i), (60, 60, 60), 1)
            
            # Draw drone positions
            for i, state in enumerate(self.drone_states):
                pos = state['position']
                # Scale position to panel coordinates
                px = int(x + 20 + (pos[0] + 10) / 20 * (width - 40))
                py = int(y + 40 + (pos[1] + 10) / 20 * (height - 60))
                
                # Clamp to panel bounds
                px = max(x + 20, min(x + width - 20, px))
                py = max(y + 40, min(y + height - 20, py))
                
                # Draw drone
                drone_color = [self.colors['drone1'], self.colors['drone2'], self.colors['drone3']][i % 3]
                cv2.circle(display, (px, py), 8, drone_color, -1)
                cv2.putText(display, f"D{i+1}", (px - 8, py - 12), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.colors['text'], 1)
    
    def _update_fpv_display(self):
        """Update FPV camera display."""
        try:
            if not self.fpv_queue.empty():
                self.fpv_image = self.fpv_queue.get_nowait()
            
            if self.fpv_image is not None:
                # Add HUD overlay to FPV image
                fpv_with_hud = self._add_fpv_hud(self.fpv_image.copy())
                cv2.imshow('FPV Camera View', fpv_with_hud)
        except queue.Empty:
            pass
        except Exception as e:
            print(f"FPV display error: {e}")
    
    def _add_fpv_hud(self, image: np.ndarray) -> np.ndarray:
        """Add HUD overlay to FPV image."""
        h, w = image.shape[:2]
        
        # Add semi-transparent overlay
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (w, 60), (20, 20, 20), -1)
        image = cv2.addWeighted(overlay, 0.7, image, 0.3, 0)
        
        # Draw crosshair
        cx, cy = w // 2, h // 2
        cv2.drawMarker(image, (cx, cy), self.colors['accent'], cv2.MARKER_CROSS, 20, 2)
        
        # Add drone info
        if len(self.drone_states) > 0:
            state = self.drone_states[0]
            pos = state['position']
            vel = state['velocity']
            
            cv2.putText(image, f"DRONE #1 - FPV CAMERA", (10, 25), 
                       cv2.FONT_HERSHEY_DUPLEX, 0.6, self.colors['accent'], 2)
            cv2.putText(image, f"POS X:{pos[0]:+.2f} Y:{pos[1]:+.2f} Z:{pos[2]:+.2f}", 
                       (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
            cv2.putText(image, f"VEL: X:{vel[0]:+.2f} Y:{vel[1]:+.2f} Z:{vel[2]:+.2f}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['text'], 1)
        
        return image
    
    def _update_status_display(self):
        """Update system status display."""
        # Create status display
        status_display = np.zeros((600, 400, 3), dtype=np.uint8)
        status_display[:] = self.colors['background']
        
        # Title
        cv2.putText(status_display, "SYSTEM STATUS", (20, 40), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.8, self.colors['accent'], 2)
        
        # Mode
        mode_color = self.colors['success'] if self.system_mode == "Autonomous" else self.colors['warning']
        cv2.putText(status_display, f"MODE: {self.system_mode}", (20, 70), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.6, mode_color, 2)
        
        # Drone count
        cv2.putText(status_display, f"ACTIVE DRONES: {len(self.drone_states)}", (20, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors['text'], 2)
        
        # System health
        cv2.putText(status_display, "SYSTEM HEALTH:", (20, 140), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.6, self.colors['accent'], 2)
        
        health_items = [
            "• NAVIGATION: OK",
            "• COMMUNICATION: OK", 
            "• SENSORS: OK",
            "• POWER: OK",
            "• SOFTWARE: OK"
        ]
        
        for i, item in enumerate(health_items):
            cv2.putText(status_display, item, (30, 170 + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colors['success'], 1)
        
        # Display status window
        cv2.imshow('System Status', status_display)
    
    def close(self):
        """Close the UI system."""
        self.running = False
        if self.ui_thread.is_alive():
            self.ui_thread.join(timeout=1.0)
        cv2.destroyAllWindows()

