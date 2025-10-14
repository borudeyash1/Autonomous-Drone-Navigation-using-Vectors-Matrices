import numpy as np
import cv2
import time
from typing import List, Dict, Tuple

class SimpleDrone:
    def __init__(self, start_pos, end_pos):
        self.position = np.array(start_pos, dtype=float)
        self.target = np.array(end_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.trajectory = [self.position.copy()]
        
    def update(self, dt=0.01):
        """Simple drone movement towards target"""
        # Calculate direction to target
        direction = self.target - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 0.1:  # If not at target
            # Normalize direction and apply speed
            direction = direction / distance
            speed = 2.0  # m/s
            self.velocity = direction * speed
            
            # Update position
            self.position += self.velocity * dt
            
            # Add to trajectory
            self.trajectory.append(self.position.copy())
        else:
            # At target, stop
            self.velocity = np.zeros(3)
            print("Drone reached target!")

def main():
    print("=== SIMPLE DRONE NAVIGATION SYSTEM ===")
    print("This version works without PyBullet!")
    
    # Create a simple map
    map_size = 600
    map_img = np.ones((map_size, map_size, 3), dtype=np.uint8) * 50
    
    # Draw grid
    for i in range(0, map_size, 30):
        cv2.line(map_img, (i, 0), (i, map_size), (100, 100, 100), 1)
        cv2.line(map_img, (0, i), (map_size, i), (100, 100, 100), 1)
    
    # Set start and end points
    start_world = np.array([-4.0, -2.0, 1.0])
    end_world = np.array([4.0, 2.0, 1.0])
    
    # Convert to image coordinates
    def world_to_img(pos):
        x = int((pos[0] + 5) / 10 * map_size)
        y = int((pos[1] + 5) / 10 * map_size)
        return x, y
    
    start_img = world_to_img(start_world)
    end_img = world_to_img(end_world)
    
    # Draw start and end points
    cv2.circle(map_img, start_img, 15, (0, 255, 0), -1)
    cv2.circle(map_img, end_img, 15, (0, 0, 255), -1)
    cv2.putText(map_img, "START", (start_img[0] - 20, start_img[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(map_img, "GOAL", (end_img[0] - 15, end_img[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Create drone
    drone = SimpleDrone(start_world, end_world)
    
    # Create windows
    cv2.namedWindow('Drone Navigation', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Drone Navigation', 800, 600)
    
    cv2.namedWindow('Status', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Status', 400, 300)
    
    print("Starting simulation...")
    print("Press 'q' to quit, 'r' to reset")
    
    running = True
    start_time = time.time()
    
    while running:
        current_time = time.time()
        dt = min(0.01, current_time - start_time)
        start_time = current_time
        
        # Update drone
        drone.update(dt)
        
        # Create display image
        display_img = map_img.copy()
        
        # Draw trajectory
        if len(drone.trajectory) > 1:
            for i in range(1, len(drone.trajectory)):
                pt1 = world_to_img(drone.trajectory[i-1])
                pt2 = world_to_img(drone.trajectory[i])
                cv2.line(display_img, pt1, pt2, (0, 255, 255), 2)
        
        # Draw drone
        drone_img = world_to_img(drone.position)
        cv2.circle(display_img, drone_img, 10, (255, 255, 0), -1)
        cv2.putText(display_img, "DRONE", (drone_img[0] - 20, drone_img[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Add title
        cv2.putText(display_img, "AUTONOMOUS DRONE NAVIGATION", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Display
        cv2.imshow('Drone Navigation', display_img)
        
        # Create status display
        status_img = np.zeros((300, 400, 3), dtype=np.uint8)
        cv2.putText(status_img, "DRONE STATUS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        pos = drone.position
        vel = drone.velocity
        distance = np.linalg.norm(drone.target - drone.position)
        
        cv2.putText(status_img, f"Position: X:{pos[0]:+.2f} Y:{pos[1]:+.2f} Z:{pos[2]:+.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(status_img, f"Velocity: X:{vel[0]:+.2f} Y:{vel[1]:+.2f} Z:{vel[2]:+.2f}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(status_img, f"Distance to Goal: {distance:.2f}m", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(status_img, f"Speed: {np.linalg.norm(vel):.2f} m/s", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Status
        if distance < 0.5:
            cv2.putText(status_img, "STATUS: TARGET REACHED!", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(status_img, "STATUS: NAVIGATING", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        
        cv2.putText(status_img, "Controls:", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(status_img, "Q - Quit", (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(status_img, "R - Reset", (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        cv2.imshow('Status', status_img)
        
        # Handle input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            running = False
        elif key == ord('r'):
            # Reset drone
            drone = SimpleDrone(start_world, end_world)
            print("Drone reset!")
        
        # Check if reached target
        if distance < 0.1:
            print("Mission accomplished! Drone reached the target!")
            time.sleep(2)
            break
    
    cv2.destroyAllWindows()
    print("Simulation ended.")

if __name__ == "__main__":
    main()
