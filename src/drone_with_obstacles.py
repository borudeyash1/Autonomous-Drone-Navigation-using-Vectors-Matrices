import numpy as np
import cv2
import time
from typing import List, Dict, Tuple

class DroneWithObstacles:
    def __init__(self, start_pos, end_pos, obstacles=None):
        self.position = np.array(start_pos, dtype=float)
        self.target = np.array(end_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.trajectory = [self.position.copy()]
        self.obstacles = obstacles or []
        self.path = self.plan_path()
        self.path_index = 0
        
    def plan_path(self):
        """Simple path planning avoiding obstacles"""
        # For now, just go directly to target
        # In a real implementation, this would use A* or similar
        return [self.target.copy()]
    
    def check_collision(self, pos):
        """Check if position collides with obstacles"""
        for obs in self.obstacles:
            if len(obs) >= 3:  # x, y, radius
                obs_pos = np.array(obs[:2])
                radius = obs[2]
                if np.linalg.norm(pos[:2] - obs_pos) < radius:
                    return True
        return False
    
    def update(self, dt=0.01):
        """Drone movement with obstacle avoidance"""
        # Calculate direction to target
        direction = self.target - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 0.1:  # If not at target
            # Normalize direction
            if distance > 0:
                direction = direction / distance
            
            # Simple obstacle avoidance
            avoidance_force = np.zeros(3)
            for obs in self.obstacles:
                if len(obs) >= 3:
                    obs_pos = np.array(obs[:2])
                    obs_radius = obs[2]
                    
                    # Calculate distance to obstacle
                    obs_distance = np.linalg.norm(self.position[:2] - obs_pos)
                    
                    if obs_distance < obs_radius * 2:  # If too close to obstacle
                        # Calculate avoidance force
                        avoid_direction = self.position[:2] - obs_pos
                        if np.linalg.norm(avoid_direction) > 0:
                            avoid_direction = avoid_direction / np.linalg.norm(avoid_direction)
                            # Stronger force when closer
                            force_strength = (obs_radius * 2 - obs_distance) / (obs_radius * 2)
                            avoidance_force[:2] += avoid_direction * force_strength * 3.0
            
            # Combine movement and avoidance
            movement_direction = direction + avoidance_force
            if np.linalg.norm(movement_direction) > 0:
                movement_direction = movement_direction / np.linalg.norm(movement_direction)
            
            # Apply speed
            speed = 1.5  # m/s (slower for obstacle avoidance)
            self.velocity = movement_direction * speed
            
            # Update position
            new_position = self.position + self.velocity * dt
            
            # Check for collisions
            if not self.check_collision(new_position):
                self.position = new_position
                self.trajectory.append(self.position.copy())
            else:
                # If collision, try to move around obstacle
                self.velocity = np.array([0.0, 0.0, 0.0])
        else:
            # At target, stop
            self.velocity = np.zeros(3)
            print("Drone reached target!")

def main():
    print("=== DRONE NAVIGATION WITH OBSTACLES ===")
    print("This version includes obstacle avoidance!")
    
    # Create a map
    map_size = 600
    map_img = np.ones((map_size, map_size, 3), dtype=np.uint8) * 50
    
    # Draw grid
    for i in range(0, map_size, 30):
        cv2.line(map_img, (i, 0), (i, map_size), (100, 100, 100), 1)
        cv2.line(map_img, (0, i), (map_size, i), (100, 100, 100), 1)
    
    # Set start and end points
    start_world = np.array([-4.0, -2.0, 1.0])
    end_world = np.array([4.0, 2.0, 1.0])
    
    # Define obstacles (x, y, radius)
    obstacles = [
        [-1.0, 0.0, 0.8],   # Obstacle 1
        [1.0, -1.0, 0.6],   # Obstacle 2
        [0.0, 1.0, 0.7],    # Obstacle 3
    ]
    
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
    
    # Draw obstacles
    for obs in obstacles:
        obs_img = world_to_img(obs)
        radius_img = int(obs[2] / 10 * map_size)
        cv2.circle(map_img, obs_img, radius_img, (0, 0, 255), -1)
        cv2.circle(map_img, obs_img, radius_img, (255, 255, 255), 2)
    
    # Create drone
    drone = DroneWithObstacles(start_world, end_world, obstacles)
    
    # Create windows
    cv2.namedWindow('Drone Navigation with Obstacles', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Drone Navigation with Obstacles', 800, 600)
    
    cv2.namedWindow('Status', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Status', 400, 300)
    
    print("Starting simulation...")
    print("Press 'q' to quit, 'r' to reset")
    
    running = True
    start_time = time.time()
    collision_count = 0
    
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
        
        # Draw velocity vector
        vel_scale = 20
        vel_end = (drone_img[0] + int(drone.velocity[0] * vel_scale), 
                  drone_img[1] + int(drone.velocity[1] * vel_scale))
        cv2.arrowedLine(display_img, drone_img, vel_end, (255, 0, 255), 2)
        
        # Add title
        cv2.putText(display_img, "AUTONOMOUS DRONE NAVIGATION WITH OBSTACLE AVOIDANCE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Display
        cv2.imshow('Drone Navigation with Obstacles', display_img)
        
        # Create status display
        status_img = np.zeros((300, 400, 3), dtype=np.uint8)
        cv2.putText(status_img, "DRONE STATUS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        pos = drone.position
        vel = drone.velocity
        distance = np.linalg.norm(drone.target - drone.position)
        speed = np.linalg.norm(vel)
        
        cv2.putText(status_img, f"Position: X:{pos[0]:+.2f} Y:{pos[1]:+.2f} Z:{pos[2]:+.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(status_img, f"Velocity: X:{vel[0]:+.2f} Y:{vel[1]:+.2f} Z:{vel[2]:+.2f}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(status_img, f"Distance to Goal: {distance:.2f}m", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(status_img, f"Speed: {speed:.2f} m/s", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Check for collisions
        if drone.check_collision(drone.position):
            collision_count += 1
            cv2.putText(status_img, "COLLISION DETECTED!", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(status_img, "STATUS: SAFE", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Status
        if distance < 0.5:
            cv2.putText(status_img, "MISSION: TARGET REACHED!", (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(status_img, "MISSION: NAVIGATING", (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        
        cv2.putText(status_img, f"Collisions: {collision_count}", (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(status_img, "Controls:", (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(status_img, "Q - Quit, R - Reset", (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        cv2.imshow('Status', status_img)
        
        # Handle input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            running = False
        elif key == ord('r'):
            # Reset drone
            drone = DroneWithObstacles(start_world, end_world, obstacles)
            collision_count = 0
            print("Drone reset!")
        
        # Check if reached target
        if distance < 0.1:
            print("Mission accomplished! Drone reached the target!")
            print(f"Total collisions: {collision_count}")
            time.sleep(3)
            break
    
    cv2.destroyAllWindows()
    print("Simulation ended.")

if __name__ == "__main__":
    main()
