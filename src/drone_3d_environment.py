import numpy as np
import cv2
import time
import math
from typing import List, Dict, Tuple
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Drone3DEnvironment:
    def __init__(self, start_pos, end_pos, obstacles=None):
        self.position = np.array(start_pos, dtype=float)
        self.target = np.array(end_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.trajectory = [self.position.copy()]
        self.obstacles = obstacles or []
        self.path = self.plan_path()
        self.path_index = 0
        
        # 3D visualization setup
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_3d_environment()
        
    def setup_3d_environment(self):
        """Setup the 3D environment visualization"""
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_zlabel('Z (meters)')
        self.ax.set_title('3D Drone Navigation Environment')
        
        # Set axis limits
        self.ax.set_xlim([-5, 5])
        self.ax.set_ylim([-5, 5])
        self.ax.set_zlim([0, 5])
        
        # Draw ground plane
        xx, yy = np.meshgrid(np.linspace(-5, 5, 20), np.linspace(-5, 5, 20))
        zz = np.zeros_like(xx)
        self.ax.plot_surface(xx, yy, zz, alpha=0.3, color='gray')
        
        # Draw obstacles
        self.draw_obstacles_3d()
        
        # Draw start and end points
        self.ax.scatter([self.position[0]], [self.position[1]], [self.position[2]], 
                       color='green', s=100, marker='o', label='Start')
        self.ax.scatter([self.target[0]], [self.target[1]], [self.target[2]], 
                       color='red', s=100, marker='*', label='Goal')
        
        self.ax.legend()
        plt.ion()  # Turn on interactive mode
        
    def draw_obstacles_3d(self):
        """Draw 3D obstacles"""
        for obs in self.obstacles:
            if len(obs) >= 4:  # x, y, z, radius
                x, y, z, radius = obs[:4]
                height = obs[4] if len(obs) > 4 else 2.0
                
                # Create cylinder obstacle
                u = np.linspace(0, 2 * np.pi, 20)
                v = np.linspace(0, height, 10)
                u, v = np.meshgrid(u, v)
                
                x_obs = x + radius * np.cos(u)
                y_obs = y + radius * np.sin(u)
                z_obs = z + v
                
                self.ax.plot_surface(x_obs, y_obs, z_obs, alpha=0.7, color='red')
    
    def plan_path(self):
        """Simple 3D path planning avoiding obstacles"""
        # For now, just go directly to target
        # In a real implementation, this would use 3D A* or similar
        return [self.target.copy()]
    
    def check_collision_3d(self, pos):
        """Check if position collides with 3D obstacles"""
        for obs in self.obstacles:
            if len(obs) >= 4:
                obs_pos = np.array(obs[:3])
                radius = obs[3]
                height = obs[4] if len(obs) > 4 else 2.0
                
                # Check horizontal distance
                horizontal_dist = np.linalg.norm(pos[:2] - obs_pos[:2])
                # Check vertical overlap
                vertical_overlap = (pos[2] >= obs_pos[2]) and (pos[2] <= obs_pos[2] + height)
                
                if horizontal_dist < radius and vertical_overlap:
                    return True
        return False
    
    def update(self, dt=0.01):
        """3D drone movement with obstacle avoidance"""
        # Calculate direction to target
        direction = self.target - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 0.1:  # If not at target
            # Normalize direction
            if distance > 0:
                direction = direction / distance
            
            # Simple 3D obstacle avoidance
            avoidance_force = np.zeros(3)
            for obs in self.obstacles:
                if len(obs) >= 4:
                    obs_pos = np.array(obs[:3])
                    obs_radius = obs[3]
                    obs_height = obs[4] if len(obs) > 4 else 2.0
                    
                    # Calculate distance to obstacle
                    horizontal_dist = np.linalg.norm(self.position[:2] - obs_pos[:2])
                    vertical_dist = abs(self.position[2] - obs_pos[2])
                    
                    # Check if we're in the obstacle's influence zone
                    if horizontal_dist < obs_radius * 2 and vertical_dist < obs_height:
                        # Calculate avoidance force
                        avoid_direction = self.position - obs_pos
                        if np.linalg.norm(avoid_direction) > 0:
                            avoid_direction = avoid_direction / np.linalg.norm(avoid_direction)
                            # Stronger force when closer
                            force_strength = (obs_radius * 2 - horizontal_dist) / (obs_radius * 2)
                            avoidance_force += avoid_direction * force_strength * 2.0
            
            # Combine movement and avoidance
            movement_direction = direction + avoidance_force
            if np.linalg.norm(movement_direction) > 0:
                movement_direction = movement_direction / np.linalg.norm(movement_direction)
            
            # Apply speed
            speed = 1.0  # m/s
            self.velocity = movement_direction * speed
            
            # Update orientation based on movement direction
            if np.linalg.norm(self.velocity) > 0:
                self.orientation[2] = math.atan2(self.velocity[1], self.velocity[0])  # yaw
                self.orientation[1] = math.atan2(-self.velocity[2], 
                                               np.linalg.norm(self.velocity[:2]))  # pitch
            
            # Update position
            new_position = self.position + self.velocity * dt
            
            # Check for collisions
            if not self.check_collision_3d(new_position):
                self.position = new_position
                self.trajectory.append(self.position.copy())
            else:
                # If collision, try to move around obstacle
                self.velocity = np.array([0.0, 0.0, 0.0])
        else:
            # At target, stop
            self.velocity = np.zeros(3)
            print("Drone reached target!")
    
    def update_3d_visualization(self):
        """Update the 3D visualization"""
        # Clear previous drone position
        self.ax.clear()
        
        # Redraw environment
        self.setup_3d_environment()
        
        # Draw trajectory
        if len(self.trajectory) > 1:
            traj = np.array(self.trajectory)
            self.ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 
                        color='cyan', linewidth=2, alpha=0.8, label='Trajectory')
        
        # Draw drone as a 3D arrow
        if np.linalg.norm(self.velocity) > 0:
            # Calculate arrow direction
            arrow_length = 0.5
            arrow_end = self.position + self.velocity * arrow_length
            
            self.ax.quiver(self.position[0], self.position[1], self.position[2],
                          self.velocity[0], self.velocity[1], self.velocity[2],
                          color='yellow', arrow_length_ratio=0.1, linewidth=3)
        
        # Draw drone position
        self.ax.scatter([self.position[0]], [self.position[1]], [self.position[2]], 
                       color='yellow', s=200, marker='^', label='Drone')
        
        # Add status text
        distance = np.linalg.norm(self.target - self.position)
        speed = np.linalg.norm(self.velocity)
        
        self.ax.text2D(0.02, 0.98, f'Distance to Goal: {distance:.2f}m\nSpeed: {speed:.2f} m/s', 
                      transform=self.ax.transAxes, fontsize=10, 
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.draw()
        plt.pause(0.01)

def main():
    print("=== 3D DRONE NAVIGATION ENVIRONMENT ===")
    print("This version provides full 3D visualization!")
    
    # Set start and end points
    start_world = np.array([-3.0, -2.0, 1.0])
    end_world = np.array([3.0, 2.0, 2.0])
    
    # Define 3D obstacles (x, y, z, radius, height)
    obstacles = [
        [-1.0, 0.0, 0.5, 0.8, 2.0],   # Obstacle 1
        [1.0, -1.0, 0.5, 0.6, 1.5],   # Obstacle 2
        [0.0, 1.0, 0.5, 0.7, 2.5],    # Obstacle 3
        [2.0, 0.0, 0.5, 0.5, 1.0],    # Obstacle 4
    ]
    
    # Create drone
    drone = Drone3DEnvironment(start_world, end_world, obstacles)
    
    print("Starting 3D simulation...")
    print("Close the 3D window to stop the simulation")
    
    running = True
    start_time = time.time()
    collision_count = 0
    
    try:
        while running:
            current_time = time.time()
            dt = min(0.01, current_time - start_time)
            start_time = current_time
            
            # Update drone
            drone.update(dt)
            
            # Update 3D visualization
            drone.update_3d_visualization()
            
            # Check for collisions
            if drone.check_collision_3d(drone.position):
                collision_count += 1
                print(f"Collision detected! Total: {collision_count}")
            
            # Check if reached target
            distance = np.linalg.norm(drone.target - drone.position)
            if distance < 0.1:
                print("Mission accomplished! Drone reached the target!")
                print(f"Total collisions: {collision_count}")
                time.sleep(3)
                break
            
            # Check if window is closed
            if not plt.get_fignums():
                running = False
                
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        plt.close('all')
        print("3D simulation ended.")

if __name__ == "__main__":
    main()
