import numpy as np
import cv2
import time
import math
from typing import List, Dict, Tuple
import matplotlib
matplotlib.use('Qt5Agg')  # Use Qt5 backend for better stability
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Button, Slider, RadioButtons
import tkinter as tk
from tkinter import ttk, messagebox

class Obstacle3D:
    def __init__(self, position, shape="cylinder", size=(1.0, 1.0, 2.0), color='red'):
        self.position = np.array(position)
        self.shape = shape  # "cylinder", "cube", "sphere"
        self.size = size  # (radius/width, length, height) for cylinder/cube, (radius) for sphere
        self.color = color
        self.id = None

class Enhanced3DDrone:
    def __init__(self, start_pos, end_pos):
        self.position = np.array(start_pos, dtype=float)
        self.target = np.array(end_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])
        self.trajectory = [self.position.copy()]
        self.obstacles = []
        self.drone_shape = "triangle"  # "triangle", "cube", "sphere"
        self.drone_size = 0.3
        
        # 3D visualization setup
        self.fig = plt.figure(figsize=(15, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_3d_environment()
        
        # Setup interactive controls
        self.setup_controls()
        
        # Interaction state
        self.placement_mode = False
        self.current_obstacle_shape = "cylinder"
        self.current_obstacle_size = (1.0, 1.0, 2.0)
        self.current_obstacle_color = 'red'
        
    def setup_3d_environment(self):
        """Setup the 3D environment visualization with 2D overlay"""
        # Create subplots: main 3D view and 2D top-down view
        self.ax_3d = self.fig.add_subplot(121, projection='3d')
        self.ax_2d = self.fig.add_subplot(122)
        
        # Setup 3D view
        self.ax_3d.set_xlabel('X (meters)')
        self.ax_3d.set_ylabel('Y (meters)')
        self.ax_3d.set_zlabel('Z (meters)')
        self.ax_3d.set_title('3D Drone Navigation Environment')
        
        # Set axis limits
        self.ax_3d.set_xlim([-5, 5])
        self.ax_3d.set_ylim([-5, 5])
        self.ax_3d.set_zlim([0, 5])
        
        # Draw ground plane
        xx, yy = np.meshgrid(np.linspace(-5, 5, 20), np.linspace(-5, 5, 20))
        zz = np.zeros_like(xx)
        self.ax_3d.plot_surface(xx, yy, zz, alpha=0.3, color='gray')
        
        # Draw start and end points
        self.ax_3d.scatter([self.position[0]], [self.position[1]], [self.position[2]], 
                           color='green', s=100, marker='o', label='Start')
        self.ax_3d.scatter([self.target[0]], [self.target[1]], [self.target[2]], 
                           color='red', s=100, marker='*', label='Goal')
        
        self.ax_3d.legend()
        
        # Setup 2D top-down view
        self.ax_2d.set_xlabel('X (meters)')
        self.ax_2d.set_ylabel('Y (meters)')
        self.ax_2d.set_title('2D Top-Down View')
        self.ax_2d.set_xlim([-5, 5])
        self.ax_2d.set_ylim([-5, 5])
        self.ax_2d.grid(True, alpha=0.3)
        self.ax_2d.set_aspect('equal')
        
        # Draw start and end points in 2D
        self.ax_2d.scatter([self.position[0]], [self.position[1]], 
                          color='green', s=100, marker='o', label='Start')
        self.ax_2d.scatter([self.target[0]], [self.target[1]], 
                          color='red', s=100, marker='*', label='Goal')
        self.ax_2d.legend()
        
        plt.ion()
        
    def setup_controls(self):
        """Setup interactive controls for obstacle placement"""
        # Create control panel
        control_panel = plt.axes([0.02, 0.02, 0.25, 0.4])
        control_panel.set_facecolor('lightgray')
        control_panel.set_title('Control Panel', fontsize=12, fontweight='bold')
        
        # Obstacle shape selection
        shape_ax = plt.axes([0.03, 0.32, 0.2, 0.1])
        self.shape_radio = RadioButtons(shape_ax, ('Cylinder', 'Cube', 'Sphere'))
        self.shape_radio.on_clicked(self.change_obstacle_shape)
        
        # Size controls
        size_ax = plt.axes([0.03, 0.22, 0.2, 0.08])
        self.size_slider = Slider(size_ax, 'Size', 0.1, 3.0, valinit=1.0, valfmt='%.1f')
        self.size_slider.on_changed(self.change_obstacle_size)
        
        # Height control
        height_ax = plt.axes([0.03, 0.12, 0.2, 0.08])
        self.height_slider = Slider(height_ax, 'Height', 0.5, 4.0, valinit=2.0, valfmt='%.1f')
        self.height_slider.on_changed(self.change_obstacle_height)
        
        # Buttons
        add_btn_ax = plt.axes([0.03, 0.05, 0.08, 0.05])
        self.add_btn = Button(add_btn_ax, 'Add Obstacle')
        self.add_btn.on_clicked(self.toggle_placement_mode)
        
        clear_btn_ax = plt.axes([0.12, 0.05, 0.08, 0.05])
        self.clear_btn = Button(clear_btn_ax, 'Clear All')
        self.clear_btn.on_clicked(self.clear_obstacles)
        
        start_btn_ax = plt.axes([0.03, 0.0, 0.17, 0.04])
        self.start_btn = Button(start_btn_ax, 'START SIMULATION')
        self.start_btn.on_clicked(self.start_simulation)
        
    def change_obstacle_shape(self, label):
        """Change obstacle shape"""
        self.current_obstacle_shape = label.lower()
        print(f"Selected obstacle shape: {self.current_obstacle_shape}")
        
    def change_obstacle_size(self, val):
        """Change obstacle size"""
        if self.current_obstacle_shape == "sphere":
            self.current_obstacle_size = (val,)
        else:
            # For cylinder and cube, preserve height
            if len(self.current_obstacle_size) >= 3:
                self.current_obstacle_size = (val, val, self.current_obstacle_size[2])
            else:
                self.current_obstacle_size = (val, val, 2.0)
        print(f"Obstacle size: {self.current_obstacle_size}")
        
    def change_obstacle_height(self, val):
        """Change obstacle height"""
        if self.current_obstacle_shape == "sphere":
            # For sphere, height is the radius
            self.current_obstacle_size = (val,)
        else:
            # For cylinder and cube, preserve width and length
            if len(self.current_obstacle_size) >= 2:
                self.current_obstacle_size = (self.current_obstacle_size[0], self.current_obstacle_size[1], val)
            else:
                self.current_obstacle_size = (1.0, 1.0, val)
        print(f"Obstacle height: {val}")
        
    def toggle_placement_mode(self, event):
        """Toggle obstacle placement mode"""
        self.placement_mode = not self.placement_mode
        if self.placement_mode:
            print("Click on the 3D plot to place obstacles!")
            self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        else:
            print("Placement mode disabled")
            self.fig.canvas.mpl_disconnect('button_press_event')
            
    def on_click(self, event):
        """Handle mouse clicks for obstacle placement"""
        if event.inaxes != self.ax or not self.placement_mode:
            return
            
        # Convert 2D click to 3D position
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return
            
        # Place obstacle at ground level
        z = 0.0
        if self.current_obstacle_shape == "sphere":
            z = self.current_obstacle_size[0] / 2  # Center sphere at half height
            
        obstacle = Obstacle3D(
            position=[x, y, z],
            shape=self.current_obstacle_shape,
            size=self.current_obstacle_size,
            color=self.current_obstacle_color
        )
        
        self.obstacles.append(obstacle)
        print(f"Added {self.current_obstacle_shape} obstacle at ({x:.2f}, {y:.2f}, {z:.2f})")
        self.update_3d_visualization()
        
    def clear_obstacles(self, event):
        """Clear all obstacles"""
        self.obstacles = []
        print("All obstacles cleared")
        self.update_3d_visualization()
        
    def start_simulation(self, event):
        """Start the drone simulation"""
        print("Starting simulation...")
        self.placement_mode = False
        self.run_simulation()
        
    def draw_obstacle_3d(self, obstacle):
        """Draw a 3D obstacle based on its shape"""
        pos = obstacle.position
        size = obstacle.size
        color = obstacle.color
        
        if obstacle.shape == "cylinder":
            radius, length, height = size
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, height, 10)
            u, v = np.meshgrid(u, v)
            
            x_obs = pos[0] + radius * np.cos(u)
            y_obs = pos[1] + radius * np.sin(u)
            z_obs = pos[2] + v
            
            self.ax_3d.plot_surface(x_obs, y_obs, z_obs, alpha=0.7, color=color)
            
        elif obstacle.shape == "cube":
            width, length, height = size
            # Create cube vertices
            vertices = np.array([
                [pos[0] - width/2, pos[1] - length/2, pos[2]],
                [pos[0] + width/2, pos[1] - length/2, pos[2]],
                [pos[0] + width/2, pos[1] + length/2, pos[2]],
                [pos[0] - width/2, pos[1] + length/2, pos[2]],
                [pos[0] - width/2, pos[1] - length/2, pos[2] + height],
                [pos[0] + width/2, pos[1] - length/2, pos[2] + height],
                [pos[0] + width/2, pos[1] + length/2, pos[2] + height],
                [pos[0] - width/2, pos[1] + length/2, pos[2] + height]
            ])
            
            # Define cube faces
            faces = [
                [0, 1, 2, 3],  # bottom
                [4, 5, 6, 7],  # top
                [0, 1, 5, 4],  # front
                [2, 3, 7, 6],  # back
                [0, 3, 7, 4],  # left
                [1, 2, 6, 5]   # right
            ]
            
            for face in faces:
                face_vertices = vertices[face]
                self.ax_3d.plot_surface(
                    face_vertices[:, 0].reshape(2, 2),
                    face_vertices[:, 1].reshape(2, 2),
                    face_vertices[:, 2].reshape(2, 2),
                    alpha=0.7, color=color
                )
                
        elif obstacle.shape == "sphere":
            radius = size[0]
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            u, v = np.meshgrid(u, v)
            
            x_obs = pos[0] + radius * np.sin(v) * np.cos(u)
            y_obs = pos[1] + radius * np.sin(v) * np.sin(u)
            z_obs = pos[2] + radius * np.cos(v)
            
            self.ax_3d.plot_surface(x_obs, y_obs, z_obs, alpha=0.7, color=color)
    
    def draw_obstacle_2d(self, obstacle):
        """Draw a 2D top-down view of obstacle"""
        pos = obstacle.position
        size = obstacle.size
        color = obstacle.color
        
        if obstacle.shape == "cylinder":
            radius, length, height = size
            circle = plt.Circle((pos[0], pos[1]), radius, color=color, alpha=0.7)
            self.ax_2d.add_patch(circle)
            
        elif obstacle.shape == "cube":
            width, length, height = size
            rect = plt.Rectangle((pos[0] - width/2, pos[1] - length/2), 
                               width, length, color=color, alpha=0.7)
            self.ax_2d.add_patch(rect)
            
        elif obstacle.shape == "sphere":
            radius = size[0]
            circle = plt.Circle((pos[0], pos[1]), radius, color=color, alpha=0.7)
            self.ax_2d.add_patch(circle)
    
    def draw_drone_3d(self):
        """Draw the drone in 3D"""
        pos = self.position
        size = self.drone_size
        
        if self.drone_shape == "triangle":
            # Draw drone as triangular arrow
            if np.linalg.norm(self.velocity) > 0:
                # Calculate arrow direction
                direction = self.velocity / np.linalg.norm(self.velocity)
                arrow_end = pos + direction * size * 2
                
                self.ax_3d.quiver(pos[0], pos[1], pos[2],
                                  direction[0], direction[1], direction[2],
                                  color='yellow', arrow_length_ratio=0.1, linewidth=3)
            else:
                # Static triangle
                self.ax_3d.scatter([pos[0]], [pos[1]], [pos[2]], 
                                   color='yellow', s=200, marker='^')
                               
        elif self.drone_shape == "cube":
            # Draw drone as small cube
            vertices = np.array([
                [pos[0] - size/2, pos[1] - size/2, pos[2]],
                [pos[0] + size/2, pos[1] - size/2, pos[2]],
                [pos[0] + size/2, pos[1] + size/2, pos[2]],
                [pos[0] - size/2, pos[1] + size/2, pos[2]],
                [pos[0] - size/2, pos[1] - size/2, pos[2] + size],
                [pos[0] + size/2, pos[1] - size/2, pos[2] + size],
                [pos[0] + size/2, pos[1] + size/2, pos[2] + size],
                [pos[0] - size/2, pos[1] + size/2, pos[2] + size]
            ])
            
            faces = [
                [0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
                [2, 3, 7, 6], [0, 3, 7, 4], [1, 2, 6, 5]
            ]
            
            for face in faces:
                face_vertices = vertices[face]
                self.ax_3d.plot_surface(
                    face_vertices[:, 0].reshape(2, 2),
                    face_vertices[:, 1].reshape(2, 2),
                    face_vertices[:, 2].reshape(2, 2),
                    alpha=0.8, color='yellow'
                )
                
        elif self.drone_shape == "sphere":
            # Draw drone as sphere
            u = np.linspace(0, 2 * np.pi, 10)
            v = np.linspace(0, np.pi, 10)
            u, v = np.meshgrid(u, v)
            
            x_drone = pos[0] + size * np.sin(v) * np.cos(u)
            y_drone = pos[1] + size * np.sin(v) * np.sin(u)
            z_drone = pos[2] + size * np.cos(v)
            
            self.ax_3d.plot_surface(x_drone, y_drone, z_drone, alpha=0.8, color='yellow')
    
    def draw_drone_2d(self):
        """Draw the drone in 2D top-down view"""
        pos = self.position
        self.ax_2d.scatter([pos[0]], [pos[1]], color='yellow', s=100, marker='^', 
                          label='Drone', zorder=10)
        
        # Draw velocity vector
        if np.linalg.norm(self.velocity) > 0:
            vel_scale = 0.5
            self.ax_2d.arrow(pos[0], pos[1], 
                           self.velocity[0] * vel_scale, self.velocity[1] * vel_scale,
                           head_width=0.1, head_length=0.1, fc='yellow', ec='yellow')
    
    def check_collision_3d(self, pos):
        """Check if position collides with 3D obstacles"""
        for obstacle in self.obstacles:
            obs_pos = obstacle.position
            size = obstacle.size
            
            if obstacle.shape == "cylinder":
                radius, length, height = size
                horizontal_dist = np.linalg.norm(pos[:2] - obs_pos[:2])
                vertical_overlap = (pos[2] >= obs_pos[2]) and (pos[2] <= obs_pos[2] + height)
                
                if horizontal_dist < radius and vertical_overlap:
                    return True
                    
            elif obstacle.shape == "cube":
                width, length, height = size
                if (abs(pos[0] - obs_pos[0]) < width/2 and
                    abs(pos[1] - obs_pos[1]) < length/2 and
                    pos[2] >= obs_pos[2] and pos[2] <= obs_pos[2] + height):
                    return True
                    
            elif obstacle.shape == "sphere":
                radius = size[0]
                distance = np.linalg.norm(pos - obs_pos)
                if distance < radius:
                    return True
        return False
    
    def update(self, dt=0.01):
        """3D drone movement with improved obstacle avoidance"""
        direction = self.target - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 0.1:
            if distance > 0:
                direction = direction / distance
            
            # Enhanced obstacle avoidance with larger safety margins
            avoidance_force = np.zeros(3)
            safety_margin = 1.5  # Increased safety margin
            
            for obstacle in self.obstacles:
                obs_pos = obstacle.position
                size = obstacle.size
                
                if obstacle.shape == "cylinder":
                    radius, length, height = size
                    horizontal_dist = np.linalg.norm(self.position[:2] - obs_pos[:2])
                    vertical_dist = abs(self.position[2] - obs_pos[2])
                    
                    # Check if we're in the obstacle's influence zone (larger than before)
                    influence_radius = radius + safety_margin
                    if horizontal_dist < influence_radius and vertical_dist < height + safety_margin:
                        avoid_direction = self.position - obs_pos
                        if np.linalg.norm(avoid_direction) > 0:
                            avoid_direction = avoid_direction / np.linalg.norm(avoid_direction)
                            # Stronger force when closer, but starts earlier
                            force_strength = max(0, (influence_radius - horizontal_dist) / influence_radius)
                            avoidance_force += avoid_direction * force_strength * 3.0  # Increased force
                            
                elif obstacle.shape == "cube":
                    width, length, height = size
                    # Check if we're near the cube (with safety margin)
                    if (abs(self.position[0] - obs_pos[0]) < width/2 + safety_margin and
                        abs(self.position[1] - obs_pos[1]) < length/2 + safety_margin and
                        abs(self.position[2] - obs_pos[2]) < height/2 + safety_margin):
                        avoid_direction = self.position - obs_pos
                        if np.linalg.norm(avoid_direction) > 0:
                            avoid_direction = avoid_direction / np.linalg.norm(avoid_direction)
                            avoidance_force += avoid_direction * 4.0  # Strong avoidance
                            
                elif obstacle.shape == "sphere":
                    radius = size[0]
                    distance_to_obs = np.linalg.norm(self.position - obs_pos)
                    influence_radius = radius + safety_margin
                    if distance_to_obs < influence_radius:
                        avoid_direction = self.position - obs_pos
                        if np.linalg.norm(avoid_direction) > 0:
                            avoid_direction = avoid_direction / np.linalg.norm(avoid_direction)
                            force_strength = max(0, (influence_radius - distance_to_obs) / influence_radius)
                            avoidance_force += avoid_direction * force_strength * 3.0
            
            # Combine movement and avoidance with better weighting
            movement_weight = 1.0
            avoidance_weight = 2.0  # Increased avoidance priority
            
            total_direction = (movement_weight * direction + avoidance_weight * avoidance_force)
            if np.linalg.norm(total_direction) > 0:
                total_direction = total_direction / np.linalg.norm(total_direction)
            
            # Adaptive speed based on proximity to obstacles
            base_speed = 1.0
            min_obstacle_dist = float('inf')
            
            for obstacle in self.obstacles:
                obs_pos = obstacle.position
                if obstacle.shape == "cylinder":
                    radius, length, height = obstacle.size
                    horizontal_dist = np.linalg.norm(self.position[:2] - obs_pos[:2])
                    min_obstacle_dist = min(min_obstacle_dist, horizontal_dist)
                elif obstacle.shape == "sphere":
                    radius = obstacle.size[0]
                    distance_to_obs = np.linalg.norm(self.position - obs_pos)
                    min_obstacle_dist = min(min_obstacle_dist, distance_to_obs)
                elif obstacle.shape == "cube":
                    width, length, height = obstacle.size
                    dist_x = abs(self.position[0] - obs_pos[0]) - width/2
                    dist_y = abs(self.position[1] - obs_pos[1]) - length/2
                    min_obstacle_dist = min(min_obstacle_dist, max(dist_x, dist_y))
            
            # Slow down when near obstacles
            if min_obstacle_dist < 2.0:
                speed_factor = max(0.3, min_obstacle_dist / 2.0)
                speed = base_speed * speed_factor
            else:
                speed = base_speed
            
            self.velocity = total_direction * speed
            
            # Update orientation
            if np.linalg.norm(self.velocity) > 0:
                self.orientation[2] = math.atan2(self.velocity[1], self.velocity[0])
                self.orientation[1] = math.atan2(-self.velocity[2], 
                                               np.linalg.norm(self.velocity[:2]))
            
            # Update position with collision check
            new_position = self.position + self.velocity * dt
            
            if not self.check_collision_3d(new_position):
                self.position = new_position
                self.trajectory.append(self.position.copy())
            else:
                # If collision detected, apply emergency avoidance
                emergency_force = np.zeros(3)
                for obstacle in self.obstacles:
                    obs_pos = obstacle.position
                    avoid_direction = self.position - obs_pos
                    if np.linalg.norm(avoid_direction) > 0:
                        avoid_direction = avoid_direction / np.linalg.norm(avoid_direction)
                        emergency_force += avoid_direction * 5.0
                
                if np.linalg.norm(emergency_force) > 0:
                    emergency_force = emergency_force / np.linalg.norm(emergency_force)
                    self.velocity = emergency_force * 0.5  # Slow emergency movement
                    self.position += self.velocity * dt
                    self.trajectory.append(self.position.copy())
                else:
                    self.velocity = np.zeros(3)
        else:
            self.velocity = np.zeros(3)
            print("Drone reached target!")
    
    def update_3d_visualization(self):
        """Update the 3D and 2D visualization"""
        # Clear both views
        self.ax_3d.clear()
        self.ax_2d.clear()
        
        # Setup both environments
        self.setup_3d_environment()
        
        # Draw obstacles in both views
        for obstacle in self.obstacles:
            self.draw_obstacle_3d(obstacle)
            self.draw_obstacle_2d(obstacle)
        
        # Draw trajectory in both views
        if len(self.trajectory) > 1:
            traj = np.array(self.trajectory)
            # 3D trajectory
            self.ax_3d.plot(traj[:, 0], traj[:, 1], traj[:, 2], 
                           color='cyan', linewidth=2, alpha=0.8, label='Trajectory')
            # 2D trajectory
            self.ax_2d.plot(traj[:, 0], traj[:, 1], 
                           color='cyan', linewidth=2, alpha=0.8, label='Trajectory')
        
        # Draw drone in both views
        self.draw_drone_3d()
        self.draw_drone_2d()
        
        # Add status text to 3D view
        distance = np.linalg.norm(self.target - self.position)
        speed = np.linalg.norm(self.velocity)
        
        self.ax_3d.text2D(0.02, 0.98, 
                          f'Distance to Goal: {distance:.2f}m\nSpeed: {speed:.2f} m/s\nObstacles: {len(self.obstacles)}', 
                          transform=self.ax_3d.transAxes, fontsize=10, 
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Add status text to 2D view
        self.ax_2d.text(0.02, 0.98, 
                       f'Drone Position: ({self.position[0]:.2f}, {self.position[1]:.2f})\nSpeed: {speed:.2f} m/s', 
                       transform=self.ax_2d.transAxes, fontsize=9, 
                       bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8),
                       verticalalignment='top')
        
        plt.draw()
        plt.pause(0.01)
    
    def run_simulation(self):
        """Run the complete simulation"""
        print("Running simulation...")
        start_time = time.time()
        collision_count = 0
        mission_completed = False
        
        try:
            while True:
                current_time = time.time()
                dt = min(0.01, current_time - start_time)
                start_time = current_time
                
                # Update drone
                self.update(dt)
                
                # Update visualization
                self.update_3d_visualization()
                
                # Check collisions
                if self.check_collision_3d(self.position):
                    collision_count += 1
                    print(f"Collision detected! Total: {collision_count}")
                
                # Check if reached target
                distance = np.linalg.norm(self.target - self.position)
                if distance < 0.1 and not mission_completed:
                    print("Mission accomplished! Drone reached the target!")
                    print(f"Total collisions: {collision_count}")
                    mission_completed = True
                    
                    # Add completion message to visualization
                    self.ax_3d.text2D(0.5, 0.5, 'MISSION COMPLETED!', 
                                    transform=self.ax_3d.transAxes, fontsize=20, 
                                    bbox=dict(boxstyle='round', facecolor='green', alpha=0.8),
                                    ha='center', va='center')
                    self.ax_2d.text(0.5, 0.5, 'MISSION COMPLETED!', 
                                   transform=self.ax_2d.transAxes, fontsize=16, 
                                   bbox=dict(boxstyle='round', facecolor='green', alpha=0.8),
                                   ha='center', va='center')
                    plt.draw()
                
                # Keep GUI open after mission completion
                if mission_completed:
                    # Continue updating visualization but don't move drone
                    time.sleep(0.1)
                    continue
                
                # Check if window is closed
                if not plt.get_fignums():
                    print("Window closed by user")
                    break
                    
        except KeyboardInterrupt:
            print("\nSimulation stopped by user")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            # Keep the window open for analysis
            print("Simulation ended. Close the window manually to exit.")
            try:
                plt.show(block=True)  # Block until window is closed
            except:
                pass

def main():
    print("=== ENHANCED 3D DRONE NAVIGATION ===")
    print("Features:")
    print("- Interactive obstacle placement")
    print("- Multiple obstacle shapes (cylinder, cube, sphere)")
    print("- Customizable sizes and heights")
    print("- Real-time 3D visualization")
    print("- 2D top-down view")
    print("- Improved obstacle avoidance")
    print()
    
    # Set start and end points
    start_world = np.array([-3.0, -2.0, 1.0])
    end_world = np.array([3.0, 2.0, 2.0])
    
    # Create enhanced drone
    drone = Enhanced3DDrone(start_world, end_world)
    
    print("Instructions:")
    print("1. Use the control panel to select obstacle shape and size")
    print("2. Click 'Add Obstacle' to enter placement mode")
    print("3. Click on the 3D plot to place obstacles")
    print("4. Click 'START SIMULATION' when ready")
    print("5. GUI will stay open after mission completion")
    print("6. Close the window manually to exit")
    print()
    
    try:
        # Show the interactive plot and keep it open
        plt.show(block=True)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Program ended")

if __name__ == "__main__":
    main()
