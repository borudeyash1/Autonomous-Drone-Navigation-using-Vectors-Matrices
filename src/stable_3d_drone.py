import numpy as np
import cv2
import time
import math
from typing import List, Dict, Tuple
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for better compatibility
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Obstacle3D:
    def __init__(self, position, shape="cylinder", size=(1.0, 1.0, 2.0), color='red'):
        self.position = np.array(position)
        self.shape = shape  # "cylinder", "cube", "sphere"
        self.size = size  # (radius/width, length, height) for cylinder/cube, (radius) for sphere
        self.color = color
        self.id = None

class Stable3DDrone:
    def __init__(self, start_pos, end_pos):
        self.position = np.array(start_pos, dtype=float)
        self.target = np.array(end_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])
        self.trajectory = [self.position.copy()]
        self.obstacles = []
        self.drone_shape = "triangle"
        self.drone_size = 0.3
        
        # Obstacle editing mode
        self.edit_mode = False
        self.selected_obstacle = None
        self.drag_start_pos = None
        
        # Simulation control
        self.simulation_paused = False
        self.simulation_running = False
        
        # Speed control
        self.base_speed = 1.0  # Base speed in m/s
        self.speed_multiplier = 1.0  # Speed multiplier for real-time adjustment
        self.min_speed = 0.1  # Minimum speed
        self.max_speed = 5.0  # Maximum speed
        
        # 3D visualization setup
        self.fig = plt.figure(figsize=(15, 10))
        self.ax_3d = self.fig.add_subplot(121, projection='3d')
        self.ax_2d = self.fig.add_subplot(122)
        
        self.setup_3d_environment()
        
        # Simple controls
        self.setup_simple_controls()
        
        # Mouse event handlers for obstacle editing
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        
    def setup_3d_environment(self):
        """Setup the 3D environment visualization with 2D overlay"""
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
        
    def setup_simple_controls(self):
        """Setup simple keyboard controls"""
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        # Add some default obstacles for testing
        self.add_default_obstacles()
        
    def add_default_obstacles(self):
        """Add some default obstacles for testing"""
        obstacles = [
            Obstacle3D([0.0, 0.0, 0.5], "cylinder", (1.0, 1.0, 2.0), 'red'),
            Obstacle3D([-2.0, 1.0, 0.5], "cube", (1.5, 1.5, 2.5), 'blue'),
            Obstacle3D([2.0, -1.0, 0.5], "sphere", (1.0,), 'green')
        ]
        self.obstacles.extend(obstacles)
        
    def on_key_press(self, event):
        """Handle keyboard input"""
        if event.key == ' ':
            self.start_simulation()
        elif event.key == 'c':
            self.obstacles.clear()
            print("All obstacles cleared")
            self.update_visualization()
        elif event.key == 'r':
            self.reset_drone()
            self.update_visualization()
        elif event.key == 'q':
            plt.close('all')
        elif event.key == 'd':
            self.delete_last_obstacle()
        elif event.key == '1':
            self.add_obstacle_at_position("cylinder", (1.0, 1.0, 2.0), 'red')
        elif event.key == '2':
            self.add_obstacle_at_position("cube", (1.5, 1.5, 2.5), 'blue')
        elif event.key == '3':
            self.add_obstacle_at_position("sphere", (1.0,), 'green')
        elif event.key == 'h':
            self.show_help()
        elif event.key == 'e':
            self.toggle_edit_mode()
        elif event.key == 's':
            self.select_nearest_obstacle()
        elif event.key == 'delete':
            self.delete_selected_obstacle()
        elif event.key == 'p':
            self.toggle_pause()
        elif event.key == '+':
            self.increase_speed()
        elif event.key == '-':
            self.decrease_speed()
        elif event.key == '=':
            self.reset_speed()
        elif event.key == '0':
            self.set_speed(0.5)
        elif event.key == '9':
            self.set_speed(1.0)
        elif event.key == '8':
            self.set_speed(1.5)
        elif event.key == '7':
            self.set_speed(2.0)
            
    def delete_last_obstacle(self):
        """Delete the last added obstacle"""
        if self.obstacles:
            removed = self.obstacles.pop()
            print(f"Removed {removed.shape} obstacle at ({removed.position[0]:.2f}, {removed.position[1]:.2f})")
            self.update_visualization()
        else:
            print("No obstacles to remove")
            
    def add_obstacle_at_position(self, shape, size, color):
        """Add obstacle at center position"""
        center_pos = np.array([0.0, 0.0, 0.5])
        
        # Find a safe position near center
        safe_positions = [
            np.array([0.0, 0.0, 0.5]),
            np.array([1.0, 1.0, 0.5]),
            np.array([-1.0, -1.0, 0.5]),
            np.array([2.0, 0.0, 0.5]),
            np.array([-2.0, 0.0, 0.5]),
            np.array([0.0, 2.0, 0.5]),
            np.array([0.0, -2.0, 0.5])
        ]
        
        for pos in safe_positions:
            collision = False
            for obstacle in self.obstacles:
                if self.check_collision_3d(pos):
                    collision = True
                    break
            
            if not collision:
                obstacle = Obstacle3D(pos, shape, size, color)
                self.obstacles.append(obstacle)
                print(f"Added {shape} obstacle at ({pos[0]:.2f}, {pos[1]:.2f})")
                self.update_visualization()
                return
        
        print(f"Cannot find safe position for {shape} obstacle")
        
    def toggle_edit_mode(self):
        """Toggle obstacle editing mode"""
        self.edit_mode = not self.edit_mode
        if self.edit_mode:
            print("Edit mode ON - Click and drag obstacles to move them")
            self.selected_obstacle = None
        else:
            print("Edit mode OFF - Click to add obstacles")
            self.selected_obstacle = None
            
    def select_nearest_obstacle(self):
        """Select the obstacle nearest to drone"""
        if not self.obstacles:
            print("No obstacles to select")
            return
            
        min_distance = float('inf')
        nearest_obstacle = None
        
        for obstacle in self.obstacles:
            distance = np.linalg.norm(self.position - obstacle.position)
            if distance < min_distance:
                min_distance = distance
                nearest_obstacle = obstacle
                
        if nearest_obstacle:
            self.selected_obstacle = nearest_obstacle
            print(f"Selected {nearest_obstacle.shape} obstacle at ({nearest_obstacle.position[0]:.2f}, {nearest_obstacle.position[1]:.2f})")
            self.update_visualization()
            
    def delete_selected_obstacle(self):
        """Delete the currently selected obstacle"""
        if self.selected_obstacle and self.selected_obstacle in self.obstacles:
            self.obstacles.remove(self.selected_obstacle)
            print(f"Deleted {self.selected_obstacle.shape} obstacle")
            self.selected_obstacle = None
            self.update_visualization()
        else:
            print("No obstacle selected")
            
    def toggle_pause(self):
        """Toggle simulation pause/resume"""
        if self.simulation_running:
            self.simulation_paused = not self.simulation_paused
            if self.simulation_paused:
                print("Simulation PAUSED - You can now add/edit obstacles")
                print("Press P again to resume, or SPACE to restart")
            else:
                print("Simulation RESUMED")
        else:
            print("Simulation not running. Press SPACE to start.")
            
    def increase_speed(self):
        """Increase drone speed"""
        self.speed_multiplier = min(self.speed_multiplier + 0.2, self.max_speed / self.base_speed)
        current_speed = self.base_speed * self.speed_multiplier
        print(f"Speed increased to {current_speed:.1f} m/s")
        
    def decrease_speed(self):
        """Decrease drone speed"""
        self.speed_multiplier = max(self.speed_multiplier - 0.2, self.min_speed / self.base_speed)
        current_speed = self.base_speed * self.speed_multiplier
        print(f"Speed decreased to {current_speed:.1f} m/s")
        
    def reset_speed(self):
        """Reset speed to default"""
        self.speed_multiplier = 1.0
        print(f"Speed reset to {self.base_speed:.1f} m/s")
        
    def set_speed(self, speed):
        """Set specific speed"""
        self.speed_multiplier = max(min(speed / self.base_speed, self.max_speed / self.base_speed), 
                                   self.min_speed / self.base_speed)
        current_speed = self.base_speed * self.speed_multiplier
        print(f"Speed set to {current_speed:.1f} m/s")
            
    def find_obstacle_at_position(self, pos):
        """Find obstacle at given position"""
        for obstacle in self.obstacles:
            if obstacle.shape == "cylinder":
                radius, length, height = obstacle.size
                horizontal_dist = np.linalg.norm(pos[:2] - obstacle.position[:2])
                if horizontal_dist < radius + 0.5:  # Selection tolerance
                    return obstacle
            elif obstacle.shape == "cube":
                width, length, height = obstacle.size
                if (abs(pos[0] - obstacle.position[0]) < width/2 + 0.5 and
                    abs(pos[1] - obstacle.position[1]) < length/2 + 0.5):
                    return obstacle
            elif obstacle.shape == "sphere":
                radius = obstacle.size[0]
                distance = np.linalg.norm(pos - obstacle.position)
                if distance < radius + 0.5:
                    return obstacle
        return None
        
    def show_help(self):
        """Show help information"""
        print("\n=== CONTROLS ===")
        print("SPACE: Start simulation")
        print("P: Pause/Resume simulation")
        print("C: Clear all obstacles")
        print("R: Reset drone to start")
        print("D: Delete last obstacle")
        print("1: Add cylinder obstacle")
        print("2: Add cube obstacle")
        print("3: Add sphere obstacle")
        print("E: Toggle edit mode")
        print("S: Select nearest obstacle")
        print("DELETE: Delete selected obstacle")
        print("H: Show this help")
        print("Q: Quit")
        print("Click: Add obstacle (or select in edit mode)")
        print("Drag: Move selected obstacle (in edit mode)")
        print("\n=== SPEED CONTROLS ===")
        print("+: Increase speed")
        print("-: Decrease speed")
        print("=: Reset speed to default")
        print("0: Set speed to 0.5 m/s")
        print("9: Set speed to 1.0 m/s")
        print("8: Set speed to 1.5 m/s")
        print("7: Set speed to 2.0 m/s")
        print("\n=== SIMULATION FEATURES ===")
        print("- Pause simulation to add/edit obstacles")
        print("- Drone will avoid obstacles automatically")
        print("- Path replanning when obstacles are added")
        print("- Smart collision detection prevents overlapping")
        print("- Real-time speed adjustment")
        print("================\n")
            
    def on_click(self, event):
        """Handle mouse clicks for obstacle placement"""
        if event.inaxes == self.ax_3d:
            # Convert 3D click to position
            x, y = event.xdata, event.ydata
            if x is not None and y is not None:
                # Check if click is within bounds
                if -5 <= x <= 5 and -5 <= y <= 5:
                    # Check for collision with existing obstacles
                    click_pos = np.array([x, y, 0.5])
                    
                    # Cycle through obstacle types
                    obstacle_types = ["cylinder", "cube", "sphere"]
                    obstacle_colors = ['red', 'blue', 'green']
                    obstacle_sizes = [(1.0, 1.0, 2.0), (1.5, 1.5, 2.5), (1.0,)]
                    
                    obstacle_type = obstacle_types[len(self.obstacles) % len(obstacle_types)]
                    obstacle_color = obstacle_colors[len(self.obstacles) % len(obstacle_colors)]
                    obstacle_size = obstacle_sizes[len(self.obstacles) % len(obstacle_sizes)]
                    
                    if not self.check_obstacle_placement_collision(click_pos, obstacle_type, obstacle_size):
                        obstacle = Obstacle3D([x, y, 0.5], obstacle_type, obstacle_size, obstacle_color)
                        self.obstacles.append(obstacle)
                        print(f"Added {obstacle_type} obstacle at ({x:.2f}, {y:.2f})")
                        
                        # Replan path if simulation is running
                        if self.simulation_running:
                            self.replan_path()
                            
                        self.update_visualization()
                    else:
                        print(f"Cannot place obstacle at ({x:.2f}, {y:.2f}) - too close to existing obstacle/drone/waypoints")
                else:
                    print("Click must be within the environment bounds (-5 to 5)")
                    
        elif event.inaxes == self.ax_2d:
            # Also allow clicking on 2D view
            x, y = event.xdata, event.ydata
            if x is not None and y is not None:
                if -5 <= x <= 5 and -5 <= y <= 5:
                    click_pos = np.array([x, y, 0.5])
                    
                    # Cycle through obstacle types
                    obstacle_types = ["cylinder", "cube", "sphere"]
                    obstacle_colors = ['red', 'blue', 'green']
                    obstacle_sizes = [(1.0, 1.0, 2.0), (1.5, 1.5, 2.5), (1.0,)]
                    
                    obstacle_type = obstacle_types[len(self.obstacles) % len(obstacle_types)]
                    obstacle_color = obstacle_colors[len(self.obstacles) % len(obstacle_colors)]
                    obstacle_size = obstacle_sizes[len(self.obstacles) % len(obstacle_sizes)]
                    
                    if not self.check_obstacle_placement_collision(click_pos, obstacle_type, obstacle_size):
                        obstacle = Obstacle3D([x, y, 0.5], obstacle_type, obstacle_size, obstacle_color)
                        self.obstacles.append(obstacle)
                        print(f"Added {obstacle_type} obstacle at ({x:.2f}, {y:.2f})")
                        
                        # Replan path if simulation is running
                        if self.simulation_running:
                            self.replan_path()
                            
                        self.update_visualization()
                    else:
                        print(f"Cannot place obstacle at ({x:.2f}, {y:.2f}) - too close to existing obstacle/drone/waypoints")
                else:
                    print("Click must be within the environment bounds (-5 to 5)")
                    
    def on_mouse_press(self, event):
        """Handle mouse press for obstacle selection and dragging"""
        if event.inaxes == self.ax_3d or event.inaxes == self.ax_2d:
            x, y = event.xdata, event.ydata
            if x is not None and y is not None:
                click_pos = np.array([x, y, 0.5])
                
                if self.edit_mode:
                    # In edit mode, try to select obstacle
                    obstacle = self.find_obstacle_at_position(click_pos)
                    if obstacle:
                        self.selected_obstacle = obstacle
                        self.drag_start_pos = click_pos.copy()
                        print(f"Selected {obstacle.shape} obstacle for dragging")
                        self.update_visualization()
                    else:
                        print("No obstacle found at clicked position")
                else:
                    # In add mode, add obstacle (existing functionality)
                    self.on_click(event)
                    
    def on_mouse_release(self, event):
        """Handle mouse release"""
        if self.selected_obstacle and self.drag_start_pos is not None:
            self.drag_start_pos = None
            print(f"Released {self.selected_obstacle.shape} obstacle")
            
    def on_mouse_move(self, event):
        """Handle mouse movement for obstacle dragging"""
        if (self.selected_obstacle and self.drag_start_pos is not None and 
            event.inaxes and event.xdata is not None and event.ydata is not None):
            
            # Calculate new position
            new_pos = np.array([event.xdata, event.ydata, 0.5])
            
            # Check bounds
            if -5 <= new_pos[0] <= 5 and -5 <= new_pos[1] <= 5:
                # Check if new position is safe
                old_pos = self.selected_obstacle.position.copy()
                self.selected_obstacle.position = new_pos
                
                # Check for collisions with other obstacles
                collision = False
                for obstacle in self.obstacles:
                    if obstacle != self.selected_obstacle:
                        if self.check_collision_3d(new_pos, safety_margin=0.5):
                            collision = True
                            break
                
                # Check collision with drone and waypoints
                drone_distance = np.linalg.norm(new_pos - self.position)
                start_distance = np.linalg.norm(new_pos - np.array([-3.0, -2.0, 1.0]))
                end_distance = np.linalg.norm(new_pos - np.array([3.0, 2.0, 2.0]))
                
                if (collision or drone_distance < 1.0 or 
                    start_distance < 1.0 or end_distance < 1.0):
                    # Revert position if collision
                    self.selected_obstacle.position = old_pos
                else:
                    # Update visualization
                    self.update_visualization()
                
    def start_simulation(self):
        """Start the drone simulation"""
        print("Starting simulation...")
        self.run_simulation()
        
    def reset_drone(self):
        """Reset drone to start position"""
        self.position = np.array([-3.0, -2.0, 1.0])
        self.velocity = np.zeros(3)
        self.trajectory = [self.position.copy()]
        print("Drone reset to start position")
        
    def draw_obstacle_3d(self, obstacle):
        """Draw a 3D obstacle based on its shape"""
        pos = obstacle.position
        size = obstacle.size
        color = obstacle.color
        
        # Highlight selected obstacle
        if obstacle == self.selected_obstacle:
            color = 'yellow'
            alpha = 0.9
        else:
            alpha = 0.7
        
        if obstacle.shape == "cylinder":
            radius, length, height = size
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, height, 10)
            u, v = np.meshgrid(u, v)
            
            x_obs = pos[0] + radius * np.cos(u)
            y_obs = pos[1] + radius * np.sin(u)
            z_obs = pos[2] + v
            
            self.ax_3d.plot_surface(x_obs, y_obs, z_obs, alpha=alpha, color=color)
            
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
                [0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
                [2, 3, 7, 6], [0, 3, 7, 4], [1, 2, 6, 5]
            ]
            
            for face in faces:
                face_vertices = vertices[face]
                self.ax_3d.plot_surface(
                    face_vertices[:, 0].reshape(2, 2),
                    face_vertices[:, 1].reshape(2, 2),
                    face_vertices[:, 2].reshape(2, 2),
                    alpha=alpha, color=color
                )
                
        elif obstacle.shape == "sphere":
            radius = size[0]
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            u, v = np.meshgrid(u, v)
            
            x_obs = pos[0] + radius * np.sin(v) * np.cos(u)
            y_obs = pos[1] + radius * np.sin(v) * np.sin(u)
            z_obs = pos[2] + radius * np.cos(v)
            
            self.ax_3d.plot_surface(x_obs, y_obs, z_obs, alpha=alpha, color=color)
    
    def draw_obstacle_2d(self, obstacle):
        """Draw a 2D top-down view of obstacle"""
        pos = obstacle.position
        size = obstacle.size
        color = obstacle.color
        
        # Highlight selected obstacle
        if obstacle == self.selected_obstacle:
            color = 'yellow'
            alpha = 0.9
        else:
            alpha = 0.7
        
        if obstacle.shape == "cylinder":
            radius, length, height = size
            circle = plt.Circle((pos[0], pos[1]), radius, color=color, alpha=alpha)
            self.ax_2d.add_patch(circle)
            
        elif obstacle.shape == "cube":
            width, length, height = size
            rect = plt.Rectangle((pos[0] - width/2, pos[1] - length/2), 
                               width, length, color=color, alpha=alpha)
            self.ax_2d.add_patch(rect)
            
        elif obstacle.shape == "sphere":
            radius = size[0]
            circle = plt.Circle((pos[0], pos[1]), radius, color=color, alpha=alpha)
            self.ax_2d.add_patch(circle)
    
    def draw_drone_3d(self):
        """Draw the drone in 3D"""
        pos = self.position
        size = self.drone_size
        
        # Draw drone as triangular arrow
        if np.linalg.norm(self.velocity) > 0:
            direction = self.velocity / np.linalg.norm(self.velocity)
            self.ax_3d.quiver(pos[0], pos[1], pos[2],
                              direction[0], direction[1], direction[2],
                              color='yellow', arrow_length_ratio=0.1, linewidth=3)
        else:
            self.ax_3d.scatter([pos[0]], [pos[1]], [pos[2]], 
                               color='yellow', s=200, marker='^')
    
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
    
    def check_collision_3d(self, pos, safety_margin=0.0):
        """Check if position collides with 3D obstacles"""
        for obstacle in self.obstacles:
            obs_pos = obstacle.position
            size = obstacle.size
            
            if obstacle.shape == "cylinder":
                radius, length, height = size
                horizontal_dist = np.linalg.norm(pos[:2] - obs_pos[:2])
                vertical_overlap = (pos[2] >= obs_pos[2] - safety_margin) and (pos[2] <= obs_pos[2] + height + safety_margin)
                
                if horizontal_dist < radius + safety_margin and vertical_overlap:
                    return True
                    
            elif obstacle.shape == "cube":
                width, length, height = size
                if (abs(pos[0] - obs_pos[0]) < width/2 + safety_margin and
                    abs(pos[1] - obs_pos[1]) < length/2 + safety_margin and
                    pos[2] >= obs_pos[2] - safety_margin and pos[2] <= obs_pos[2] + height + safety_margin):
                    return True
                    
            elif obstacle.shape == "sphere":
                radius = size[0]
                distance = np.linalg.norm(pos - obs_pos)
                if distance < radius + safety_margin:
                    return True
        return False
        
    def check_obstacle_placement_collision(self, pos, obstacle_shape, obstacle_size):
        """Check if placing an obstacle at pos would collide with existing obstacles or drone"""
        # Check collision with existing obstacles (with safety margin)
        if self.check_collision_3d(pos, safety_margin=0.5):
            return True
            
        # Check collision with drone (more restrictive when simulation is running)
        drone_distance = np.linalg.norm(pos - self.position)
        min_drone_distance = 1.5 if self.simulation_running else 1.0
        if drone_distance < min_drone_distance:
            return True
            
        # Check collision with start/end points
        start_distance = np.linalg.norm(pos - np.array([-3.0, -2.0, 1.0]))
        end_distance = np.linalg.norm(pos - np.array([3.0, 2.0, 2.0]))
        
        if start_distance < 1.0 or end_distance < 1.0:
            return True
            
        return False
    
    def replan_path(self):
        """Replan the path when obstacles are added/changed"""
        # Simple path replanning - find alternative route
        if len(self.obstacles) > 0:
            # Calculate alternative waypoints
            current_pos = self.position.copy()
            target_pos = self.target.copy()
            
            # Find the best path around obstacles
            waypoints = self.find_waypoints_around_obstacles(current_pos, target_pos)
            
            # Update trajectory with new waypoints
            if waypoints:
                self.trajectory.extend(waypoints)
                print(f"Path replanned with {len(waypoints)} waypoints")
                
    def find_waypoints_around_obstacles(self, start, end):
        """Find waypoints to navigate around obstacles"""
        waypoints = []
        
        # Simple strategy: find points that avoid obstacles
        direction = end - start
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            direction = direction / distance
            
            # Check for obstacles in the direct path
            num_checks = int(distance * 2)
            for i in range(1, num_checks):
                check_pos = start + direction * (distance * i / num_checks)
                
                # If obstacle detected, find alternative path
                if self.check_collision_3d(check_pos, safety_margin=1.0):
                    # Find alternative waypoint
                    alt_waypoint = self.find_alternative_waypoint(check_pos)
                    if alt_waypoint is not None:
                        waypoints.append(alt_waypoint)
                        
        return waypoints
        
    def find_alternative_waypoint(self, blocked_pos):
        """Find an alternative waypoint around a blocked position"""
        # Try different directions around the blocked position
        directions = [
            np.array([1.0, 0.0, 0.0]),   # Right
            np.array([-1.0, 0.0, 0.0]),  # Left
            np.array([0.0, 1.0, 0.0]),   # Forward
            np.array([0.0, -1.0, 0.0]),  # Backward
            np.array([1.0, 1.0, 0.0]),   # Diagonal
            np.array([-1.0, 1.0, 0.0]),  # Diagonal
        ]
        
        for direction in directions:
            alt_pos = blocked_pos + direction * 2.0  # 2 meter offset
            if not self.check_collision_3d(alt_pos, safety_margin=1.0):
                return alt_pos
                
        return None
    
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
            base_speed = self.base_speed * self.speed_multiplier
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
    
    def update_visualization(self):
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
        
        mode_text = "EDIT MODE" if self.edit_mode else "ADD MODE"
        selected_text = f"Selected: {self.selected_obstacle.shape}" if self.selected_obstacle else "No selection"
        pause_text = "PAUSED" if self.simulation_paused else "RUNNING" if self.simulation_running else "STOPPED"
        current_speed = self.base_speed * self.speed_multiplier
        
        self.ax_3d.text2D(0.02, 0.98, 
                          f'Mode: {mode_text}\n{selected_text}\nSimulation: {pause_text}\nDistance to Goal: {distance:.2f}m\nCurrent Speed: {current_speed:.1f} m/s\nActual Speed: {speed:.2f} m/s\nObstacles: {len(self.obstacles)}', 
                          transform=self.ax_3d.transAxes, fontsize=10, 
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Add status text to 2D view
        self.ax_2d.text(0.02, 0.98, 
                       f'Drone Position: ({self.position[0]:.2f}, {self.position[1]:.2f})\nSpeed: {speed:.2f} m/s', 
                       transform=self.ax_2d.transAxes, fontsize=9, 
                       bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8),
                       verticalalignment='top')
        
        # Add controls info
        controls_text = ('Controls: SPACE=Start, P=Pause/Resume, C=Clear, R=Reset\n'
                        '1=Cylinder, 2=Cube, 3=Sphere, E=Edit Mode, S=Select\n'
                        'D=Delete Last, H=Help, Q=Quit, Click=Add/Select, Drag=Move\n'
                        'Speed: +/- = Adjust, = = Reset, 0-9 = Preset Speeds')
        self.ax_2d.text(0.02, 0.02, controls_text, 
                       transform=self.ax_2d.transAxes, fontsize=7, 
                       bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8),
                       verticalalignment='bottom')
        
        # Add obstacle info
        if self.obstacles:
            obstacle_info = f"Obstacles: {len(self.obstacles)}"
            for i, obs in enumerate(self.obstacles[-3:]):  # Show last 3 obstacles
                obstacle_info += f"\n{i+1}. {obs.shape} at ({obs.position[0]:.1f}, {obs.position[1]:.1f})"
            if len(self.obstacles) > 3:
                obstacle_info += f"\n... and {len(self.obstacles) - 3} more"
            
            self.ax_2d.text(0.98, 0.02, obstacle_info, 
                           transform=self.ax_2d.transAxes, fontsize=7, 
                           bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8),
                           verticalalignment='bottom', horizontalalignment='right')
        
        plt.draw()
        plt.pause(0.01)
    
    def run_simulation(self):
        """Run the complete simulation"""
        print("Running simulation...")
        self.simulation_running = True
        self.simulation_paused = False
        start_time = time.time()
        collision_count = 0
        mission_completed = False
        
        try:
            while True:
                current_time = time.time()
                dt = min(0.01, current_time - start_time)
                start_time = current_time
                
                # Update drone only if not paused
                if not self.simulation_paused:
                    self.update(dt)
                
                # Always update visualization
                self.update_visualization()
                
                # Check collisions only if not paused
                if not self.simulation_paused:
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
            self.simulation_running = False
            # Keep the window open for analysis
            print("Simulation ended. Close the window manually to exit.")
            try:
                plt.show(block=True)  # Block until window is closed
            except:
                pass

def main():
    print("=== STABLE 3D DRONE NAVIGATION ===")
    print("Features:")
    print("- Interactive obstacle placement")
    print("- Multiple obstacle shapes (cylinder, cube, sphere)")
    print("- Real-time 3D visualization")
    print("- 2D top-down view")
    print("- Improved obstacle avoidance")
    print("- Stable GUI that stays open")
    print()
    
    # Set start and end points
    start_world = np.array([-3.0, -2.0, 1.0])
    end_world = np.array([3.0, 2.0, 2.0])
    
    # Create stable drone
    drone = Stable3DDrone(start_world, end_world)
    
    print("Instructions:")
    print("- SPACE: Start simulation")
    print("- P: Pause/Resume simulation (add obstacles while paused)")
    print("- C: Clear all obstacles")
    print("- R: Reset drone to start")
    print("- D: Delete last obstacle")
    print("- 1: Add cylinder obstacle")
    print("- 2: Add cube obstacle")
    print("- 3: Add sphere obstacle")
    print("- E: Toggle edit mode (drag obstacles)")
    print("- S: Select nearest obstacle")
    print("- DELETE: Delete selected obstacle")
    print("- H: Show help")
    print("- Q: Quit")
    print("- Click: Add obstacle (or select in edit mode)")
    print("- Drag: Move selected obstacle (in edit mode)")
    print("- GUI will stay open after mission completion")
    print()
    print("SPEED CONTROLS:")
    print("- +/-: Increase/Decrease speed")
    print("- =: Reset speed to default (1.0 m/s)")
    print("- 0: Set speed to 0.5 m/s (slow)")
    print("- 9: Set speed to 1.0 m/s (normal)")
    print("- 8: Set speed to 1.5 m/s (fast)")
    print("- 7: Set speed to 2.0 m/s (very fast)")
    print()
    print("FEATURES:")
    print("- Real-time obstacle editing during simulation")
    print("- Smart collision detection prevents drone overlap")
    print("- Automatic path replanning around new obstacles")
    print("- Pause simulation to safely add obstacles")
    print("- Real-time speed adjustment")
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
