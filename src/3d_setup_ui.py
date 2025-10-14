import numpy as np
import cv2
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from stable_3d_drone import Stable3DDrone

class Drone3DSetupUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("3D Drone Environment Setup")
        self.root.geometry("800x600")
        
        # Default settings
        self.start_pos = np.array([-3.0, -2.0, 1.0])
        self.end_pos = np.array([3.0, 2.0, 2.0])
        self.obstacles = []
        self.drone_shape = "triangle"
        
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the user interface"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="3D Drone Environment Setup", 
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # Start position
        ttk.Label(main_frame, text="Start Position:").grid(row=1, column=0, sticky=tk.W, pady=5)
        start_frame = ttk.Frame(main_frame)
        start_frame.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(start_frame, text="X:").grid(row=0, column=0, padx=(0, 5))
        self.start_x = tk.DoubleVar(value=-3.0)
        ttk.Entry(start_frame, textvariable=self.start_x, width=8).grid(row=0, column=1, padx=(0, 10))
        
        ttk.Label(start_frame, text="Y:").grid(row=0, column=2, padx=(0, 5))
        self.start_y = tk.DoubleVar(value=-2.0)
        ttk.Entry(start_frame, textvariable=self.start_y, width=8).grid(row=0, column=3, padx=(0, 10))
        
        ttk.Label(start_frame, text="Z:").grid(row=0, column=4, padx=(0, 5))
        self.start_z = tk.DoubleVar(value=1.0)
        ttk.Entry(start_frame, textvariable=self.start_z, width=8).grid(row=0, column=5)
        
        # End position
        ttk.Label(main_frame, text="End Position:").grid(row=2, column=0, sticky=tk.W, pady=5)
        end_frame = ttk.Frame(main_frame)
        end_frame.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(end_frame, text="X:").grid(row=0, column=0, padx=(0, 5))
        self.end_x = tk.DoubleVar(value=3.0)
        ttk.Entry(end_frame, textvariable=self.end_x, width=8).grid(row=0, column=1, padx=(0, 10))
        
        ttk.Label(end_frame, text="Y:").grid(row=0, column=2, padx=(0, 5))
        self.end_y = tk.DoubleVar(value=2.0)
        ttk.Entry(end_frame, textvariable=self.end_y, width=8).grid(row=0, column=3, padx=(0, 10))
        
        ttk.Label(end_frame, text="Z:").grid(row=0, column=4, padx=(0, 5))
        self.end_z = tk.DoubleVar(value=2.0)
        ttk.Entry(end_frame, textvariable=self.end_z, width=8).grid(row=0, column=5)
        
        # Drone shape
        ttk.Label(main_frame, text="Drone Shape:").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.drone_shape_var = tk.StringVar(value="triangle")
        drone_combo = ttk.Combobox(main_frame, textvariable=self.drone_shape_var, 
                                  values=["triangle", "cube", "sphere"], state="readonly")
        drone_combo.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=5)
        
        # Obstacle management
        ttk.Label(main_frame, text="Obstacles:", font=("Arial", 12, "bold")).grid(row=4, column=0, sticky=tk.W, pady=(20, 5))
        
        # Obstacle list
        obstacle_frame = ttk.Frame(main_frame)
        obstacle_frame.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Obstacle listbox
        list_frame = ttk.Frame(obstacle_frame)
        list_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=(0, 10))
        
        ttk.Label(list_frame, text="Current Obstacles:").grid(row=0, column=0, sticky=tk.W)
        self.obstacle_listbox = tk.Listbox(list_frame, height=8, width=40)
        self.obstacle_listbox.grid(row=1, column=0, sticky=(tk.W, tk.E))
        
        # Obstacle controls
        control_frame = ttk.Frame(obstacle_frame)
        control_frame.grid(row=0, column=1, sticky=(tk.W, tk.E))
        
        # Add obstacle
        ttk.Label(control_frame, text="Add Obstacle:").grid(row=0, column=0, sticky=tk.W, pady=(0, 5))
        
        # Shape
        ttk.Label(control_frame, text="Shape:").grid(row=1, column=0, sticky=tk.W)
        self.obs_shape = tk.StringVar(value="cylinder")
        ttk.Combobox(control_frame, textvariable=self.obs_shape, 
                     values=["cylinder", "cube", "sphere"], state="readonly", width=10).grid(row=1, column=1, pady=2)
        
        # Position
        pos_frame = ttk.Frame(control_frame)
        pos_frame.grid(row=2, column=0, columnspan=2, pady=5)
        
        ttk.Label(pos_frame, text="X:").grid(row=0, column=0, padx=(0, 2))
        self.obs_x = tk.DoubleVar(value=0.0)
        ttk.Entry(pos_frame, textvariable=self.obs_x, width=6).grid(row=0, column=1, padx=(0, 5))
        
        ttk.Label(pos_frame, text="Y:").grid(row=0, column=2, padx=(0, 2))
        self.obs_y = tk.DoubleVar(value=0.0)
        ttk.Entry(pos_frame, textvariable=self.obs_y, width=6).grid(row=0, column=3, padx=(0, 5))
        
        ttk.Label(pos_frame, text="Z:").grid(row=0, column=4, padx=(0, 2))
        self.obs_z = tk.DoubleVar(value=0.0)
        ttk.Entry(pos_frame, textvariable=self.obs_z, width=6).grid(row=0, column=5)
        
        # Size
        size_frame = ttk.Frame(control_frame)
        size_frame.grid(row=3, column=0, columnspan=2, pady=5)
        
        ttk.Label(size_frame, text="Size:").grid(row=0, column=0, padx=(0, 2))
        self.obs_size = tk.DoubleVar(value=1.0)
        ttk.Entry(size_frame, textvariable=self.obs_size, width=6).grid(row=0, column=1, padx=(0, 5))
        
        ttk.Label(size_frame, text="Height:").grid(row=0, column=2, padx=(0, 2))
        self.obs_height = tk.DoubleVar(value=2.0)
        ttk.Entry(size_frame, textvariable=self.obs_height, width=6).grid(row=0, column=3)
        
        # Buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=4, column=0, columnspan=2, pady=10)
        
        ttk.Button(button_frame, text="Add Obstacle", command=self.add_obstacle).grid(row=0, column=0, padx=(0, 5))
        ttk.Button(button_frame, text="Remove Selected", command=self.remove_obstacle).grid(row=0, column=1, padx=5)
        ttk.Button(button_frame, text="Clear All", command=self.clear_obstacles).grid(row=0, column=2, padx=5)
        
        # Preview button
        ttk.Button(button_frame, text="Preview 3D", command=self.preview_3d).grid(row=1, column=0, columnspan=3, pady=(10, 0))
        
        # Action buttons
        action_frame = ttk.Frame(main_frame)
        action_frame.grid(row=6, column=0, columnspan=2, pady=20)
        
        ttk.Button(action_frame, text="Start Simulation", command=self.start_simulation, 
                  style="Accent.TButton").grid(row=0, column=0, padx=10)
        ttk.Button(action_frame, text="Save Configuration", command=self.save_config).grid(row=0, column=1, padx=10)
        ttk.Button(action_frame, text="Load Configuration", command=self.load_config).grid(row=0, column=2, padx=10)
        ttk.Button(action_frame, text="Exit", command=self.root.quit).grid(row=0, column=3, padx=10)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        obstacle_frame.columnconfigure(0, weight=1)
        
    def add_obstacle(self):
        """Add a new obstacle"""
        shape = self.obs_shape.get()
        pos = [self.obs_x.get(), self.obs_y.get(), self.obs_z.get()]
        size = self.obs_size.get()
        height = self.obs_height.get()
        
        if shape == "sphere":
            obstacle_size = (size,)
        else:
            obstacle_size = (size, size, height)
        
        obstacle = {
            'shape': shape,
            'position': pos,
            'size': obstacle_size,
            'color': 'red'
        }
        
        self.obstacles.append(obstacle)
        self.update_obstacle_list()
        
    def remove_obstacle(self):
        """Remove selected obstacle"""
        selection = self.obstacle_listbox.curselection()
        if selection:
            index = selection[0]
            del self.obstacles[index]
            self.update_obstacle_list()
            
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles = []
        self.update_obstacle_list()
        
    def update_obstacle_list(self):
        """Update the obstacle listbox"""
        self.obstacle_listbox.delete(0, tk.END)
        for i, obs in enumerate(self.obstacles):
            shape = obs['shape']
            pos = obs['position']
            size = obs['size']
            text = f"{i+1}. {shape} at ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) size={size}"
            self.obstacle_listbox.insert(tk.END, text)
            
    def preview_3d(self):
        """Preview the 3D environment"""
        try:
            start_pos = np.array([self.start_x.get(), self.start_y.get(), self.start_z.get()])
            end_pos = np.array([self.end_x.get(), self.end_y.get(), self.end_z.get()])
            
            # Create a simple 3D preview
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # Draw start and end
            ax.scatter([start_pos[0]], [start_pos[1]], [start_pos[2]], 
                      color='green', s=100, marker='o', label='Start')
            ax.scatter([end_pos[0]], [end_pos[1]], [end_pos[2]], 
                      color='red', s=100, marker='*', label='Goal')
            
            # Draw obstacles
            for obs in self.obstacles:
                pos = obs['position']
                shape = obs['shape']
                size = obs['size']
                
                if shape == "cylinder":
                    radius, length, height = size
                    u = np.linspace(0, 2 * np.pi, 20)
                    v = np.linspace(0, height, 10)
                    u, v = np.meshgrid(u, v)
                    
                    x_obs = pos[0] + radius * np.cos(u)
                    y_obs = pos[1] + radius * np.sin(u)
                    z_obs = pos[2] + v
                    
                    ax.plot_surface(x_obs, y_obs, z_obs, alpha=0.7, color='red')
                    
                elif shape == "cube":
                    width, length, height = size
                    # Simple cube representation
                    ax.bar3d(pos[0] - width/2, pos[1] - length/2, pos[2], 
                            width, length, height, alpha=0.7, color='red')
                            
                elif shape == "sphere":
                    radius = size[0]
                    u = np.linspace(0, 2 * np.pi, 20)
                    v = np.linspace(0, np.pi, 20)
                    u, v = np.meshgrid(u, v)
                    
                    x_obs = pos[0] + radius * np.sin(v) * np.cos(u)
                    y_obs = pos[1] + radius * np.sin(v) * np.sin(u)
                    z_obs = pos[2] + radius * np.cos(v)
                    
                    ax.plot_surface(x_obs, y_obs, z_obs, alpha=0.7, color='red')
            
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('3D Environment Preview')
            ax.legend()
            
            plt.show()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to create preview: {str(e)}")
            
    def start_simulation(self):
        """Start the 3D simulation"""
        try:
            start_pos = np.array([self.start_x.get(), self.start_y.get(), self.start_z.get()])
            end_pos = np.array([self.end_x.get(), self.end_y.get(), self.end_z.get()])
            
            # Create and run the stable 3D drone
            drone = Stable3DDrone(start_pos, end_pos)
            
            # Clear default obstacles and add our custom ones
            drone.obstacles.clear()
            
            # Add obstacles using the Obstacle3D class
            for obs in self.obstacles:
                from stable_3d_drone import Obstacle3D
                obstacle = Obstacle3D(
                    position=obs['position'],
                    shape=obs['shape'],
                    size=obs['size'],
                    color=obs['color']
                )
                drone.obstacles.append(obstacle)
            
            # Close the setup window
            self.root.destroy()
            
            # Run simulation
            drone.run_simulation()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start simulation: {str(e)}")
            
    def save_config(self):
        """Save configuration to file"""
        try:
            config = {
                'start_pos': [self.start_x.get(), self.start_y.get(), self.start_z.get()],
                'end_pos': [self.end_x.get(), self.end_y.get(), self.end_z.get()],
                'drone_shape': self.drone_shape_var.get(),
                'obstacles': self.obstacles
            }
            
            import json
            with open('drone_config.json', 'w') as f:
                json.dump(config, f, indent=2)
                
            messagebox.showinfo("Success", "Configuration saved to drone_config.json")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save configuration: {str(e)}")
            
    def load_config(self):
        """Load configuration from file"""
        try:
            import json
            with open('drone_config.json', 'r') as f:
                config = json.load(f)
                
            # Load start position
            self.start_x.set(config['start_pos'][0])
            self.start_y.set(config['start_pos'][1])
            self.start_z.set(config['start_pos'][2])
            
            # Load end position
            self.end_x.set(config['end_pos'][0])
            self.end_y.set(config['end_pos'][1])
            self.end_z.set(config['end_pos'][2])
            
            # Load drone shape
            self.drone_shape_var.set(config['drone_shape'])
            
            # Load obstacles
            self.obstacles = config['obstacles']
            self.update_obstacle_list()
            
            messagebox.showinfo("Success", "Configuration loaded from drone_config.json")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load configuration: {str(e)}")
            
    def run(self):
        """Run the setup UI"""
        self.root.mainloop()

def main():
    print("=== 3D DRONE ENVIRONMENT SETUP ===")
    print("This tool helps you configure the 3D drone environment")
    print("before starting the simulation.")
    print()
    
    app = Drone3DSetupUI()
    app.run()

if __name__ == "__main__":
    main()
