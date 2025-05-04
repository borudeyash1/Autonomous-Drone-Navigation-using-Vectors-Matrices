<<<<<<< HEAD
# Autonomous-Drone-Navigation-using-Vectors-Matrices
.
=======
# Autonomous Multi-Drone Navigation System

An advanced, interactive, and beginner-friendly drone navigation system using Python, PyBullet, and computer vision for multi-drone simulation, dynamic obstacle avoidance, and real-time path planning.

---

## Features

- **Multi-Drone Support:** Simulate and control multiple drones simultaneously, each with a unique color, independent path, and controller. Easily scale up the number of drones for swarm or fleet scenarios.
- **Dynamic Obstacles & No-Fly Zones:** Add obstacles and no-fly zones that move or animate in real time. Drones automatically replan their paths to avoid collisions and restricted areas.
- **Realistic Physics:** Accurate quadrotor dynamics and propeller visuals using PyBullet.
- **Interactive 2D Map:** Drag-and-drop UI for setting start/end points, static and dynamic obstacles, and no-fly zones. Edit the path in real time.
- **Autonomous & Manual Modes:** Switch between full autonomy (A* path following) and manual PID-based control for all drones. Manual mode uses intuitive sliders for thrust, roll, pitch, and yaw.
- **Robust Pathfinding:** A* algorithm ensures drones never cut corners or pass through obstacles/walls. Real-time replanning if the environment changes.
- **Environment Validation:** Clear errors if start/end is inside an obstacle or unreachable.
- **Smooth 3D Animation:** Animated drone movement, propeller visuals, and modern 3D UI.
- **3D Camera Controls:** Rotate, pan, zoom, and follow drones in GUI mode.
- **Mission Logging:** Every move is logged with simulated latitude/longitude and action for each drone.
- **Extensible Architecture:** Modular codebase for easy addition of new features (e.g., mission scripting, swarm behaviors, advanced planners).
- **Beginner-Friendly:** In-map tutorial overlay, clear error messages, and step-by-step guide.

---

## Example Images

> **Note:** Replace the image links below with your own screenshots or GIFs for best results.

| 2D Map Setup | 3D Simulation (Multi-Drone) | Dynamic Obstacles & No-Fly Zones |
|:------------:|:--------------------------:|:-------------------------------:|
| ![2D Map Example](docs/example_map.png) | ![3D Multi-Drone](docs/example_3d.png) | ![Dynamic Obstacles](docs/example_dynamic.png) |

---

## Quick Start Guide

### 1. **Run the Simulation**
- Open a terminal in your project root.
- Run:
  ```bash
  python src/main.py
  ```
- For 3D camera controls, run:
  ```bash
  python src/main.py --gui
  ```

### 2. **Set Up the Map (Interactive 2D UI)**
- **Left-click:** Set the start point (green)
- **Right-click:** Add dynamic obstacle (blue)
- **Middle-click:** Add static obstacle (black)
- **Drag corners:** Resize obstacles (handles appear as red dots)
- **Left-click and drag:** Move obstacles
- **Shift+Left-click:** Mark a no-fly zone (red)
- **Drag yellow path:** Edit the path in real time
- **Right-click on a cell:** Set the end point (red)
- **Press Enter:** Confirm setup and start simulation

### 3. **Multi-Drone Usage**
- The number of drones is set in `main.py` (see the `num_drones` parameter in the `Simulator` initialization).
- Each drone is assigned a unique color and independent path.
- Paths are planned and set for each drone at startup.
- All drones are visualized in both 2D and 3D views.

### 4. **Simulation Controls**
- **Switch Mode:** Press `m` to toggle between Autonomous and Manual control for all drones.
- **Manual Control:** Use on-screen sliders to adjust thrust, roll, pitch, and yaw for all drones.
- **Quit:** Press `q` to exit simulation.

### 5. **3D Camera Controls (GUI Mode)**
- **Rotate:** Left mouse drag
- **Pan:** Right mouse drag
- **Zoom:** Mouse wheel
- **Follow drone:** Press `F`

---

## Multi-Drone & Advanced Features

- **Multiple Drones:**
  - Easily configure the number of drones in `main.py` or via the `Simulator` class.
  - Each drone is managed by the `DroneFleet` class, with its own controller and path.
  - Drones can be added or removed at runtime (see `DroneFleet.add_drone` and `remove_drone`).
- **Dynamic Obstacles & No-Fly Zones:**
  - Obstacles and zones can move with user-defined velocity vectors.
  - Drones detect and avoid these in real time, triggering path replanning if blocked.
  - No-fly zones are visualized in red; dynamic obstacles in blue.
- **Mission Logging:**
  - All drone movements are logged with simulated GPS coordinates and actions.
  - Logs can be extended for export or replay.
- **Extensible & Modular:**
  - Add new drone behaviors, mission scripts, or swarm logic by extending `DroneFleet`, `Controller`, or `PathPlanner`.
  - Easily integrate new sensors, planners, or UI features.

---

## Project Architecture

```
autonomous-drone-navigation/
├── src/
│   ├── control/
│   │   └── controller.py      # PID control for each drone
│   ├── perception/
│   │   └── obstacle_detector.py  # (Optional) Obstacle detection
│   ├── planning/
│   │   └── path_planner.py    # A* path planning, grid/world conversion, obstacle management
│   ├── simulation/
│   │   ├── simulator.py       # PyBullet simulation, 3D UI, status display
│   │   ├── quadrotor.py       # Drone model, propeller visuals, physics
│   │   └── drone_fleet.py     # Multi-drone management, per-drone paths/controllers
│   ├── ui/
│   │   └── map_ui.py          # Interactive 2D map UI, obstacle editing, path visualization
│   └── main.py                # Main entry point, scenario setup, mission logic
├── models/                    # Model files
├── data/                      # Data storage
├── tests/                     # Test files
├── docs/                      # Documentation
├── requirements.txt           # Dependencies
└── setup.py                   # Setup script
```

### Module Descriptions
- **control/controller.py:** Implements a PID controller for each drone, computing thrust and torques to reach target positions.
- **perception/obstacle_detector.py:** (Optional) For advanced perception or vision-based obstacle detection.
- **planning/path_planner.py:** Handles A* pathfinding, grid/world coordinate conversion, and manages static/dynamic obstacles and no-fly zones.
- **simulation/simulator.py:** Orchestrates the PyBullet simulation, 3D rendering, camera controls, and status display for all drones.
- **simulation/quadrotor.py:** Defines the drone's physical model, propeller visuals, and applies control inputs.
- **simulation/drone_fleet.py:** Manages all drones in the simulation, including their states, paths, and controllers. Supports adding/removing drones at runtime.
- **ui/map_ui.py:** Provides the interactive 2D map for user setup, obstacle editing, and real-time path visualization.
- **main.py:** Entry point for the simulation. Sets up the scenario, initializes drones, plans paths, and manages the main loop.

---

## API Usage Examples

Here are some code snippets to help you extend or automate the system:

### Add a Drone at Runtime
```python
from simulation.simulator import Simulator
sim = Simulator(num_drones=2)
# Add a new drone at a specific position
new_idx = sim.fleet.add_drone(position=np.array([2, 2, 1]))
```

### Set a Path for a Drone
```python
# Set a new path for drone 0
sim.fleet.set_drone_path(0, [np.array([0,0,1]), np.array([5,5,1])])
```

### Remove a Drone
```python
sim.fleet.remove_drone(1)  # Remove the second drone
```

### Access Drone States
```python
states = sim.fleet.get_drone_states()
for i, state in enumerate(states):
    print(f"Drone {i} position: {state['position']}")
```

### Custom Path Planning
```python
from planning.path_planner import PathPlanner
planner = PathPlanner()
start = np.array([-5, -5, 1])
end = np.array([5, 5, 1])
path = planner.plan_path(start, end)
```

---

## Advanced Configuration

- **Number of Drones:**
  - Set `num_drones` in `main.py` or pass as an argument to `Simulator`.
- **Drone Colors:**
  - Edit the `drone_colors` list in `simulation/drone_fleet.py` for custom palettes.
- **PID Controller Gains:**
  - Adjust `kp`, `ki`, `kd` in `control/controller.py` or when instantiating `Controller`.
- **Map/Grid Size:**
  - Change `grid_size` and `world_size` in `planning/path_planner.py` for larger or finer maps.
- **Obstacle/Zone Behavior:**
  - Modify velocity vectors in the dynamic obstacles/no-fly zones dictionaries in `main.py`.
- **Simulation Mode:**
  - Use `--gui` for 3D camera controls, or default for headless mode.
- **Logging:**
  - Extend the logging logic in `main.py` to export to CSV/JSON or implement mission replay.

---

## Customization & Extensibility
- **Add More Drones:** Change `num_drones` in `main.py` or via `Simulator`.
- **Custom Drone Colors:** Edit the color palette in `drone_fleet.py` for unique drone appearances.
- **Advanced Path Planning:** Extend `PathPlanner` for multi-objective, time-dependent, or swarm planning.
- **Mission Scripting:** Add mission logic in `main.py` or create new modules for complex tasks.
- **UI/UX:** Tweak the 2D map or 3D UI in `map_ui.py` and `simulator.py` for custom controls or visuals.
- **Logging/Export:** Extend logging in `main.py` for CSV/JSON export, or implement replay features.
- **Sensors/Perception:** Integrate new sensors or vision modules in `perception/` for advanced autonomy.

---

## FAQ

**Q: Can I use this as a beginner?**
- Absolutely! The UI and code are designed for accessibility. Follow the in-map tutorial and this guide.

**Q: How do I add more drones?**
- Change the `num_drones` parameter in `main.py` or `Simulator`. Each drone will get its own color and path.

**Q: Can I add or remove drones during simulation?**
- Yes! Use `DroneFleet.add_drone()` or `remove_drone()` in your code to manage the fleet dynamically.

**Q: How do dynamic obstacles and no-fly zones work?**
- Add them in the 2D map. They move according to their velocity, and drones will replan paths in real time to avoid them.

**Q: Can I assign different missions or paths to each drone?**
- Yes! Each drone can have its own start/end and path. Extend `main.py` or `DroneFleet` for more complex missions.

**Q: How do I extend the system for research or teaching?**
- The code is modular and well-commented. Add new modules, planners, or UI features as needed.

**Q: Can I save and load missions or logs?**
- Export/import features are planned. For now, all actions are logged and can be extended for export or replay.

---

## License
MIT License

## Contact
For questions or support, please contact:
- Email: [your-email@example.com]
- GitHub: [your-github-profile] 
>>>>>>> 965084a (asdfsdf)
