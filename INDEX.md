# Project Index

## Project Overview
- Autonomous Multi-Drone Navigation System
- Python-based simulation using PyBullet
- Features multi-drone support, dynamic obstacles, and real-time path planning

## Directory Structure

### Root Directory
- `README.md` - Main project documentation
- `requirements.txt` - Python dependencies
- `setup.py` - Project setup and installation
- `test_pybullet_gui.py` - GUI testing script
- `.gitignore` - Git ignore rules

### Main Directories
- `src/` - Source code
- `models/` - Drone and environment model files
- `data/` - Data storage
- `docs/` - Documentation
- `tests/` - Test files
- `simulation/` - Simulation-related files
- `.venv/` - Virtual environment

## Source Code (`src/`)

### Core Files
- `main.py` - Main entry point and simulation controller

### Subdirectories
1. `simulation/`
   - Core simulation engine
   - Drone fleet management
   - Quadrotor physics and visualization

2. `ui/`
   - Interactive 2D map interface
   - User interface components

3. `control/`
   - PID controller implementation
   - Drone control systems

4. `planning/`
   - Path planning algorithms
   - A* implementation
   - Grid/world conversion

5. `perception/`
   - Obstacle detection
   - Computer vision utilities

6. `utils/`
   - Utility functions and helpers

7. `models/`
   - Model definitions and configurations

## Key Features
1. Multi-Drone Support
2. Dynamic Obstacles & No-Fly Zones
3. Realistic Physics Simulation
4. Interactive 2D Map
5. Autonomous & Manual Control Modes
6. Robust Pathfinding
7. Environment Validation
8. 3D Visualization
9. Mission Logging
10. Extensible Architecture

## Documentation
- `README.md` - Comprehensive project documentation
- `docs/` - Additional documentation files

## Testing
- `tests/` - Test suite
- `test_pybullet_gui.py` - GUI testing

## Dependencies
- numpy
- opencv-python
- pybullet
- (See `requirements.txt` for complete list)

## Configuration
- Environment setup in `setup.py`
- Dependencies in `requirements.txt`
- Model configurations in `models/`
- Simulation parameters in `src/simulation/`

## Data Management
- `data/` - Storage for logs and mission data
- Mission logging system
- Data export capabilities

## Development Tools
- Virtual environment (`.venv/`)
- Git version control
- Testing framework
- Documentation system 