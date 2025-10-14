import numpy as np
from typing import List, Tuple, Dict
from queue import PriorityQueue

class PathPlanner:
    def __init__(self, grid_size: int = 20, world_size: float = 10.0):
        """
        Initialize the path planner.
        
        Args:
            grid_size: Size of the planning grid
            world_size: Size of the world in meters
        """
        self.grid_size = grid_size
        self.world_size = world_size
        self.obstacles = set()
        self.dynamic_obstacles = set()
        self.no_fly_zones = set()
    
    def world_to_grid(self, pos: np.ndarray) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates.
        
        Args:
            pos: Position in world coordinates [x, y, z]
            
        Returns:
            Grid coordinates (x, y)
        """
        x = int(round((pos[0] + self.world_size/2) / (self.world_size / (self.grid_size-1))))
        y = int(round((pos[1] + self.world_size/2) / (self.world_size / (self.grid_size-1))))
        return (x, y)
    
    def grid_to_world(self, pos: Tuple[int, int], z: float = 1.0) -> np.ndarray:
        """
        Convert grid coordinates to world coordinates.
        
        Args:
            pos: Grid coordinates (x, y)
            z: Z coordinate in world space
            
        Returns:
            Position in world coordinates [x, y, z]
        """
        x = (pos[0] / (self.grid_size-1)) * self.world_size - self.world_size/2
        y = (pos[1] / (self.grid_size-1)) * self.world_size - self.world_size/2
        return np.array([x, y, z])
    
    def add_obstacle(self, pos: np.ndarray, radius: float = 0.5):
        """Add a static obstacle."""
        grid_pos = self.world_to_grid(pos)
        self.obstacles.add(grid_pos)
    
    def add_dynamic_obstacle(self, pos: np.ndarray, radius: float = 0.5):
        """Add a dynamic obstacle."""
        grid_pos = self.world_to_grid(pos)
        self.dynamic_obstacles.add(grid_pos)
    
    def add_no_fly_zone(self, pos: np.ndarray, radius: float = 1.0):
        """Add a no-fly zone."""
        grid_pos = self.world_to_grid(pos)
        self.no_fly_zones.add(grid_pos)
    
    def is_valid_position(self, pos: Tuple[int, int]) -> bool:
        """
        Check if a position is valid (not in obstacle or no-fly zone).
        
        Args:
            pos: Grid coordinates (x, y)
            
        Returns:
            True if the position is valid
        """
        if not (0 <= pos[0] < self.grid_size and 0 <= pos[1] < self.grid_size):
            return False
        if pos in self.obstacles or pos in self.no_fly_zones:
            return False
        return True
    
    def plan_path(self, start: np.ndarray, end: np.ndarray) -> List[np.ndarray]:
        """
        Plan a path from start to end using A* algorithm with enhanced obstacle avoidance.
        
        Args:
            start: Start position in world coordinates
            end: End position in world coordinates
            
        Returns:
            List of waypoints in world coordinates
        """
        # Convert to grid coordinates
        start_grid = self.world_to_grid(start)
        end_grid = self.world_to_grid(end)
        
        # Validate start and end positions
        if not self.is_valid_position(start_grid):
            print(f"Invalid start position: {start_grid}")
            return []
        if not self.is_valid_position(end_grid):
            print(f"Invalid end position: {end_grid}")
            return []
        
        # Initialize A* search
        visited = set()
        prev = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, end_grid)}
        pq = PriorityQueue()
        pq.put((f_score[start_grid], start_grid))
        
        # Search directions (8-connected grid)
        directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
        
        while not pq.empty():
            _, current = pq.get()
            
            if current == end_grid:
                # Reconstruct path
                path = []
                while current != start_grid:
                    path.append(self.grid_to_world(current))
                    current = prev[current]
                path.append(self.grid_to_world(start_grid))
                path = path[::-1]
                return self.path_smooth(path)
            
            visited.add(current)
            
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not self.is_valid_position(neighbor) or neighbor in visited:
                    continue
                
                # Calculate cost with obstacle avoidance
                base_cost = 1.0
                if abs(dx) + abs(dy) == 2:  # Diagonal movement
                    base_cost = 1.414  # sqrt(2)
                
                # Add penalty for being near obstacles
                obstacle_penalty = self._calculate_obstacle_penalty(neighbor)
                cost = base_cost + obstacle_penalty
                
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    prev[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, end_grid)
                    pq.put((f_score[neighbor], neighbor))
        
        return []
    
    def _calculate_obstacle_penalty(self, pos: Tuple[int, int]) -> float:
        """Calculate penalty for being near obstacles."""
        penalty = 0.0
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                neighbor = (pos[0] + dx, pos[1] + dy)
                if neighbor in self.obstacles:
                    distance = np.sqrt(dx*dx + dy*dy)
                    if distance > 0:
                        penalty += 0.5 / distance  # Inverse distance penalty
        return penalty
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """
        Manhattan distance heuristic for A*.
        
        Args:
            a: First position (x, y)
            b: Second position (x, y)
            
        Returns:
            Manhattan distance between the positions
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def update_dynamic_obstacles(self, obstacles: List[Dict]):
        """
        Update the positions of dynamic obstacles.
        
        Args:
            obstacles: List of dynamic obstacle dictionaries
        """
        self.dynamic_obstacles.clear()
        for obstacle in obstacles:
            grid_pos = self.world_to_grid(obstacle['position'])
            self.dynamic_obstacles.add(grid_pos)
    
    def update_no_fly_zones(self, zones: List[Dict]):
        """
        Update the positions of no-fly zones.
        
        Args:
            zones: List of no-fly zone dictionaries
        """
        self.no_fly_zones.clear()
        for zone in zones:
            grid_pos = self.world_to_grid(zone['position'])
            self.no_fly_zones.add(grid_pos)
    
    def path_smooth(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """Remove unnecessary waypoints and smooth the path for better navigation."""
        if len(path) <= 2:
            return path
        
        # First pass: Remove collinear points
        smoothed = [path[0]]
        for i in range(1, len(path)-1):
            prev = smoothed[-1]
            curr = path[i]
            next = path[i+1]
            v1 = curr - prev
            v2 = next - curr
            v1 = v1 / (np.linalg.norm(v1) + 1e-8)
            v2 = v2 / (np.linalg.norm(v2) + 1e-8)
            # If direction changes significantly, keep the waypoint
            if not np.allclose(v1, v2, atol=0.1):
                smoothed.append(curr)
        smoothed.append(path[-1])
        
        # Second pass: Add intermediate waypoints for smoother navigation
        if len(smoothed) >= 2:
            final_path = [smoothed[0]]
            for i in range(len(smoothed) - 1):
                current = smoothed[i]
                next = smoothed[i + 1]
                distance = np.linalg.norm(next - current)
                
                # Add intermediate waypoints if distance is too large
                if distance > 2.0:
                    num_intermediate = int(distance / 1.5)
                    for j in range(1, num_intermediate + 1):
                        t = j / (num_intermediate + 1)
                        intermediate = current + t * (next - current)
                        final_path.append(intermediate)
                
                final_path.append(next)
            return final_path
        
        return smoothed
    
    def check_path_safety(self, path: List[np.ndarray]) -> bool:
        """Check if the path is safe from obstacles."""
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]
            
            # Check line segment for obstacles
            num_checks = 10
            for j in range(num_checks + 1):
                t = j / num_checks
                check_pos = start + t * (end - start)
                grid_pos = self.world_to_grid(check_pos)
                
                if not self.is_valid_position(grid_pos):
                    return False
        return True
    
    def replan_path(self, start: np.ndarray, end: np.ndarray, current_path: List[np.ndarray] = None) -> List[np.ndarray]:
        """Replan path with enhanced obstacle avoidance."""
        # Try direct path first
        direct_path = [start, end]
        if self.check_path_safety(direct_path):
            return self.path_smooth(direct_path)
        
        # Use A* if direct path is not safe
        planned_path = self.plan_path(start, end)
        if planned_path and self.check_path_safety(planned_path):
            return planned_path
        
        # If A* fails, try with relaxed constraints
        return self.plan_path_with_relaxed_constraints(start, end)
    
    def plan_path_with_relaxed_constraints(self, start: np.ndarray, end: np.ndarray) -> List[np.ndarray]:
        """Plan path with relaxed obstacle constraints for emergency navigation."""
        # Convert to grid coordinates
        start_grid = self.world_to_grid(start)
        end_grid = self.world_to_grid(end)
        
        # Use larger search radius
        search_radius = 3
        directions = []
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                if dx != 0 or dy != 0:
                    directions.append((dx, dy))
        
        # Simple path: go up, over, then down
        mid_height = max(start[2], end[2]) + 2.0
        path = [
            start,
            np.array([start[0], start[1], mid_height]),
            np.array([end[0], end[1], mid_height]),
            end
        ]
        
        return path 

    def world_to_grid_3d(self, pos: np.ndarray) -> Tuple[int, int, int]:
        x = int(round((pos[0] + self.world_size/2) / (self.world_size / (self.grid_size-1))))
        y = int(round((pos[1] + self.world_size/2) / (self.world_size / (self.grid_size-1))))
        z = int(round((pos[2]) / (self.world_size / (self.grid_size-1))))
        return (x, y, z)

    def grid_to_world_3d(self, pos: Tuple[int, int, int]) -> np.ndarray:
        x = (pos[0] / (self.grid_size-1)) * self.world_size - self.world_size/2
        y = (pos[1] / (self.grid_size-1)) * self.world_size - self.world_size/2
        z = (pos[2] / (self.grid_size-1)) * self.world_size
        return np.array([x, y, z])

    def is_valid_position_3d(self, pos: Tuple[int, int, int], obstacles_3d: list) -> bool:
        x, y, z = pos
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size and 0 <= z < self.grid_size):
            return False
        for ox, oy, oz, w, l, h in obstacles_3d:
            if (ox <= x < ox+w) and (oy <= y < oy+l) and (oz <= z < oz+h):
                return False
        return True

    def plan_path_3d(self, start: np.ndarray, end: np.ndarray, obstacles_3d: list) -> list:
        start_grid = self.world_to_grid_3d(start)
        end_grid = self.world_to_grid_3d(end)
        from queue import PriorityQueue
        visited = set()
        prev = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic_3d(start_grid, end_grid)}
        pq = PriorityQueue()
        pq.put((f_score[start_grid], start_grid))
        # 26-connected neighbors
        directions = [(dx,dy,dz) for dx in [-1,0,1] for dy in [-1,0,1] for dz in [-1,0,1] if not (dx==0 and dy==0 and dz==0)]
        while not pq.empty():
            _, current = pq.get()
            if current == end_grid:
                path = []
                while current != start_grid:
                    path.append(self.grid_to_world_3d(current))
                    current = prev[current]
                path.append(self.grid_to_world_3d(start_grid))
                return path[::-1]
            visited.add(current)
            for dx, dy, dz in directions:
                neighbor = (current[0]+dx, current[1]+dy, current[2]+dz)
                if not self.is_valid_position_3d(neighbor, obstacles_3d) or neighbor in visited:
                    continue
                # Human-like logic: prefer up if vertical cost < horizontal detour
                vertical_cost = abs(dz)
                horizontal_cost = abs(dx) + abs(dy)
                cost = 1.0
                if vertical_cost > 0 and vertical_cost < horizontal_cost:
                    cost = 0.7  # prefer up if it's shorter
                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    prev[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic_3d(neighbor, end_grid)
                    pq.put((f_score[neighbor], neighbor))
        return []

    def heuristic_3d(self, a: Tuple[int,int,int], b: Tuple[int,int,int]) -> float:
        return abs(a[0]-b[0]) + abs(a[1]-b[1]) + abs(a[2]-b[2]) 