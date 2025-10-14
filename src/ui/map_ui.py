import numpy as np
import cv2

class MapUI:
    def __init__(self, grid_size=20, cell_size=30):
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.map = np.ones((grid_size, grid_size), dtype=np.uint8) * 255  # 255=free, 0=obstacle
        self.start = None
        self.end = None
        self.obstacles = set()
        self.obstacle_sizes = {}  # (gx,gy): (w,h)
        self.path = []
        self.drone_pos = None
        self.window_name = 'Interactive Map'
        self._dragging_obstacle = False
        self._dragged_obstacle = None
        self._resizing_obstacle = False
        self._resize_origin = None
        self._resize_corner = None
        self._dragging_path = False
        self._dragged_path_idx = None
        self.dynamic_obstacles = set()  # (gx,gy) for moving obstacles
        self.dynamic_obstacle_states = {}  # (gx,gy): {'dir': (dx,dy), 'speed': float}
        self.nofly_zones = set()        # (gx,gy) for no-fly zones
        self.nofly_zone_states = {}     # (gx,gy): {'timer': int, 'active': bool, 'period': int}
        self.drone_paths = {}  # drone_id: list of (gx, gy)
        self.locked = False
        self.path_points = []  # Store path points
        self.drone_positions = {}  # Store drone positions
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_event)

    def _find_resize_corner(self, gx, gy, x, y):
        # Returns corner name if (x,y) is near a corner of obstacle at (gx,gy)
        size = self.obstacle_sizes.get((gx,gy), (1,1))
        px, py = gx*self.cell_size, gy*self.cell_size
        corners = {
            'tl': (px, py),
            'tr': (px+size[0]*self.cell_size, py),
            'bl': (px, py+size[1]*self.cell_size),
            'br': (px+size[0]*self.cell_size, py+size[1]*self.cell_size)
        }
        for name, (cx,cy) in corners.items():
            if abs(x-cx)<8 and abs(y-cy)<8:
                return name
        return None

    def _mouse_event(self, event, x, y, flags, param):
        # Calculate grid position accurately
        gx = max(0, min(self.grid_size - 1, x // self.cell_size))
        gy = max(0, min(self.grid_size - 1, y // self.cell_size))
        
        # Always update mouse position for keyboard controls
        self.last_mouse_pos = (gx, gy)
        
        if self.locked:
            return
        
        # Debug: Print event info
        print(f"[DEBUG] Mouse event: {event}, flags: {flags}, pos: ({gx}, {gy})")
        
        # Check for modifier keys first
        ctrl_pressed = bool(flags & cv2.EVENT_FLAG_CTRLKEY)
        shift_pressed = bool(flags & cv2.EVENT_FLAG_SHIFTKEY)
        
        # Handle different mouse events
        if event == cv2.EVENT_LBUTTONDOWN:
            if ctrl_pressed:
                # Ctrl+Left-click: Toggle Dynamic Obstacle (Blue)
                if (gx, gy) in self.dynamic_obstacles:
                    self.dynamic_obstacles.remove((gx, gy))
                    print(f"[MAP] Dynamic obstacle removed at ({gx}, {gy})")
                else:
                    self.dynamic_obstacles.add((gx, gy))
                    print(f"[MAP] Dynamic obstacle added at ({gx}, {gy})")
            elif shift_pressed:
                # Shift+Left-click: Toggle No-Fly Zone (Red)
                if (gx, gy) in self.nofly_zones:
                    self.nofly_zones.remove((gx, gy))
                    print(f"[MAP] No-fly zone removed at ({gx}, {gy})")
                else:
                    self.nofly_zones.add((gx, gy))
                    print(f"[MAP] No-fly zone added at ({gx}, {gy})")
            else:
                # Left-click: Set Start Point (Green)
                self.start = (gx, gy)
                print(f"[MAP] Start point set at ({gx}, {gy})")
                
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right-click: Set End Point (Red)
            self.end = (gx, gy)
            print(f"[MAP] End point set at ({gx}, {gy})")
            
        elif event == cv2.EVENT_MBUTTONDOWN:
            # Middle-click: Toggle Static Obstacle (Black)
            if (gx, gy) in self.obstacles:
                self.obstacles.remove((gx, gy))
                self.map[gy, gx] = 255
                print(f"[MAP] Static obstacle removed at ({gx}, {gy})")
            else:
                self.obstacles.add((gx, gy))
                self.map[gy, gx] = 0
                self.obstacle_sizes[(gx, gy)] = (1, 1)
                print(f"[MAP] Static obstacle added at ({gx}, {gy})")
        
        # Also track mouse movement for keyboard controls
        elif event == cv2.EVENT_MOUSEMOVE:
            self.last_mouse_pos = (gx, gy)

    def draw(self):
        img = np.ones((self.grid_size * self.cell_size, self.grid_size * self.cell_size, 3), dtype=np.uint8) * 255
        
        # Draw grid first
        for i in range(self.grid_size + 1):
            cv2.line(img, (0, i * self.cell_size), (self.grid_size * self.cell_size, i * self.cell_size), (200, 200, 200), 1)
            cv2.line(img, (i * self.cell_size, 0), (i * self.cell_size, self.grid_size * self.cell_size), (200, 200, 200), 1)
        
        # Draw no-fly zones (red)
        for (gx, gy) in self.nofly_zones:
            cv2.rectangle(img, (gx * self.cell_size, gy * self.cell_size), 
                         ((gx + 1) * self.cell_size, (gy + 1) * self.cell_size), (0, 0, 255), -1)
        
        # Draw dynamic obstacles (blue)
        for (gx, gy) in self.dynamic_obstacles:
            cv2.rectangle(img, (gx * self.cell_size, gy * self.cell_size), 
                         ((gx + 1) * self.cell_size, (gy + 1) * self.cell_size), (255, 0, 0), -1)
        
        # Draw static obstacles (black)
        for (gx, gy) in self.obstacles:
            size = self.obstacle_sizes.get((gx, gy), (1, 1))
            cv2.rectangle(img, (gx * self.cell_size, gy * self.cell_size), 
                         ((gx + size[0]) * self.cell_size, (gy + size[1]) * self.cell_size), (0, 0, 0), -1)
        
        # Draw path (yellow)
        for (gx, gy) in self.path:
            cv2.rectangle(img, (gx * self.cell_size, gy * self.cell_size), 
                         ((gx + 1) * self.cell_size, (gy + 1) * self.cell_size), (0, 255, 255), -1)
        
        # Draw start point (green)
        if self.start:
            cv2.rectangle(img, (self.start[0] * self.cell_size, self.start[1] * self.cell_size), 
                         ((self.start[0] + 1) * self.cell_size, (self.start[1] + 1) * self.cell_size), (0, 255, 0), -1)
            # Add "START" label
            cv2.putText(img, "START", (self.start[0] * self.cell_size + 5, self.start[1] * self.cell_size + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Draw end point (red)
        if self.end:
            cv2.rectangle(img, (self.end[0] * self.cell_size, self.end[1] * self.cell_size), 
                         ((self.end[0] + 1) * self.cell_size, (self.end[1] + 1) * self.cell_size), (0, 0, 255), -1)
            # Add "END" label
            cv2.putText(img, "END", (self.end[0] * self.cell_size + 5, self.end[1] * self.cell_size + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Draw drone position
        if self.drone_pos:
            cv2.circle(img, (self.drone_pos[0] * self.cell_size + self.cell_size // 2, 
                            self.drone_pos[1] * self.cell_size + self.cell_size // 2), 
                      self.cell_size // 3, (255, 0, 0), -1)
        
        # Draw drone paths (history)
        for drone_id, path in self.drone_paths.items():
            if len(path) > 1:
                for i in range(1, len(path)):
                    x1, y1 = path[i - 1]
                    x2, y2 = path[i]
                    color = (0, 0, 255 - 50 * drone_id)  # Different color for each drone
                    cv2.line(img,
                            (x1 * self.cell_size + self.cell_size // 2, y1 * self.cell_size + self.cell_size // 2),
                            (x2 * self.cell_size + self.cell_size // 2, y2 * self.cell_size + self.cell_size // 2),
                            color, 2)
        
        # Draw simplified controls overlay
        overlay_lines = [
            'CONTROLS:',
            'MOUSE:',
            'Left-click: Set START (green)',
            'Right-click: Set END (red)',
            'Middle-click: Toggle OBSTACLE (black)',
            '',
            'KEYBOARD:',
            'S: Set START | E: Set END',
            'O: Toggle OBSTACLE (black)',
            'D: Toggle DYNAMIC OBSTACLE (blue)',
            'N: Toggle NO-FLY ZONE (red)',
            '',
            'Press ENTER to start simulation'
        ]
        
        # Draw semi-transparent background for controls
        overlay_bg = img.copy()
        cv2.rectangle(overlay_bg, (5, 5), (350, 200), (240, 240, 240), -1)
        img = cv2.addWeighted(img, 0.7, overlay_bg, 0.3, 0)
        
        # Draw control text
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        y0 = 25
        
        for i, line in enumerate(overlay_lines):
            color = (0, 0, 0) if i == 0 else (50, 50, 50)  # Title in black, others in dark gray
            cv2.putText(img, line, (10, y0 + i * 18), font, font_scale, color, thickness, cv2.LINE_AA)
        
        cv2.imshow(self.window_name, img)

    def get_map(self):
        return self.map.copy(), self.start, self.end, list(self.obstacles), dict(self.obstacle_sizes), list(self.dynamic_obstacles), list(self.nofly_zones)

    def set_path(self, path):
        self.path = path

    def set_drone_pos(self, pos):
        self.drone_pos = pos

    def wait_for_setup(self):
        print("\n" + "="*60)
        print("INTERACTIVE MAP SETUP")
        print("="*60)
        print("MOUSE CONTROLS:")
        print("  Left-click: Set START point (green)")
        print("  Right-click: Set END point (red)")
        print("  Middle-click: Toggle STATIC OBSTACLE (black)")
        print("  Ctrl+Left-click: Toggle DYNAMIC OBSTACLE (blue)")
        print("  Shift+Left-click: Toggle NO-FLY ZONE (red)")
        print("")
        print("KEYBOARD CONTROLS (if mouse doesn't work):")
        print("  Press 'S': Set START point at mouse position")
        print("  Press 'E': Set END point at mouse position")
        print("  Press 'O': Toggle STATIC OBSTACLE at mouse position")
        print("  Press 'D': Toggle DYNAMIC OBSTACLE at mouse position")
        print("  Press 'N': Toggle NO-FLY ZONE at mouse position")
        print("  Press ENTER: Start simulation")
        print("="*60)
        print("Please set START and END points, then press ENTER to begin...")
        
        # Store last mouse position for keyboard controls
        self.last_mouse_pos = (0, 0)
        
        while True:
            self.draw()
            key = cv2.waitKey(50) & 0xFF
            
            if key == 13:  # Enter
                if self.start and self.end:
                    print(f"\n[SETUP COMPLETE]")
                    print(f"  Start: {self.start}")
                    print(f"  End: {self.end}")
                    print(f"  Static Obstacles: {len(self.obstacles)}")
                    print(f"  Dynamic Obstacles: {len(self.dynamic_obstacles)}")
                    print(f"  No-Fly Zones: {len(self.nofly_zones)}")
                    print("="*60)
                    break
                else:
                    print("[WARNING] Please set both START and END points before pressing ENTER!")
            
            # Keyboard controls
            elif key == ord('s') or key == ord('S'):
                gx, gy = self.last_mouse_pos
                self.start = (gx, gy)
                print(f"[MAP] Start point set at ({gx}, {gy})")
            
            elif key == ord('e') or key == ord('E'):
                gx, gy = self.last_mouse_pos
                self.end = (gx, gy)
                print(f"[MAP] End point set at ({gx}, {gy})")
            
            elif key == ord('o') or key == ord('O'):
                gx, gy = self.last_mouse_pos
                if (gx, gy) in self.obstacles:
                    self.obstacles.remove((gx, gy))
                    self.map[gy, gx] = 255
                    print(f"[MAP] Static obstacle removed at ({gx}, {gy})")
                else:
                    self.obstacles.add((gx, gy))
                    self.map[gy, gx] = 0
                    self.obstacle_sizes[(gx, gy)] = (1, 1)
                    print(f"[MAP] Static obstacle added at ({gx}, {gy})")
            
            elif key == ord('d') or key == ord('D'):
                gx, gy = self.last_mouse_pos
                if (gx, gy) in self.dynamic_obstacles:
                    self.dynamic_obstacles.remove((gx, gy))
                    print(f"[MAP] Dynamic obstacle removed at ({gx}, {gy})")
                else:
                    self.dynamic_obstacles.add((gx, gy))
                    print(f"[MAP] Dynamic obstacle added at ({gx}, {gy})")
            
            elif key == ord('n') or key == ord('N'):
                gx, gy = self.last_mouse_pos
                if (gx, gy) in self.nofly_zones:
                    self.nofly_zones.remove((gx, gy))
                    print(f"[MAP] No-fly zone removed at ({gx}, {gy})")
                else:
                    self.nofly_zones.add((gx, gy))
                    print(f"[MAP] No-fly zone added at ({gx}, {gy})")
        
        result = self.get_map()
        self.locked = True  # Lock editing after setup
        return result

    def close(self):
        cv2.destroyWindow(self.window_name)

    def step_dynamic_elements(self):
        """
        Animate dynamic obstacles and time-dependent no-fly zones.
        Dynamic obstacles move in a direction and bounce off walls.
        No-fly zones toggle on/off with a timer (periodic).
        """
        # --- Animate dynamic obstacles ---
        new_dynamic_obstacles = set()
        new_dynamic_states = {}
        for (gx, gy) in self.dynamic_obstacles:
            state = self.dynamic_obstacle_states.get((gx, gy), {'dir': (1, 0), 'speed': 1})
            dx, dy = state['dir']
            speed = state['speed']
            ngx, ngy = gx + dx * speed, gy + dy * speed
            # Bounce off walls
            if not (0 <= ngx < self.grid_size):
                dx = -dx
                ngx = gx + dx * speed
            if not (0 <= ngy < self.grid_size):
                dy = -dy
                ngy = gy + dy * speed
            ngx, ngy = int(ngx), int(ngy)
            # Avoid static obstacles
            if (ngx, ngy) in self.obstacles:
                dx, dy = -dx, -dy
                ngx, ngy = gx + dx * speed, gy + dy * speed
                ngx, ngy = int(ngx), int(ngy)
            new_dynamic_obstacles.add((ngx, ngy))
            new_dynamic_states[(ngx, ngy)] = {'dir': (dx, dy), 'speed': speed}
        self.dynamic_obstacles = new_dynamic_obstacles
        self.dynamic_obstacle_states = new_dynamic_states

        # --- Animate no-fly zones (toggle active/inactive) ---
        new_nofly_zones = set()
        new_nofly_states = {}
        for (gx, gy) in self.nofly_zones:
            state = self.nofly_zone_states.get((gx, gy), {'timer': 0, 'active': True, 'period': 60})
            timer = state['timer'] + 1
            period = state['period']
            active = state['active']
            if timer >= period:
                active = not active
                timer = 0
            if active:
                new_nofly_zones.add((gx, gy))
            new_nofly_states[(gx, gy)] = {'timer': timer, 'active': active, 'period': period}
        self.nofly_zones = new_nofly_zones
        self.nofly_zone_states = new_nofly_states

    def update_drone_path(self, drone_id, gx, gy):
        if drone_id not in self.drone_paths:
            self.drone_paths[drone_id] = []
        self.drone_paths[drone_id].append((gx, gy))
        self.drone_positions[drone_id] = (gx, gy)

    def clear_path(self):
        """Clear all path points."""
        self.path_points = []

    def add_path_point(self, x, y):
        """Add a point to the path."""
        self.path_points.append((x, y))

    def draw_path(self):
        """Draw the current path."""
        img = np.ones((self.grid_size * self.cell_size, self.grid_size * self.cell_size, 3), dtype=np.uint8) * 255
        for (gx, gy) in self.path_points:
            cv2.rectangle(img, (gx*self.cell_size, gy*self.cell_size), ((gx+1)*self.cell_size, (gy+1)*self.cell_size), (0,255,255), -1)
        cv2.imshow(self.window_name, img)

    def draw_drone_paths(self):
        """Draw all drone paths."""
        img = np.ones((self.grid_size * self.cell_size, self.grid_size * self.cell_size, 3), dtype=np.uint8) * 255
        for drone_id, path in self.drone_paths.items():
            if len(path) > 1:
                for i in range(1, len(path)):
                    x1, y1 = path[i-1]
                    x2, y2 = path[i]
                    color = (0, 0, 255 - 50*drone_id)  # Different color for each drone
                    cv2.line(img,
                        (x1*self.cell_size+self.cell_size//2, y1*self.cell_size+self.cell_size//2),
                        (x2*self.cell_size+self.cell_size//2, y2*self.cell_size+self.cell_size//2),
                        color, 2)
        cv2.imshow(self.window_name, img)

    def draw_drone_positions(self):
        """Draw all drone positions."""
        img = np.ones((self.grid_size * self.cell_size, self.grid_size * self.cell_size, 3), dtype=np.uint8) * 255
        for drone_id, (gx, gy) in self.drone_positions.items():
            cv2.circle(img, (gx*self.cell_size+self.cell_size//2, gy*self.cell_size+self.cell_size//2), self.cell_size//4, (255,0,255), -1)
        cv2.imshow(self.window_name, img) 