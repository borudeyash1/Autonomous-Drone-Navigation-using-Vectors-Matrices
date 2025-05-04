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
        gx, gy = x // self.cell_size, y // self.cell_size
        if gx >= self.grid_size or gy >= self.grid_size:
            return
        if self.locked:
            return
        # End point: Ctrl+Right-click
        if event == cv2.EVENT_RBUTTONDOWN and (flags & cv2.EVENT_FLAG_CTRLKEY):
            self.end = (gx, gy)
            return
        # No-fly zone: shift+left-click
        if event == cv2.EVENT_LBUTTONDOWN and (flags & cv2.EVENT_FLAG_SHIFTKEY):
            self.nofly_zones.add((gx, gy))
            return
        # Dynamic obstacle: right-click (no Ctrl)
        if event == cv2.EVENT_RBUTTONDOWN:
            self.dynamic_obstacles.add((gx, gy))
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            # Check for resize
            for obs in self.obstacles:
                corner = self._find_resize_corner(obs[0], obs[1], x, y)
                if corner:
                    self._resizing_obstacle = True
                    self._resize_origin = obs
                    self._resize_corner = corner
                    return
            if (gx, gy) in self.obstacles:
                self._dragging_obstacle = True
                self._dragged_obstacle = (gx, gy)
            else:
                self.start = (gx, gy)
            # Path dragging
            for idx, (px, py) in enumerate(self.path):
                if abs(gx-px)<=0 and abs(gy-py)<=0:
                    self._dragging_path = True
                    self._dragged_path_idx = idx
                    return
        elif event == cv2.EVENT_MOUSEMOVE and self._resizing_obstacle:
            # Resize obstacle
            ox, oy = self._resize_origin
            size = list(self.obstacle_sizes.get((ox,oy), (1,1)))
            dx = (x - ox*self.cell_size) // self.cell_size
            dy = (y - oy*self.cell_size) // self.cell_size
            if self._resize_corner == 'br':
                size[0] = max(1, dx)
                size[1] = max(1, dy)
            elif self._resize_corner == 'tr':
                size[0] = max(1, dx)
            elif self._resize_corner == 'bl':
                size[1] = max(1, dy)
            self.obstacle_sizes[(ox,oy)] = tuple(size)
        elif event == cv2.EVENT_LBUTTONUP and self._resizing_obstacle:
            self._resizing_obstacle = False
            self._resize_origin = None
            self._resize_corner = None
        elif event == cv2.EVENT_MOUSEMOVE and self._dragging_obstacle:
            pass
        elif event == cv2.EVENT_LBUTTONUP and self._dragging_obstacle:
            if self._dragged_obstacle:
                self.obstacles.discard(self._dragged_obstacle)
                self.map[self._dragged_obstacle[1], self._dragged_obstacle[0]] = 255
                self.obstacles.add((gx, gy))
                self.map[gy, gx] = 0
                # Move size info
                if self._dragged_obstacle in self.obstacle_sizes:
                    self.obstacle_sizes[(gx,gy)] = self.obstacle_sizes.pop(self._dragged_obstacle)
            self._dragging_obstacle = False
            self._dragged_obstacle = None
        elif event == cv2.EVENT_MBUTTONDOWN:
            self.obstacles.add((gx, gy))
            self.map[gy, gx] = 0
            self.obstacle_sizes[(gx,gy)] = (1,1)
        elif event == cv2.EVENT_MOUSEMOVE and self._dragging_path:
            if self._dragged_path_idx is not None:
                self.path[self._dragged_path_idx] = (gx, gy)
        elif event == cv2.EVENT_LBUTTONUP and self._dragging_path:
            self._dragging_path = False
            self._dragged_path_idx = None

    def draw(self):
        img = np.ones((self.grid_size * self.cell_size, self.grid_size * self.cell_size, 3), dtype=np.uint8) * 255
        # Draw no-fly zones (red)
        for (gx, gy) in self.nofly_zones:
            cv2.rectangle(img, (gx*self.cell_size, gy*self.cell_size), ((gx+1)*self.cell_size, (gy+1)*self.cell_size), (0,0,255), -1)
        # Draw dynamic obstacles (blue)
        for (gx, gy) in self.dynamic_obstacles:
            cv2.rectangle(img, (gx*self.cell_size, gy*self.cell_size), ((gx+1)*self.cell_size, (gy+1)*self.cell_size), (255,0,0), -1)
        # Draw static obstacles (black)
        for (gx, gy) in self.obstacles:
            size = self.obstacle_sizes.get((gx,gy), (1,1))
            cv2.rectangle(img, (gx*self.cell_size, gy*self.cell_size), ((gx+size[0])*self.cell_size, (gy+size[1])*self.cell_size), (0,0,0), -1)
            for dx,dy in [(0,0),(size[0],0),(0,size[1]),(size[0],size[1])]:
                cx, cy = (gx+dx)*self.cell_size, (gy+dy)*self.cell_size
                cv2.circle(img, (cx,cy), 5, (0,0,255), -1)
        # Draw path
        for (gx, gy) in self.path:
            cv2.rectangle(img, (gx*self.cell_size, gy*self.cell_size), ((gx+1)*self.cell_size, (gy+1)*self.cell_size), (0,255,255), -1)
        # Draw start
        if self.start:
            cv2.rectangle(img, (self.start[0]*self.cell_size, self.start[1]*self.cell_size), ((self.start[0]+1)*self.cell_size, (self.start[1]+1)*self.cell_size), (0,255,0), -1)
        # Draw end
        if self.end:
            cv2.rectangle(img, (self.end[0]*self.cell_size, self.end[1]*self.cell_size), ((self.end[0]+1)*self.cell_size, (self.end[1]+1)*self.cell_size), (0,0,255), -1)
        # Draw drone
        if self.drone_pos:
            cv2.circle(img, (self.drone_pos[0]*self.cell_size+self.cell_size//2, self.drone_pos[1]*self.cell_size+self.cell_size//2), self.cell_size//3, (255,0,0), -1)
        # Draw drone paths (history)
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
        # Draw grid
        for i in range(self.grid_size+1):
            cv2.line(img, (0, i*self.cell_size), (self.grid_size*self.cell_size, i*self.cell_size), (200,200,200), 1)
            cv2.line(img, (i*self.cell_size, 0), (i*self.cell_size, self.grid_size*self.cell_size), (200,200,200), 1)
        # Draw tutorial/instructions overlay
        overlay_lines = [
            'Controls:',
            'Left-click: Set Start (green)',
            'Right-click: Add Dynamic Obstacle (blue)',
            'Middle-click: Add Obstacle (black)',
            'Drag corners: Resize Obstacle',
            'Left-click+drag: Move Obstacle',
            'Shift+Left-click: No-Fly Zone (red)',
            'Drag yellow: Edit Path',
            'Right-click on cell: Set End (red)',
            'Enter: Start Simulation',
        ]
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.45
        thickness = 1
        y0 = 18
        for i, line in enumerate(overlay_lines):
            cv2.putText(img, line, (8, y0 + i*18), font, font_scale, (40,40,40), thickness, cv2.LINE_AA)
        cv2.imshow(self.window_name, img)

    def get_map(self):
        return self.map.copy(), self.start, self.end, list(self.obstacles), dict(self.obstacle_sizes), list(self.dynamic_obstacles), list(self.nofly_zones)

    def set_path(self, path):
        self.path = path

    def set_drone_pos(self, pos):
        self.drone_pos = pos

    def wait_for_setup(self):
        print("\n[Map UI] Left-click: Set Start | Right-click: Set End | Middle-click: Add Obstacle | Drag corners to resize | Press Enter to confirm setup.")
        while True:
            self.draw()
            key = cv2.waitKey(50)
            if key == 13:  # Enter
                if self.start and self.end:
                    break
        print(f"[Map UI] Start: {self.start}, End: {self.end}, Obstacles: {len(self.obstacles)}")
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