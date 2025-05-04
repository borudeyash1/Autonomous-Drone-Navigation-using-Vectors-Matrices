import numpy as np
import cv2
from typing import List, Tuple, Optional
import os

class ObstacleDetector:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """
        Initialize the obstacle detector with camera parameters.
        
        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Camera distortion coefficients
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        
        # Parameters for simple obstacle detection
        self.min_area = 100  # Minimum contour area
        self.threshold = 50  # Threshold for edge detection
        self.max_points = 10000  # Maximum number of points to process
        self.downsample_factor = 4  # Factor to downsample depth image
    
    def process_depth_image(self, depth_image: np.ndarray) -> List[Tuple[np.ndarray, float]]:
        """
        Process depth image to detect obstacles.
        
        Args:
            depth_image: Depth image from depth camera
            
        Returns:
            List of (position, size) tuples for detected obstacles
        """
        # Downsample depth image
        height, width = depth_image.shape
        new_height = height // self.downsample_factor
        new_width = width // self.downsample_factor
        depth_image = cv2.resize(depth_image, (new_width, new_height))
        
        # Convert depth image to point cloud
        points = []
        
        for y in range(0, new_height, 2):  # Process every other row
            for x in range(0, new_width, 2):  # Process every other column
                depth = depth_image[y, x]
                if depth > 0:  # Valid depth measurement
                    # Scale coordinates back to original size
                    orig_x = x * self.downsample_factor
                    orig_y = y * self.downsample_factor
                    # Convert pixel coordinates to 3D points
                    point = self._pixel_to_point(orig_x, orig_y, depth)
                    points.append(point)
        
        # Limit number of points to process
        if len(points) > self.max_points:
            indices = np.random.choice(len(points), self.max_points, replace=False)
            points = [points[i] for i in indices]
        
        # Cluster points to identify obstacles
        obstacles = self._cluster_points(points)
        return obstacles
    
    def process_rgb_image(self, image: np.ndarray) -> List[Tuple[np.ndarray, float]]:
        """
        Process RGB image to detect obstacles using simple edge detection.
        
        Args:
            image: RGB image from camera
            
        Returns:
            List of (position, size) tuples for detected obstacles
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Detect edges using Canny
        edges = cv2.Canny(blurred, self.threshold, self.threshold * 2)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process contours
        obstacles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w/2
                center_y = y + h/2
                
                # Convert to 3D position (approximate)
                position = self._pixel_to_point(int(center_x), int(center_y), 10.0)  # Assuming 10m distance
                size = max(w, h) / image.shape[1]  # Normalized size
                
                obstacles.append((position, size))
        
        return obstacles
    
    def _pixel_to_point(self, x: int, y: int, depth: float) -> np.ndarray:
        """
        Convert pixel coordinates to 3D point in camera frame.
        
        Args:
            x, y: Pixel coordinates
            depth: Depth value
            
        Returns:
            3D point in camera frame
        """
        # Convert pixel coordinates to normalized coordinates
        x_norm = (x - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0]
        y_norm = (y - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1]
        
        # Convert to 3D point
        point = np.array([
            x_norm * depth,
            y_norm * depth,
            depth
        ])
        
        return point
    
    def _cluster_points(self, points: List[np.ndarray]) -> List[Tuple[np.ndarray, float]]:
        """
        Cluster 3D points to identify obstacles.
        
        Args:
            points: List of 3D points
            
        Returns:
            List of (position, size) tuples for detected obstacles
        """
        # Simple clustering using DBSCAN
        from sklearn.cluster import DBSCAN
        
        if not points:
            return []
        
        points_array = np.array(points)
        
        # Use more efficient clustering parameters
        clustering = DBSCAN(
            eps=2.0,  # Increased epsilon to reduce number of clusters
            min_samples=3,  # Reduced minimum samples
            n_jobs=-1  # Use all available cores
        ).fit(points_array)
        
        obstacles = []
        unique_labels = set(clustering.labels_)
        
        # Early termination if only noise points
        if len(unique_labels) == 1 and -1 in unique_labels:
            return obstacles
        
        for label in unique_labels:
            if label == -1:  # Noise points
                continue
                
            cluster_points = points_array[clustering.labels_ == label]
            position = np.mean(cluster_points, axis=0)
            size = np.max(np.linalg.norm(cluster_points - position, axis=1))
            
            obstacles.append((position, size))
        
        return obstacles 