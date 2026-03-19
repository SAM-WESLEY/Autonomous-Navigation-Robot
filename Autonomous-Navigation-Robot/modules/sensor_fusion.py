import numpy as np
import time
import math
import random


class SensorFusion:
    """
    Fuses LiDAR, ultrasonic, and camera data into a unified
    obstacle map using a simple Kalman-inspired weighted average.
    """

    def __init__(self, grid_size=(50, 50)):
        self.grid_size      = grid_size
        self.obstacle_map   = np.zeros(grid_size, dtype=np.float32)
        self.robot_pos      = [grid_size[0] // 2, grid_size[1] // 2]
        self.robot_heading  = 0.0    # degrees, 0 = North
        self.lidar_data     = []
        self.ultrasonic_data = {}
        self.camera_data    = {}
        self.demo_mode      = True

    # ── LiDAR ──────────────────────────────────────────────────────────────────
    def update_lidar(self, scan_data=None):
        """
        Process LiDAR scan. scan_data = list of (angle_deg, distance_m).
        In demo mode, generates synthetic scan.
        """
        if scan_data is None:
            scan_data = self._demo_lidar_scan()

        self.lidar_data = scan_data
        for angle_deg, dist_m in scan_data:
            if dist_m < 0.1 or dist_m > 6.0:
                continue
            # Convert polar to grid coordinates
            angle_rad = math.radians(angle_deg + self.robot_heading)
            dist_cells = int(dist_m * 5)   # 5 cells per metre
            or_, oc = self.robot_pos
            nr = int(or_ + dist_cells * math.cos(angle_rad))
            nc = int(oc + dist_cells * math.sin(angle_rad))
            if 0 <= nr < self.grid_size[0] and 0 <= nc < self.grid_size[1]:
                self.obstacle_map[nr, nc] = min(1.0,
                    self.obstacle_map[nr, nc] + 0.3)

        # Decay old readings
        self.obstacle_map *= 0.98

    def _demo_lidar_scan(self):
        """Generate realistic synthetic LiDAR scan."""
        scan = []
        t    = time.time()
        for angle in range(0, 360, 5):
            # Add some static obstacles
            base_dist = 3.0
            if 45 <= angle <= 90:
                base_dist = 1.2 + 0.3 * math.sin(t)
            elif 180 <= angle <= 220:
                base_dist = 2.0 + 0.2 * math.cos(t * 0.5)
            noise = random.gauss(0, 0.05)
            scan.append((angle, max(0.15, base_dist + noise)))
        return scan

    # ── Ultrasonic ─────────────────────────────────────────────────────────────
    def update_ultrasonic(self, readings=None):
        """
        readings = dict: {'front': dist_cm, 'left': dist_cm, 'right': dist_cm}
        """
        if readings is None:
            readings = self._demo_ultrasonic()
        self.ultrasonic_data = readings

        DANGER_CM = 30
        for direction, dist_cm in readings.items():
            if dist_cm < DANGER_CM:
                dr, dc = self._direction_to_delta(direction)
                nr = self.robot_pos[0] + dr * 3
                nc = self.robot_pos[1] + dc * 3
                if 0 <= nr < self.grid_size[0] and 0 <= nc < self.grid_size[1]:
                    self.obstacle_map[nr, nc] = 1.0

    def _demo_ultrasonic(self):
        t = time.time()
        return {
            "front": max(10, 80 + 20 * math.sin(t * 0.3)),
            "left":  max(10, 60 + 15 * math.cos(t * 0.5)),
            "right": max(10, 70 + 10 * math.sin(t * 0.7)),
        }

    def _direction_to_delta(self, direction):
        return {"front": (-1, 0), "back": (1, 0),
                "left": (0, -1), "right": (0, 1)}.get(direction, (0, 0))

    # ── Camera ─────────────────────────────────────────────────────────────────
    def update_camera(self, frame=None):
        """Process camera frame for obstacle detection."""
        if frame is None:
            self.camera_data = {"obstacle_detected": False, "lane_clear": True}
            return
        try:
            import cv2
            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges   = cv2.Canny(blurred, 50, 150)
            lines   = cv2.HoughLinesP(edges, 1, np.pi/180, 50,
                                       minLineLength=50, maxLineGap=10)
            self.camera_data = {
                "obstacle_detected": lines is not None and len(lines) > 10,
                "lane_clear":        lines is None or len(lines) < 5,
            }
        except Exception:
            self.camera_data = {"obstacle_detected": False, "lane_clear": True}

    # ── Fused state ────────────────────────────────────────────────────────────
    def get_state(self):
        return {
            "robot_pos":       self.robot_pos,
            "robot_heading":   self.robot_heading,
            "obstacle_map":    self.obstacle_map.tolist(),
            "ultrasonic":      self.ultrasonic_data,
            "camera":          self.camera_data,
            "obstacle_nearby": any(
                v < 25 for v in self.ultrasonic_data.values()),
        }

    def move_robot(self, dr, dc):
        """Update robot position on grid."""
        nr = self.robot_pos[0] + dr
        nc = self.robot_pos[1] + dc
        if (0 <= nr < self.grid_size[0] and
                0 <= nc < self.grid_size[1] and
                self.obstacle_map[nr, nc] < 0.7):
            self.robot_pos = [nr, nc]
            return True
        return False
