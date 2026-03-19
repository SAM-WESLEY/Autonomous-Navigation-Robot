import time
import math

# ── PID Motor Controller ───────────────────────────────────────────────────────
class MotorController:
    def __init__(self):
        self.kp = 1.2
        self.ki = 0.05
        self.kd = 0.8
        self._prev_error  = 0.0
        self._integral    = 0.0
        self._last_time   = time.time()
        self.left_speed   = 0
        self.right_speed  = 0
        self.gpio_enabled = False
        self._init_gpio()

    def _init_gpio(self):
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            # L298N Motor Driver pins
            GPIO.setup([18, 19, 20, 21], GPIO.OUT)
            self.pwm_l = GPIO.PWM(18, 1000)
            self.pwm_r = GPIO.PWM(20, 1000)
            self.pwm_l.start(0)
            self.pwm_r.start(0)
            self.GPIO          = GPIO
            self.gpio_enabled  = True
            print("[MotorController] GPIO ready.")
        except Exception:
            print("[MotorController] GPIO unavailable — simulation mode.")

    def pid(self, setpoint, measured):
        now   = time.time()
        dt    = max(now - self._last_time, 1e-6)
        error = setpoint - measured
        self._integral   += error * dt
        derivative        = (error - self._prev_error) / dt
        output            = (self.kp * error +
                             self.ki * self._integral +
                             self.kd * derivative)
        self._prev_error  = error
        self._last_time   = now
        return max(-100, min(100, output))

    def move_towards(self, current_heading, target_heading, speed=60):
        heading_error = target_heading - current_heading
        heading_error = (heading_error + 180) % 360 - 180
        correction    = self.pid(0, heading_error)
        left  = int(speed + correction)
        right = int(speed - correction)
        self.left_speed  = max(0, min(100, left))
        self.right_speed = max(0, min(100, right))
        self._set_motors(self.left_speed, self.right_speed)
        return self.left_speed, self.right_speed

    def stop(self):
        self.left_speed = self.right_speed = 0
        self._set_motors(0, 0)

    def _set_motors(self, left_pct, right_pct):
        if not self.gpio_enabled:
            return
        self.pwm_l.ChangeDutyCycle(left_pct)
        self.pwm_r.ChangeDutyCycle(right_pct)


# ── Occupancy Map ──────────────────────────────────────────────────────────────
class OccupancyMap:
    def __init__(self, rows=50, cols=50, resolution=0.2):
        import numpy as np
        self.rows       = rows
        self.cols       = cols
        self.resolution = resolution   # metres per cell
        self.grid       = np.zeros((rows, cols), dtype=np.float32)
        self.robot_pos  = [rows // 2, cols // 2]

    def update(self, sensor_state):
        omap = sensor_state.get("obstacle_map", [])
        if omap:
            import numpy as np
            self.grid = np.clip(np.array(omap, dtype=np.float32), 0, 1)

    def is_free(self, r, c, threshold=0.5):
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return self.grid[r, c] < threshold
        return False

    def to_binary(self, threshold=0.5):
        return (self.grid > threshold).astype(int)


# ── Vision Module ──────────────────────────────────────────────────────────────
class VisionProcessor:
    def __init__(self):
        self.last_result = {}

    def process(self, frame=None):
        if frame is None:
            return {"status": "no_frame", "obstacles": [], "lane_clear": True}
        try:
            import cv2
            import numpy as np
            h, w = frame.shape[:2]
            roi   = frame[h//2:, :]
            gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur  = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 50, 150)
            lines = cv2.HoughLinesP(edges, 1, math.pi/180, 50,
                                     minLineLength=50, maxLineGap=10)
            result = {
                "status":    "ok",
                "obstacles": len(lines) if lines is not None else 0,
                "lane_clear": lines is None or len(lines) < 5,
            }
            self.last_result = result
            return result
        except Exception as e:
            return {"status": f"error: {e}", "obstacles": 0, "lane_clear": True}
