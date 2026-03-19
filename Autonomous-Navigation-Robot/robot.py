from flask import Flask, render_template, jsonify, request
import threading
import time
import math
import argparse
from modules.astar          import AStarPlanner
from modules.sensor_fusion  import SensorFusion
from modules.motor_controller import MotorController, OccupancyMap

app      = Flask(__name__)
GRID     = 50
planner  = AStarPlanner(grid_size=(GRID, GRID))
sensors  = SensorFusion(grid_size=(GRID, GRID))
motors   = MotorController()
omap     = OccupancyMap(rows=GRID, cols=GRID)

state = {
    "robot_pos":     [GRID//2, GRID//2],
    "robot_heading": 0.0,
    "target":        None,
    "path":          [],
    "path_idx":      0,
    "status":        "IDLE",
    "ultrasonic":    {},
    "speed_l":       0,
    "speed_r":       0,
    "obstacle_map":  [],
    "step_count":    0,
    "running":       False,
}


def navigation_loop():
    """Main robot navigation loop."""
    while True:
        if not state["running"] or state["target"] is None:
            time.sleep(0.2)
            continue

        # Update sensors
        sensors.update_lidar()
        sensors.update_ultrasonic()
        sensor_state = sensors.get_state()
        omap.update(sensor_state)

        state["ultrasonic"]   = sensor_state["ultrasonic"]
        state["obstacle_map"] = sensor_state["obstacle_map"]
        state["robot_pos"]    = sensors.robot_pos

        # Update A* grid with new obstacle info
        binary = omap.to_binary()
        for r in range(GRID):
            for c in range(GRID):
                planner.grid[r, c] = binary[r][c]

        pos    = tuple(sensors.robot_pos)
        target = tuple(state["target"])

        # Reached target?
        if pos == target or (abs(pos[0]-target[0]) <= 1 and abs(pos[1]-target[1]) <= 1):
            motors.stop()
            state["status"]  = "REACHED TARGET"
            state["running"] = False
            state["speed_l"] = state["speed_r"] = 0
            continue

        # Re-plan if obstacle nearby
        if sensor_state["obstacle_nearby"] or not state["path"]:
            path = planner.plan(pos, target)
            if path:
                state["path"]    = [list(p) for p in path]
                state["path_idx"] = 0
                state["status"]  = "NAVIGATING"
            else:
                motors.stop()
                state["status"]  = "PATH BLOCKED"
                state["speed_l"] = state["speed_r"] = 0
                time.sleep(0.5)
                continue

        # Follow path
        if state["path_idx"] < len(state["path"]):
            next_wp = state["path"][state["path_idx"]]
            dr = next_wp[0] - sensors.robot_pos[0]
            dc = next_wp[1] - sensors.robot_pos[1]
            target_heading = math.degrees(math.atan2(dc, -dr)) % 360
            l, r = motors.move_towards(sensors.robot_heading, target_heading)
            state["speed_l"] = l
            state["speed_r"] = r
            sensors.move_robot(dr // max(abs(dr), 1) if dr else 0,
                               dc // max(abs(dc), 1) if dc else 0)
            sensors.robot_heading = target_heading

            if sensors.robot_pos == next_wp:
                state["path_idx"] += 1

        state["step_count"] += 1
        time.sleep(0.3)


# ── Flask Routes ───────────────────────────────────────────────────────────────
@app.route('/')
def index(): return render_template('index.html')


@app.route('/status')
def status():
    return jsonify({k: v for k, v in state.items() if k != "obstacle_map"})


@app.route('/map')
def map_data():
    return jsonify({
        "obstacle_map": state["obstacle_map"],
        "robot_pos":    state["robot_pos"],
        "path":         state["path"],
        "target":       state["target"],
        "grid_size":    GRID,
    })


@app.route('/set_target', methods=['POST'])
def set_target():
    data   = request.json
    target = [int(data['row']), int(data['col'])]
    if planner.grid[target[0]][target[1]] == 1:
        return jsonify({"error": "Target is an obstacle!"}), 400
    state["target"]  = target
    state["path"]    = []
    state["path_idx"] = 0
    state["status"]  = "PLANNING"
    state["running"] = True
    return jsonify({"status": "target set", "target": target})


@app.route('/start', methods=['POST'])
def start():
    state["running"] = True
    state["status"]  = "NAVIGATING"
    return jsonify({"status": "started"})


@app.route('/stop', methods=['POST'])
def stop():
    state["running"] = False
    state["status"]  = "STOPPED"
    motors.stop()
    return jsonify({"status": "stopped"})


@app.route('/reset', methods=['POST'])
def reset():
    state["running"]     = False
    state["target"]      = None
    state["path"]        = []
    state["path_idx"]    = 0
    state["status"]      = "IDLE"
    state["step_count"]  = 0
    sensors.robot_pos    = [GRID//2, GRID//2]
    sensors.robot_heading = 0.0
    planner.clear_grid()
    motors.stop()
    return jsonify({"status": "reset"})


@app.route('/add_obstacle', methods=['POST'])
def add_obstacle():
    d = request.json
    planner.add_obstacle_circle((d['row'], d['col']), radius=d.get('radius', 2))
    sensors.obstacle_map[d['row'], d['col']] = 1.0
    return jsonify({"status": "obstacle added"})


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', default='sim', choices=['sim', 'real'])
    parser.add_argument('--target', default=None)
    args = parser.parse_args()

    # Add some demo obstacles
    planner.add_obstacle_circle((20, 20), radius=4)
    planner.add_obstacle_circle((35, 15), radius=3)
    planner.add_obstacle_circle((15, 35), radius=3)

    if args.target:
        r, c = map(int, args.target.split(','))
        state["target"]  = [r, c]
        state["running"] = True

    t = threading.Thread(target=navigation_loop, daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=5000, debug=False)
