"""
simulate.py — Run Autonomous Navigation Robot in simulation mode.
No hardware required. Visualises the A* path planning in terminal.
"""
import time
import threading
from modules.astar         import AStarPlanner
from modules.sensor_fusion import SensorFusion

GRID = 30

def clear(): print("\033[H\033[J", end="")

def render(planner, robot_pos, path, target):
    grid_vis = []
    for r in range(GRID):
        row = []
        for c in range(GRID):
            if planner.grid[r, c] == 1:
                row.append('██')
            else:
                row.append('  ')
        grid_vis.append(row)

    for (r, c) in (path or []):
        if 0 <= r < GRID and 0 <= c < GRID:
            grid_vis[r][c] = '·'

    if target:
        grid_vis[target[0]][target[1]] = '🎯'
    grid_vis[robot_pos[0]][robot_pos[1]] = '🤖'

    clear()
    print("=" * 62)
    print("  AI Autonomous Navigation Robot — Simulation")
    print(f"  Robot: {robot_pos}  Target: {target}  Path: {len(path)} steps")
    print("=" * 62)
    for row in grid_vis:
        print('|' + ''.join(row) + '|')
    print("=" * 62)

def run_simulation():
    planner  = AStarPlanner(grid_size=(GRID, GRID))
    sensors  = SensorFusion(grid_size=(GRID, GRID))

    # Add obstacles
    planner.add_obstacle_circle((10, 10), radius=3)
    planner.add_obstacle_circle((18, 8),  radius=2)
    planner.add_obstacle_circle((8, 20),  radius=2)

    start  = (2, 2)
    target = (GRID-3, GRID-3)
    sensors.robot_pos = list(start)

    print("Planning path...")
    path = planner.plan(start, target)

    if not path:
        print("[Simulation] No path found!")
        return

    print(f"Path found! {len(path)} waypoints. Starting navigation...")
    time.sleep(1)

    for i, wp in enumerate(path):
        sensors.update_lidar()
        sensors.update_ultrasonic()
        sensors.robot_pos = list(wp)
        render(planner, list(wp), path[i:], list(target))
        time.sleep(0.15)

    print("\n✅ Target reached! Navigation complete.")

if __name__ == "__main__":
    run_simulation()
