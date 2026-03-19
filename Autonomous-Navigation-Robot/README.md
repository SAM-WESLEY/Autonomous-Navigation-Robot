# AI-Powered Autonomous Navigation Robot with Real-Time Path Planning

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.9+-3776AB?style=for-the-badge&logo=python&logoColor=white"/>
  <img src="https://img.shields.io/badge/ROS-Robot%20OS-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
  <img src="https://img.shields.io/badge/OpenCV-27338e?style=for-the-badge&logo=opencv&logoColor=white"/>
  <img src="https://img.shields.io/badge/A*-Path%20Planning-green?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Flask-Dashboard-000000?style=for-the-badge&logo=flask&logoColor=white"/>
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge"/>
</p>

> An intelligent robotic system that navigates dynamically using computer vision, sensor fusion, and real-time A* path planning — capable of autonomous obstacle avoidance and goal-directed movement without human intervention.

---

## 📌 Overview

AI-Powered Autonomous Navigation Robot is a complete robotics system that fuses data from LiDAR, ultrasonic sensors, and a camera to build a real-time occupancy grid map of its environment. An A* path planning algorithm computes the optimal route to the target, while a PID motor controller executes smooth, precise movements. A live Flask dashboard streams the robot's position, sensor data, and map in real time.

---

## ✨ Features

- 🚗 **Autonomous navigation** — goal-directed movement without human control
- 🧭 **A\* path planning** — optimal shortest-path computation on occupancy grid
- 🧠 **AI-based decision making** — dynamic re-planning when obstacles are detected
- 📷 **Vision processing** — OpenCV lane detection + obstacle identification
- 📡 **Sensor fusion** — LiDAR + Ultrasonic + Camera combined for accurate mapping
- 🚧 **Real-time obstacle avoidance** — dynamic replanning around new obstacles
- 🗺️ **SLAM-ready** — occupancy grid mapping with live visualisation
- 📊 **Live dashboard** — real-time robot position, map, sensor readings

---

## 🏗️ System Architecture

```
[Sensors + Camera]
    ├── LiDAR         → 360° distance mapping
    ├── Ultrasonic    → Close-range obstacle detection
    └── Pi Camera     → Lane detection + visual odometry
         ↓
[ROS Node System]
    ├── sensor_fusion.py   → Merges all sensor data
    ├── occupancy_map.py   → Builds real-time grid map
    ├── path_planner.py    → A* algorithm computes route
    └── motor_controller.py → PID control for movement
         ↓
[Raspberry Pi / Jetson Nano]
         ↓
[Motor Driver (L298N)]
    ├── Left Motors
    └── Right Motors
         ↓
[Flask Dashboard — Live Map + Telemetry]
```

---

## 🛠️ Tech Stack

| Component | Technology |
|---|---|
| Programming | Python 3.9+ / C++ |
| Robot OS | ROS Noetic / ROS2 Humble |
| Computer Vision | OpenCV |
| Path Planning | A* Algorithm (custom implementation) |
| Sensor Fusion | NumPy + Custom Kalman Filter |
| Motor Control | PID Controller + L298N Driver |
| Hardware | Raspberry Pi 4B / NVIDIA Jetson Nano |
| Sensors | RPLiDAR A1 + HC-SR04 Ultrasonic + Pi Camera |
| Dashboard | Flask + HTML + Chart.js |

---

## 🚀 Quick Start

### Prerequisites
```bash
pip install -r requirements.txt
```

### Run Simulation (No Hardware Needed)
```bash
git clone https://github.com/SAM-WESLEY/Autonomous-Navigation-Robot
cd Autonomous-Navigation-Robot
python simulate.py
```

### Run on Real Robot
```bash
# Start ROS core
roscore &

# Launch all nodes
python robot.py --mode real --target 5,8
```

### Access Dashboard
```
http://localhost:5000
```

---

## 🗺️ Path Planning (A* Algorithm)

```
Start: (0, 0)    Target: (10, 8)

Grid Map:
. . . . # # . . . .
. . . . # . . . . .
. . . # # . . . . .
. . . . . . . # . .
S . . . . . . # T .

Legend: S=Start  T=Target  #=Obstacle  .=Free
Path:   ─────────────────────────>
```

The A* algorithm uses Manhattan distance heuristic and dynamically updates the path when the sensor fusion module detects new obstacles.

---

## 📊 Sensor Fusion

| Sensor | Range | Update Rate | Purpose |
|---|---|---|---|
| RPLiDAR A1 | 0.15–12m | 10 Hz | 360° obstacle mapping |
| HC-SR04 | 2cm–4m | 20 Hz | Close-range avoidance |
| Pi Camera | — | 30 FPS | Lane + visual detection |

---

## 🎛️ Controls (Dashboard)

| Button | Action |
|---|---|
| Set Target | Click on map to set destination |
| Start | Begin autonomous navigation |
| Stop | Emergency stop |
| Reset | Clear path and return to origin |

---

## 🗂️ Project Structure

```
Autonomous-Navigation-Robot/
├── robot.py                     # Main robot controller
├── simulate.py                  # Simulation mode (no hardware)
├── modules/
│   ├── astar.py                 # A* path planning algorithm
│   ├── sensor_fusion.py         # LiDAR + Ultrasonic + Camera fusion
│   ├── occupancy_map.py         # Real-time grid map builder
│   ├── motor_controller.py      # PID motor control
│   └── vision.py                # OpenCV vision processing
├── ros/
│   ├── navigation_node.py       # ROS navigation node
│   └── sensor_node.py           # ROS sensor publisher node
├── maps/
│   └── default_map.json         # Default environment map
├── templates/
│   └── index.html               # Live robot dashboard
├── static/
│   └── style.css
├── requirements.txt
└── README.md
```

---

## 🌍 Applications

- 🏭 Warehouse automation and logistics robots
- 🚚 Last-mile delivery robots
- 🏥 Hospital medicine delivery
- 🏠 Home service robots
- 🚜 Agricultural field robots

---

## 🔮 Future Scope

- 🗺️ Full SLAM (Simultaneous Localisation and Mapping)
- 🤖 Deep Reinforcement Learning navigation
- 🚁 Drone adaptation with 3D path planning
- 🌐 Multi-robot coordination
- 🧠 Natural language goal commands

---

## 📬 Contact

**Sam Wesley S**
📧 samwesley@karunya.edu.in
🔗 [LinkedIn](https://linkedin.com/in/samwesleys)
🐙 [GitHub](https://github.com/SAM-WESLEY)

---

<p align="center">
  <i>Built with ❤️ at Karunya Institute of Technology and Sciences</i>
</p>

<p align="center">If this project helped you, please give it a ⭐</p>
