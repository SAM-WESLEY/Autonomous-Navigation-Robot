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

AI-Powered Autonomous Navigation Robot is a complete robotics system that fuses data from LiDAR, ultrasonic sensors, and a camera to build a real-time occupancy grid map of its environment. An A* path planning algorithm computes the optimal route to the target, while a PID motor controller executes smooth, precise movements. A live Flask dashboard streams the robot's position, sensor data, and map in real time — accessible from any browser over WiFi.

---

## ✨ Features

- 🚗 **Autonomous navigation** — goal-directed movement without human control
- 🧭 **A\* path planning** — optimal shortest-path computation on occupancy grid
- 🧠 **AI-based decision making** — dynamic re-planning when obstacles are detected
- 📷 **Vision processing** — OpenCV lane detection and obstacle identification
- 📡 **Sensor fusion** — LiDAR + Ultrasonic + Camera combined for accurate mapping
- 🚧 **Real-time obstacle avoidance** — dynamic replanning around new obstacles
- 🗺️ **Live occupancy grid** — real-time map visualisation on web dashboard
- 📊 **Live dashboard** — robot position, path, sensor readings from any browser

---

## 🏗️ System Architecture

```
[Sensors + Camera]
    ├── RPLiDAR A1    → 360° distance mapping
    ├── HC-SR04       → Close-range obstacle detection
    └── Pi Camera     → Lane detection + visual odometry
         ↓
[ROS Node System]
    ├── sensor_fusion.py    → Merges all sensor data
    ├── occupancy_map.py    → Builds real-time grid map
    ├── astar.py            → A* algorithm computes route
    └── motor_controller.py → PID control for movement
         ↓
[Raspberry Pi 4B / NVIDIA Jetson Nano]
         ↓
[Motor Driver — L298N]
    ├── Left Motors
    └── Right Motors
         ↓
[Flask Dashboard — any browser on same WiFi]
```

---

## 🛠️ Tech Stack

| Component | Technology |
|---|---|
| Programming | Python 3.9+ |
| Robot OS | ROS Noetic / ROS2 Humble |
| Path Planning | A* Algorithm (custom implementation) |
| Computer Vision | OpenCV |
| Sensor Fusion | NumPy + Kalman Filter |
| Motor Control | PID Controller + L298N Driver |
| Hardware | Raspberry Pi 4B / NVIDIA Jetson Nano |
| Sensors | RPLiDAR A1 + HC-SR04 + Pi Camera |
| Backend | Flask (Python) |
| Frontend | HTML + CSS + Canvas API |

---

## 🚀 Quick Start

### Prerequisites
```bash
pip install flask numpy opencv-python
```

### Run Simulation (No Hardware Needed)
```bash
git clone https://github.com/SAM-WESLEY/Autonomous-Navigation-Robot
cd Autonomous-Navigation-Robot
python simulate.py
```

### Run on Real Robot
```bash
python robot.py --mode real --target 40,40
```

### Access Dashboard
Open any browser:
```
http://localhost:5000
```

---

## 🗺️ Path Planning (A* Algorithm)

```
Start: (2, 2)    Target: (47, 47)

Grid Map (simplified):
. . . . # # . . . .
. . . . # . . . . .
S . . # # . . . . .
. . . . . . . # . .
. . . . . . . # . T

Legend: S=Start  T=Target  #=Obstacle  .=Free  *=Path
```

The A* planner uses Manhattan distance heuristic with 8-directional movement and dynamically updates the path when new obstacles are detected by the sensor fusion module.

---

## 📊 Sensor Specifications

| Sensor | Range | Update Rate | Purpose |
|---|---|---|---|
| RPLiDAR A1 | 0.15–12m | 10 Hz | 360° obstacle mapping |
| HC-SR04 | 2cm–4m | 20 Hz | Close-range avoidance |
| Pi Camera | — | 30 FPS | Lane + visual detection |

---

## 🎛️ Dashboard Controls

| Control | Action |
|---|---|
| Click on map | Set navigation target |
| ▶ Start | Begin autonomous navigation |
| ⏹ Stop | Emergency stop |
| ↺ Reset | Clear path and return to origin |
| + Obstacle | Add random obstacle to map |

---

## 🗂️ Project Structure

```
Autonomous-Navigation-Robot/
├── robot.py                     # Main Flask app + navigation loop
├── simulate.py                  # Terminal simulation (no hardware needed)
├── modules/
│   ├── astar.py                 # A* path planning algorithm
│   ├── sensor_fusion.py         # LiDAR + Ultrasonic + Camera fusion
│   ├── motor_controller.py      # PID motor control + Occupancy Map
│   └── __init__.py
├── templates/
│   └── index.html               # Live robot dashboard (canvas map)
├── static/
│   └── style.css
├── maps/
│   └── default_map.json         # Default environment map
├── requirements.txt
└── README.md
```

---

## 🌍 Applications

- 🏭 Warehouse automation and logistics robots
- 🚚 Last-mile delivery robots
- 🏥 Hospital medicine and supply delivery
- 🏠 Home service and companion robots
- 🚜 Agricultural field monitoring robots

---

## 🔮 Future Scope

- 🗺️ Full SLAM (Simultaneous Localisation and Mapping)
- 🤖 Deep Reinforcement Learning navigation
- 🚁 Drone adaptation with 3D path planning
- 🌐 Multi-robot swarm coordination
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
