# 🔥 Drone-Based Fire Detection System
### UE23CS343BB7 — Mobile and Autonomous Robotics
**PES University, Bangalore | Department of Computer Science & Engineering**

---

## 📌 Project Overview

This project implements a **simulation-based autonomous drone fire detection system** using ROS2 and Gazebo. A drone is deployed in a virtual environment where it autonomously navigates the area, detects fire sources using computer vision, and triggers alerts with real-time visualization — all without any physical hardware.

The system is designed to simulate a real-world use case where drones are deployed over areas prone to fire (forests, warehouses, industrial zones) to detect and report fire outbreaks faster than traditional methods.

---

## 💡 The Idea

The core idea is simple:
- A drone flies over an environment in a systematic search pattern
- Its onboard camera continuously streams video
- A fire detection algorithm watches the feed for red/orange heat signatures
- When fire is detected, the system logs the location and raises a visual alert

Everything runs in simulation — no hardware required.

---

## 🛠️ Tech Stack

| Tool | Purpose |
|---|---|
| **ROS2 Jazzy / Humble** | Robot middleware — handles all nodes, topics, communication |
| **Gazebo Harmonic** | 3D physics simulation environment |
| **Python (rclpy)** | Writing all ROS2 nodes |
| **OpenCV** | Camera feed processing and fire detection |
| **RViz2** | Real-time visualization of drone path and fire markers |
| **SDF (Simulation Description Format)** | Defining the drone model and world |

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────┐
│                    Gazebo Simulation                 │
│  ┌──────────────┐        ┌─────────────────────┐    │
│  │  Fire World  │        │   Drone + Camera     │    │
│  │  - fire1     │        │   /drone/camera      │    │
│  │  - fire2     │        │   topic (30fps)      │    │
│  │  - building  │        └─────────────────────┘    │
│  └──────────────┘                                    │
└─────────────────────────────────────────────────────┘
           │                        │
           ▼                        ▼
┌─────────────────┐      ┌──────────────────────┐
│  Flight Node    │      │  Fire Detection Node  │
│  (Member 2)     │      │  (Member 3)           │
│                 │      │                       │
│ - Search pattern│      │ - Subscribes to camera│
│ - Waypoints     │      │ - HSV color detection │
│ - cmd_vel pub   │      │ - Publishes to        │
│                 │      │   /fire_detected      │
└─────────────────┘      └──────────────────────┘
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │  Alert + Viz Node     │
                         │  (Member 4)           │
                         │                       │
                         │ - Subscribes to       │
                         │   /fire_detected      │
                         │ - Logs coordinates    │
                         │ - RViz2 markers       │
                         │ - Alert trigger       │
                         └──────────────────────┘
```

---

## 📁 Project Structure

```
drone_fire_ws/
└── src/
    └── drone_fire_detection/
        ├── worlds/
        │   └── fire_world.sdf        # Gazebo world with fire sources + obstacles
        ├── models/
        │   └── drone/
        │       └── model.sdf         # Drone model with camera sensor
        ├── launch/
        │   └── fire_world.launch.py  # Launch file to start the full simulation
        ├── CMakeLists.txt            # Build configuration
        └── package.xml              # ROS2 package descriptor
```

> **Note:** Flight node, detection node, and alert node (Python scripts) will be added by Members 2, 3, and 4 respectively inside the same package.

---

## 🚀 How to Run

### Prerequisites
- Ubuntu 22.04 with ROS2 Humble OR Ubuntu 24.04 with ROS2 Jazzy
- Gazebo Harmonic

### Setup
```bash
# Install Gazebo bridge
sudo apt install ros-humble-ros-gz -y   # For Humble
# OR
sudo apt install ros-jazzy-ros-gz -y    # For Jazzy

# Build the workspace
cd ~/drone_fire_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch
```bash
ros2 launch drone_fire_detection fire_world.launch.py
```

> If you're on WSL2 (Windows), run these before launching:
> ```bash
> export LIBGL_ALWAYS_SOFTWARE=1
> export MESA_GL_VERSION_OVERRIDE=3.3
> ```

---

## 👥 Team Member Breakdown

### Member 1 — Environment & Drone Setup ✅ COMPLETE
**Responsibility:** Set up the ROS2 workspace, create the Gazebo simulation world, and define the drone model.

**What was done:**
- Created the ROS2 workspace (`drone_fire_ws`) with package `drone_fire_detection`
- Wrote `fire_world.sdf` — a Gazebo Harmonic world containing:
  - A 50x50m ground plane
  - 2 fire sources (orange cylinders) placed at coordinates (5,3) and (-4,6)
  - A building obstacle at (2,-3)
  - Proper physics, rendering and lighting plugins
- Wrote `model.sdf` — a drone model with:
  - Realistic mass and inertia properties
  - A forward-facing camera sensor streaming at 30fps to `/drone/camera` topic
- Wrote `fire_world.launch.py` — a single launch file that starts the entire simulation
- Configured `CMakeLists.txt` to properly install all directories

---

### Member 2 — Autonomous Flight & Navigation 🔲 PENDING
**Responsibility:** Write a ROS2 Python node that makes the drone autonomously navigate the environment.

**To implement:**
- Takeoff sequence
- Lawnmower/spiral search pattern using waypoints
- Publish velocity commands to move the drone
- Landing when search is complete

---

### Member 3 — Fire Detection Algorithm 🔲 PENDING
**Responsibility:** Process the drone's camera feed and detect fire using OpenCV.

**To implement:**
- Subscribe to `/drone/camera` topic
- Convert ROS2 image messages using `cv_bridge`
- Apply HSV color masking for red/orange fire detection
- Draw bounding boxes around detected fire regions
- Publish detection results to `/fire_detected` topic

---

### Member 4 — Alert System + Visualization + Report 🔲 PENDING
**Responsibility:** Handle fire alerts, RViz2 visualization, demo recording, and the final report.

**To implement:**
- Subscribe to `/fire_detected` topic
- Log fire location coordinates to terminal
- Publish RViz2 markers showing fire positions on the map
- Record a `ros2 bag` demo of the full system
- Compile and write the final project report

---

## 📡 ROS2 Topics

| Topic | Type | Published By | Subscribed By |
|---|---|---|---|
| `/drone/camera` | `sensor_msgs/Image` | Gazebo (drone sensor) | Member 3 |
| `/drone/cmd_vel` | `geometry_msgs/Twist` | Member 2 (flight node) | Gazebo |
| `/fire_detected` | `std_msgs/String` | Member 3 (detection node) | Member 4 |
| `/fire_markers` | `visualization_msgs/Marker` | Member 4 (alert node) | RViz2 |

---

## 📝 Notes

- The fire sources are represented as bright orange/red cylinders in Gazebo — easily detectable by HSV color filtering
- The drone model is intentionally simple (no propeller animations) to keep simulation lightweight
- The system is fully modular — each member's node is independent and communicates only through ROS2 topics
- Future extension to real hardware would only require replacing the simulated sensor/actuator interfaces

---

*Mini Project | UE23CS343BB7 | PES University Bangalore*
