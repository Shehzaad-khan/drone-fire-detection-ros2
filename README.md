# Drone Fire Detection Simulation (ROS 2 + Gazebo)

This project simulates an autonomous drone operating in a fire-response environment using ROS 2 and Gazebo.  
It provides a complete mini workflow for launching a custom world, spawning a drone model, and running an autonomous flight node that follows mission parameters.

## What This Project Does

- Launches a fire-scene simulation world in Gazebo.
- Spawns a drone model configured for autonomous operation.
- Runs a ROS 2 flight control node for waypoint-based movement.
- Uses configurable flight parameters for behavior tuning.
- Supports testing and demonstration of drone navigation in emergency-like scenarios.

## Core Components

- `launch/fire_world.launch.py`  
  Starts the full simulation stack (world + drone + nodes).

- `worlds/fire_world.sdf`  
  Defines the environment where fire-response behavior is tested.

- `models/drone/model.sdf`  
  Contains the drone model used in simulation.

- `scripts/autonomous_flight_node.py`  
  Handles autonomous drone flight logic.

- `config/flight_params.yaml`  
  Stores mission and controller parameters.

## Run the Project

### 1) Build workspace

```bash
colcon build
```

### 2) Source workspace

```bash
source install/setup.bash
```

### 3) Launch simulation

```bash
ros2 launch drone_fire_detection fire_world.launch.py
```

## Use Cases

- Academic mini-projects in robotics and autonomous systems.
- Testing ROS 2 drone behaviors in a safe virtual environment.
- Demonstrating simulation-first workflows before real-world deployment.
