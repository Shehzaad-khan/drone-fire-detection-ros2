# Drone Fire Detection (ROS 2 + Gazebo)

Minimal ROS 2 simulation project for autonomous drone flight and fire-scene testing in Gazebo.

## Project Structure

- `src/drone_fire_detection`: ROS 2 package with launch files, world, model, config, and flight node.

## Quick Start

### 1) Build

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

## Notes

- Generated folders (`build/`, `install/`, `log/`) are ignored in Git.
- Sensitive env files (`.env`, `.env.*`) are also ignored.
