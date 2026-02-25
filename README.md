# Kalman Filter Tutorial

Learn state estimation with hands-on robotics examples. Implement a Linear Kalman Filter from scratch and see it filter noisy sensor data in real time.

Part of [Leet Robotics](https://leet-robotics.com) — the hands-on robotics learning platform.

## What You'll Build

A Kalman Filter that estimates a robot's position from noisy odometry data. You'll see:
- **Red**: Raw noisy measurements (what the sensor gives you)
- **Green**: Your Kalman filter output (what you compute)
- **Blue**: Ground truth (what's actually happening)

## Prerequisites

- Python 3 + NumPy
- ROS2 Humble (for running the ROS nodes)
- Basic understanding of matrices and probability

## Quick Start (Local)

### 1. Clone and install

```bash
cd ~/ros2_ws/src
git clone https://github.com/leet-robotics/kalman-filter-tutorial.git
cd ~/ros2_ws
colcon build --packages-select kalman_filter
source install/setup.bash
```

### 2. Run the demo (no Gazebo needed)

```bash
ros2 launch kalman_filter test_no_gazebo.launch.py
```

This launches a simulated robot moving in a circle with noisy odometry, plus the Kalman filter node.

### 3. Visualize

Open Foxglove Studio and connect to `ws://localhost:9090`. Import the layout from `config/foxglove/kalman_layout.json`.

Or use the command line:
```bash
# See filtered output
ros2 topic echo /kalman/pose

# See noisy input
ros2 topic echo /odom

# See ground truth
ros2 topic echo /ground_truth
```

## Tutorial Steps

The tutorial walks you through implementing the Kalman Filter in 3 coding steps:

1. **Define the state transition matrix** (`linear_kf.py` — Step 6)
   - Fill in the 4x4 matrix F for a constant velocity model

2. **Implement the predict step** (`linear_kf.py` — Step 7)
   - State prediction: `x = F @ x`
   - Covariance prediction: `P = F @ P @ F.T + Q`

3. **Implement the update step** (`linear_kf.py` — Step 10)
   - Innovation, Kalman gain, state update, covariance update

### Starter Code

The `starter/` directory has the skeleton code with TODO placeholders. The `solution/` directory has the complete implementation.

To use the starter code:
```bash
cp starter/kalman_filter/linear_kf.py kalman_filter/linear_kf.py
```

To check against the solution:
```bash
diff kalman_filter/linear_kf.py solution/kalman_filter/linear_kf.py
```

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Noisy odometry (input) |
| `/ground_truth` | `nav_msgs/Odometry` | True position (reference) |
| `/kalman/pose` | `PoseWithCovarianceStamped` | Filtered estimate (output) |
| `/kalman/uncertainty` | `visualization_msgs/Marker` | Uncertainty ellipse (visualization) |

## Running with Gazebo

For the full simulation with a 3D robot:

```bash
ros2 launch kalman_filter demo.launch.py
```

This requires Gazebo Harmonic and the `ros_gz` bridge packages.

## Project Structure

```
kalman_filter/
├── __init__.py
├── linear_kf.py          # The Kalman Filter implementation (you edit this)
├── kalman_node.py         # ROS2 node that runs the filter
└── noisy_odom.py          # Simulated noisy odometry publisher
launch/
├── demo.launch.py         # Full demo with Gazebo
└── test_no_gazebo.launch.py  # Quick test without Gazebo
worlds/
└── simple_world.sdf       # Gazebo world with diff-drive robot
config/foxglove/
└── kalman_layout.json     # Foxglove Studio panel layout
starter/                   # Skeleton code with TODOs
solution/                  # Complete reference implementation
```

## License

MIT
