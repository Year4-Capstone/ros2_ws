[« Back to ros2_ws README](../../README.md)
# robot_bringup

## Overview

Hardware interface and launch package for the Ibex robot using ros2_control. Provides integration between ROS2 controllers and Phidget BLDC motor controllers for differential drive operation, along with simulation support, SLAM, and Nav2 navigation capabilities.

## Hardware Setup

### Hub Configuration

**Hub 1 (Serial: 527103):**
- Port 2: Back Left wheel
- Port 3: Back Right wheel

**Hub 2 (Serial: 527164):**
- Port 2: Front Left wheel
- Port 3: Front Right wheel

**Note:** port configuration might be wrong

## Dependencies

### ROS2 Dependencies
- `rclcpp` - ROS2 C++ client library
- `rclcpp_lifecycle` - Lifecycle node support
- `hardware_interface` - ros2_control hardware interface
- `pluginlib` - Plugin loading system
- `controller_manager` - Controller lifecycle management
- `diff_drive_controller` - Differential drive controller
- `joint_state_broadcaster` - Joint state publisher
- `robot_state_publisher` - TF broadcaster
- `robot_description` - Robot URDF package
- `xacro` - URDF processing
- `slam_toolbox` - SLAM implementation
- `nav2_*` - Navigation2 stack
- `twist_mux` - Velocity command multiplexer
- `ros_gz_bridge` - Gazebo-ROS2 bridge


### System Dependencies
- Phidget22 C library (`libphidget22`)

### Install Dependencies

**Using rosdep (recommended):**
```bash
cd ~/ros2_ws
rosdep install --from-paths src/robot_bringup --ignore-src -r -y
```

**Manual Phidget22 installation:**
View README.md for phidgets test if required

**Note:** Ensure `phidgets_test` and `robot_description` packages are built before building `robot_bringup`.

## Usage

### Building the Package

```bash
cd ~/ros2_ws
colcon build --packages-select robot_teleop phidgets_test robot_description robot_bringup
source install/setup.bash
```
## Configuration Files
### Twist Mux Configuration

**Topics** (`config/twist_mux/twist_mux_topics.yaml`)
- **Gamepad input:**
  - Topic: `/cmd_vel_gamepad_stamped`
  - Priority: 100 (highest)
  - Timeout: 0.5 s
- **Navigation input:**
  - Topic: `/cmd_vel_nav_stamped`
  - Priority: 10
  - Timeout: 1.0 s
- Output: `/diff_drive_base_controller/cmd_vel`
- Uses stamped messages

## Launch Files

### 1. Hardware Launch (ibex.launch.py)

Launch the physical robot with hardware interface.

**Note:** Review README.md for the phidgets_test package, ideally motors should be homed before driving

**Terminal 1:** Start joy node (requires gamepad connected)
```bash
ros2 launch robot_teleop teleop_joy_launch.py
```
**Note:** Review robot_teleop README.md for more information about the launch file.

**Note:** Ensure the controller is behaving correctly before moving on.

**Terminal 2:** Basic launch (hardware + visualization):
```bash
ros2 launch robot_bringup ibex.launch.py
```

This starts:
- **Controller Manager** - Manages ros2_control lifecycle
- **Hardware Interface** - Connects to Phidget motors
- **Joint State Broadcaster** - Publishes joint states to `/joint_states`
- **Diff Drive Controller** - Provides velocity control via `/diff_drive_base_controller/cmd_vel`
- **Robot State Publisher** - Publishes TF transforms
- **RViz2** - Visualization

### 2. Simulation Launch (ibex_sim.launch.py)

Launch the robot in Gazebo simulation with Nav2 navigation stack. This launch file includes the teleop node.

```bash
ros2 launch robot_bringup ibex_sim.launch.py
```

**Arguments:**
- `world` - World file to load (default: `cave_world.world`)
  - Options: `small_house.world`, `cave_world.world`, `empty_world.sdf`
- `map_name` - Map name for navigation (default: `cave_world`)
  - Options: `small_house`, `cave_world`

**Example with custom world:**
```bash
ros2 launch robot_bringup ibex_sim.launch.py world:=small_house.world map_name:=small_house
```

This starts:
- **Gazebo Simulation** - Physics simulation environment
- **Joint State Broadcaster** - Publishes simulated joint states
- **Diff Drive Controller** - Simulated differential drive control
- **Twist Mux** - Multiplexes gamepad and navigation commands
- **Nav2 Stack** - Full navigation system with:
  - Map Server
  - AMCL (localization)
  - Planner Server
  - Controller Server
  - Behavior Server
  - BT Navigator
  - Lifecycle Manager
- **Ground Truth Bridge** - Publishes GT pose from Gazebo to `/ground_truth_pose`
- **RViz2** - Navigation visualization

### 3. SLAM Launch (ibex_sim_slam.launch.py)

Launch the robot in simulation with SLAM mapping.

```bash
ros2 launch robot_bringup ibex_sim_slam.launch.py
```

This starts:
- **Gazebo Simulation** - Physics simulation environment
- **SLAM Toolbox** - Online async SLAM
- **Joint State Broadcaster** - Publishes simulated joint states
- **Diff Drive Controller** - Simulated differential drive control
- **Ground Truth Bridge** - Publishes GT pose from Gazebo
- **RViz2** - SLAM visualization



### Visualization Tools (Hardware Only)

**Joint velocity tracking:**
```bash
ros2 run robot_bringup plot_joint_velocities.py
```
Shows actual vs commanded velocities for all 4 wheels with tracking error statistics.

**Robot base velocity tracking:**
```bash
ros2 run robot_bringup plot_robot_velocities.py
```
Shows actual vs commanded linear and angular velocities from odometry.

### Checking System Status

**List active controllers:**
```bash
ros2 control list_controllers
```

**Check controller status:**
```bash
ros2 control list_hardware_interfaces
```

**Monitor joint states:**
```bash
ros2 topic echo /joint_states
```

**Monitor odometry:**
```bash
ros2 topic echo /diff_drive_base_controller/odom
```
**Monitor ground truth (simulation only):**
```bash
ros2 topic echo /ground_truth_pose
```
### Nav2 Status

**Check navigation status:**
```bash
ros2 topic echo /navigate_to_pose/_action/status
```

**View costmaps:**
```bash
ros2 topic echo /local_costmap/costmap
ros2 topic echo /global_costmap/costmap
```

**Monitor planned path:**
```bash
ros2 topic echo /plan
```

## Controller Configuration

### Differential Drive Parameters

Edit `config/diff_drive_controller.yaml`:

```yaml
diff_drive_base_controller:
  ros__parameters:
    # Wheel configuration
    left_wheel_names: ["front_left_wheel_joint", "rear_left_wheel_joint"]
    right_wheel_names: ["front_right_wheel_joint", "rear_right_wheel_joint"]
    
    # Physical dimensions
    wheel_separation: 0.609  # meters (distance between left and right wheels)
    wheel_radius: 0.102      # meters
    
    # Calibration multipliers
    wheel_separation_multiplier: 2.40  # Adjust for slip/skid
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0
    
    # Velocity limits (matching motor capabilities)
    linear.x.max_velocity: 0.88    # m/s (100% motor speed)
    angular.z.max_velocity: 1.2    # rad/s
```

### Update Rate

```yaml
controller_manager:
  ros__parameters:
    update_rate: 10  # Hz - frequency of read/write cycles
```

**Note:** Higher update rates provide smoother control but is limited by the Phidgets Hubs

## Hardware Interface

### Implementation

The hardware interface (`hardware/diffbot_system.cpp`) provides:

**Lifecycle States:**
- `on_configure()` - Initialize Phidget motor controllers
- `on_activate()` - Enable motors and reset positions
- `on_deactivate()` - Stop motors and cleanup

**Control Loop:**
- `read()` - Read encoder positions, calculate velocities
- `write()` - Send velocity commands to motors

## Troubleshooting

**TODO**

## Published Topics

**Core Topics:**
- `/joint_states` (sensor_msgs/msg/JointState) - Joint positions and velocities
- `/diff_drive_base_controller/odom` (nav_msgs/msg/Odometry) - Robot odometry
- `/tf` (tf2_msgs/msg/TFMessage) - Transform tree
- `/ground_truth_pose` (geometry_msgs/msg/PoseStamped) - Ground truth from Gazebo (sim only)

**Navigation Topics:**
- `/map` (nav_msgs/msg/OccupancyGrid) - Global map
- `/local_costmap/costmap` (nav_msgs/msg/OccupancyGrid) - Local costmap
- `/global_costmap/costmap` (nav_msgs/msg/OccupancyGrid) - Global costmap
- `/plan` (nav_msgs/msg/Path) - Planned path

**SLAM Topics:**
- `/map` (nav_msgs/msg/OccupancyGrid) - SLAM-generated map
- `/slam_toolbox/graph_visualization` (visualization_msgs/msg/MarkerArray) - Pose graph

### Subscribed Topics

**Control Topics:**
- `/diff_drive_base_controller/cmd_vel` (geometry_msgs/msg/TwistStamped) - Velocity commands
- `/cmd_vel_gamepad_stamped` (geometry_msgs/msg/TwistStamped) - Gamepad input (via twist_mux)
- `/cmd_vel_nav_stamped` (geometry_msgs/msg/TwistStamped) - Navigation input (via twist_mux)

**Sensor Topics:**
- `/scan` (sensor_msgs/msg/LaserScan) - LiDAR data

### Services

- `/controller_manager/*` - Controller lifecycle management
- `/diff_drive_base_controller/*` - Controller-specific services
- `/slam_toolbox/*` - SLAM services (save map, etc.)
- Navigation action servers (navigate_to_pose, etc.)

## Package Structure

```
robot_bringup/
├── config/
│   ├── diff_drive_controller.yaml       # Hardware controller config
│   ├── sim_diff_drive_controller.yaml   # Simulation controller config
│   ├── slam_params.yaml                 # SLAM Toolbox configuration
│   ├── nav2/                            # Nav2 configuration files
│   │   ├── amcl.yaml
│   │   ├── behavior_server.yaml
│   │   ├── bt_navigator.yaml
│   │   ├── controller_server.yaml
│   │   ├── global_costmap.yaml
│   │   ├── local_costmap.yaml
│   │   ├── map_server.yaml
│   │   └── planner_server.yaml
│   └── twist_mux/
│       └── twist_mux_topics.yaml        # Twist mux topic configuration
├── hardware/
│   ├── include/
│   │   └── diffbot_system.hpp           # Hardware interface header
│   └── diffbot_system.cpp               # Hardware interface implementation
├── launch/
│   ├── ibex.launch.py                   # Hardware launch file
│   ├── ibex_sim.launch.py               # Simulation + Nav2 launch file
│   └── ibex_sim_slam.launch.py          # Simulation + SLAM launch file
├── maps/
│   ├── cave_world/
│   │   ├── map.pgm
│   │   └── map.yaml
│   └── small_house/
│       ├── map.pgm
│       └── map.yaml
├── scripts/
│   ├── gt_robot_pose.py                 # Ground truth pose extractor
│   ├── plot_joint_velocities.py         # Joint velocity visualization
│   └── plot_robot_velocities.py         # Robot velocity visualization
├── ibex_control.xml                     # Plugin description
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Integration with Other Packages

### Required Packages

**Build-time:**
- `phidgets_test` - Motor driver headers
- `robot_description` - URDF for controller configuration

**Runtime:**
- `robot_teleop` - Joystick control (optional)
- `slam_toolbox` - SLAM mapping (for SLAM launch)
- `nav2_*` - Navigation stack (for simulation launch)
- `twist_mux` - Command multiplexing
