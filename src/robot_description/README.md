[« Back to ros2_ws README](../../README.md)
# robot_description

## Overview

URDF description package for the Ibex robot platform. Provides robot visualization in RViz and Gazebo simulation with support for ros2_control integration.

## Hardware

### Robot Platform
- **Name:** Ibex
- **Type:** 4-wheel differential drive with fixed flipper tracks
- **Sensors:** IMU, 2D LiDAR

### Coordinate Frames
- `base_footprint` - Ground projection
- `base_link` - Main chassis
- `{front/rear}_{left/right}_wheel_link` - Four wheels
- `{front/rear}_{left/right}_flipper` - Four flipper tracks
- `imu_link` - IMU sensor (0.45, -0.30, 0.07)
- `laser` - LiDAR sensor (0.45, 0, 0.12)

## Dependencies

### ROS2 Dependencies
- `robot_state_publisher` - Publishes robot state to TF
- `joint_state_publisher` - Publishes joint states
- `joint_state_publisher_gui` - GUI for manual joint control
- `rviz2` - Robot visualization
- `xacro` - URDF macro processing
- `ros_gz_sim` - Gazebo Sim integration
- `ros_gz_bridge` - ROS2-Gazebo message bridge

### Install Dependencies

**Using rosdep (recommended):**
```bash
cd ~/ros2_ws
rosdep install --from-paths src/robot_description --ignore-src -r -y
```

**Manual installation:**
```bash
sudo apt install ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-rviz2 \
                 ros-jazzy-xacro \
                 ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge
```

## Usage

### Building the Package

```bash
cd ~/ros2_ws
colcon build --packages-select robot_description
source install/setup.bash
```

### Visualizing the Robot

**Basic visualization:**
```bash
ros2 launch robot_description view_robot_rviz.launch.py
```

This opens:
- **RViz2** - 3D visualization of robot model
- **Joint State Publisher GUI** - Sliders to control wheel positions

### Simulating the Robot in Gazebo

**Launch with default empty world:**
```bash
ros2 launch robot_description view_robot_gazebo.launch.py
```

**Launch with specific world:**
```bash
# Cave world
ros2 launch robot_description view_robot_gazebo.launch.py world:=cave_world.world

# Small house world
ros2 launch robot_description view_robot_gazebo.launch.py world:=small_house.world
```

The Gazebo launch includes:
- Gazebo Sim with specified world
- Robot spawned at origin (0, 0, 0.5)
- ROS2-Gazebo bridges for `/clock`, `/imu`, and `/scan` topics

## Gazebo Model Setup

The package includes models in the `models/` directory:
- `aws/` - AWS RoboMaker models (small house world dependencies)
- `cave_world/` - Cave environment models

The launch file automatically configures `GZ_SIM_RESOURCE_PATH` to include:
- Package models directory
- AWS models subdirectory
- URDF meshes

## Customization

### Adjusting Sensor Positions

**IMU position:**
```xml
<joint name="imu_joint" type="fixed">
    <origin xyz="0.45 -0.30 0.07" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="imu_link" />
</joint>
```

**LiDAR position:**
```xml
<joint name="laser_joint" type="fixed">
    <origin xyz="0.45 0 0.12" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="laser" />
</joint>
```

### Changing Spawn Position

Edit the spawn arguments in `view_robot_gazebo.launch.py`:
```python
spawn_node = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'ibex_robot',
        '-topic', 'robot_description',
        '-x', '0', '-y', '0', '-z', '0.5'  # Modify x, y, z here
    ],
    ...
)
```

## Troubleshooting

### Robot Not Displaying in RViz

1. **Verify launch file started correctly:**
   ```bash
   ros2 launch robot_description view_robot_rviz.launch.py
   ```

2. **Check RViz configuration:**
   - Fixed Frame should be `base_footprint`
   - RobotModel display is enabled
   - No red errors in Displays panel

### Missing Meshes

If error messages mention missing mesh files:

1. **Check mesh paths use correct format:**
   ```xml
   package://robot_description/meshes/filename.stl
   ```

2. **Rebuild package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_description --cmake-clean-cache
   source install/setup.bash
   ```

### TF Transform Errors

1. **Check robot_state_publisher running:**
   ```bash
   ros2 node list | grep robot_state_publisher
   ```

2. **Verify joint states publishing:**
   ```bash
   ros2 topic echo /joint_states
   ```

3. **View TF tree:**
   ```bash
   ros2 run tf2_tools view_frames
   # Opens frames.pdf showing transform tree
   ```

### URDF Processing Errors

1. **Check xacro syntax**
2. **Verify included files exist:**
   - `ibex_core.urdf.xacro`
   - `ibex_sim_ros2_control.urdf.xacro` (for simulation)
   - `ibex_hardware_ros2_control.urdf.xacro` (for hardware)

## ros2_control Integration

### Simulation Mode vs Hardware Mode

The URDF uses a `sim_mode` argument to switch between simulation and hardware. This is applied in the launch files.

### Hardware Interface

Uses custom hardware plugin:
```xml
<plugin>ibex_control/DiffBotSystemHardware</plugin>
```

### Motor Controller Configuration

**Phidget Hub 1 (Serial: 527103):**
- Port 2: Back Left wheel
- Port 3: Back Right wheel

**Phidget Hub 2 (Serial: 527164):**
- Port 2: Front Left wheel
- Port 3: Front Right wheel

### Control Interfaces

**Command Interfaces:**
- Velocity control for each wheel (duty cycle: -1 to 1)

**State Interfaces:**
- Position (radians)
- Velocity (rad/s)

## URDF Structure

### File Organization

**ibex.urdf.xacro** - Main entry point
- Includes `ibex_core.urdf.xacro`
- Includes `ibex_ros2_control.urdf.xacro`

**ibex_core.urdf.xacro** - Robot description
- Link and joint definitions
- Macros for wheels and flippers
- Mesh files and materials
- Inertial properties

**ibex_gazebo.urdf.xacro** - Gazebo-specific configuration
- Sensor plugins (IMU, LiDAR)
- Material properties for simulation
- Collision properties

**ibex_sim_ros2_control.urdf.xacro** - Simulation control interface
- Gazebo ros2_control plugin
- Simulated joint interfaces

**ibex_hardware_ros2_control.urdf.xacro** - Hardware control interface
- Physical hardware plugin configuration
- Phidget motor controller parameters
- Command/state interfaces

## Package Structure

```python
robot_description/
├── launch/
│   ├── view_robot_rviz.launch.py      # RViz visualization
│   └── view_robot_gazebo.launch.py    # Gazebo simulation
├── meshes/
│   ├── base_link.stl                  # Main chassis mesh
│   ├── wheel_*.stl                    # Wheel meshes (4 files)
│   └── flipper_*.stl                  # Flipper meshes (4 files)
├── models/
│   ├── aws/                           # AWS RoboMaker models
│   └── cave_world/                    # Cave environment models
├── rviz/
│   ├── ibex.rviz                      # Basic RViz config
│   ├── ibex_visualize.rviz            # Visualization config
│   └── ibex_nav2.rviz                 # Navigation config
├── urdf/
│   ├── ibex.urdf.xacro                # Main URDF (includes other files)
│   ├── ibex_core.urdf.xacro           # Robot geometry and properties
│   ├── ibex_gazebo.urdf.xacro         # Gazebo sensor plugins
│   ├── ibex_sim_ros2_control.urdf.xacro      # Simulation control
│   └── ibex_hardware_ros2_control.urdf.xacro # Hardware control
├── worlds/
│   ├── empty_world.sdf                # Empty test world
│   ├── small_house.world              # AWS small house
│   ├── cave_world.world               # Cave environment
│   └── small_warehouse.world          # Warehouse environment
├── CMakeLists.txt
├── package.xml
└── README.md
```

## External Model Resources
**AWS RoboMaker worlds:**

https://github.com/aws-robotics/aws-robomaker-small-house-world

**Cave world:**

https://github.com/LTU-RAI/gazebo_cave_world
