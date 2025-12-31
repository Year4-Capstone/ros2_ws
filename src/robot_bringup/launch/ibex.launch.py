from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Package Name ---
    # Ensure this matches your actual package name (e.g., 'robot_description' or 'robot_bringup')
    pkg_robot_description = FindPackageShare('robot_description')
    pkg_robot_bringup = FindPackageShare('robot_bringup')
    # --- Paths ---
    
    # Path to the controller configuration YAML file
    controller_config_file = PathJoinSubstitution(
        [pkg_robot_bringup, 'config', 'diff_drive_controller.yaml']
    )
    
    xacro_file = PathJoinSubstitution(
        [pkg_robot_description, 'urdf', 'ibex.urdf.xacro']
    )

    rviz_config_path = PathJoinSubstitution(
        [pkg_robot_description, 'rviz', 'ibex.rviz']
    )

    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file, ' sim_mode:=false']
    )
    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    node_urg_lidar = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'serial_baud': 115200,
            'frame_id': 'laser',
            'angle_min': -1.570796,
            'angle_max': 1.570796,
            'use_sim_time': False
        }]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config_file],
        output="screen",
    )

    # 2. Launch Rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # --- ROS 2 Control Spawners ---
    # Launched immediately without event handlers. 
    # They might error initially while waiting for Gazebo to start, but will retry.

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[diff_drive_base_controller_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_urg_lidar,
        controller_manager_node,
        rviz_node,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
    ])