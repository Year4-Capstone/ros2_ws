from launch import LaunchDescription
from launch.actions import RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- Package Name ---
    # Ensure this matches your actual package name (e.g., 'robot_description' or 'robot_bringup')
    pkg_robot_description = FindPackageShare('robot_description')
    pkg_robot_bringup = FindPackageShare('robot_bringup')
    # --- Paths ---

    def nav2_cfg(name):
        return PathJoinSubstitution([
            pkg_robot_bringup,
            'config',
            'nav2',
            name
        ])
    
    # Path to the controller configuration YAML file
    controller_config_file = PathJoinSubstitution(
        [pkg_robot_bringup, 'config', 'diff_drive_controller.yaml']
    )
    
    xacro_file = PathJoinSubstitution(
        [pkg_robot_description, 'urdf', 'ibex.urdf.xacro']
    )

    rviz_config_path = PathJoinSubstitution(
        [pkg_robot_description, 'rviz', 'ibex_nav2.rviz']
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

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='lukes_house'
    )

    map_name   = LaunchConfiguration('map_name')

    map_yaml = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'maps',
        map_name,
        'map.yaml'
    ])

    nav2_launch = GroupAction(actions=[
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_cfg('map_server.yaml'),
                {'yaml_filename': map_yaml},
            ],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_cfg('amcl.yaml')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                nav2_cfg('planner_server.yaml'),
                nav2_cfg('global_costmap.yaml'),
            ],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                nav2_cfg('controller_server.yaml'),
                nav2_cfg('local_costmap.yaml'),
            ],
            remappings=[
                ('cmd_vel', '/diff_drive_base_controller/cmd_vel')
            ],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_cfg('behavior_server.yaml')],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_cfg('bt_navigator.yaml')],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ),

        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False,
        #         'autostart': True,
        #         'node_names': [
        #             'map_server',
        #             'amcl',
        #         ],
        #     }],
        # ),
    ])

    return LaunchDescription([
        map_name_arg,
        node_robot_state_publisher,
        controller_manager_node,
        rviz_node,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        nav2_launch
    ])