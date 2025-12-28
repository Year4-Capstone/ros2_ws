import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_robot_sim_description = get_package_share_directory('robot_sim_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # world argument    
    world_name_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.sdf',
        description='World to load: small_house.world, cave_world.world, or empty_world.sdf'
    )
    world_name = LaunchConfiguration('world')

    # Construct the path to the world file
    world_path = PathJoinSubstitution([
        pkg_robot_sim_description,
        'worlds',
        world_name
    ])

    # Gazebo path setup
    models_path = os.path.join(pkg_robot_sim_description, 'models') # cave_world
    aws_models_path = os.path.join(models_path, 'aws') # small_house
    robot_sim_description_parent_path = os.path.dirname(pkg_robot_sim_description) # URDF meshes

    default_paths = [
        os.path.expanduser("~/.gz/models"),
        "/usr/share/gz/models",
    ]
    combined_resource_path = ":".join(
        default_paths + [
            robot_sim_description_parent_path, 
            models_path, 
            aws_models_path])

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_resource_path
    )

    # URDF
    urdf_path = os.path.join(get_package_share_path('robot_sim_description'), 'urdf', 'cad_urdf.urdf.xacro')

    robot_description = {
        'robot_description': Command([
            'xacro ', urdf_path, ' use_gazebo:=true'
        ])
    }

    # Launch Gazebo with the specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_path]
        }.items()
    )

    # Nodes

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ibex_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_name_arg,
        set_gz_resource_path, 
        gazebo,
        robot_state_publisher_node,
        spawn_node,
        bridge_node,
    ])
