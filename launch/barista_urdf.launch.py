import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('barista_robot_description'), 'launch')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # URDF file path
    urdf_file_name = 'barista_robot_model.urdf'
    urdf = os.path.join(
        get_package_share_directory('barista_robot_description'),
        'urdf',
        urdf_file_name)

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'barista_robot', '-file', urdf],
        output='screen'
    )

    # RViz
    rviz_config_dir = os.path.join(
        get_package_share_directory('barista_robot_description'),
        'rviz',
        'barista_robot.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        arguments=[urdf]
    )

    return LaunchDescription([        
        robot_state_publisher_node,
        rviz
    #    gazebo,
    #    spawn_entity
    ])
