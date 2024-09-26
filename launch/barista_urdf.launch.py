import os
import random
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch.substitutions import Command
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_description = "barista_robot_description"
    install_dir = get_package_prefix(package_description)
    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    gazebo_models_path = os.path.join(package_description, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ[
                                              'GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH==" + str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH==" + str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Get the launch directory
    #launch_dir = os.path.join(get_package_share_directory('barista_robot_description'), 'launch')
    gazebo_argument = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(package_description, 'worlds', 'barista_bot_empty.world'), ''],
        description='SDF world file')

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
        arguments=['-entity', 'barista_robot', '-topic', '/robot_description'],
        output='screen'
    )

    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file_name)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        emulate_tty=True,
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'barista_robot.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz2_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz,
        gazebo_argument,
        gazebo,
        spawn_entity
    ])