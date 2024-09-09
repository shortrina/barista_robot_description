import os
import random
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch.substitutions import Command, LaunchConfiguration
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
    # launch_dir = os.path.join(get_package_share_directory('barista_robot_description'), 'launch')
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
    # urdf_file_name = 'barista_robot_gazebo_model.urdf'
    xacro_file_name = 'barista_robot_model.urdf.xacro'
    xacro = os.path.join(
        get_package_share_directory('barista_robot_description'),
        'xacro',
        xacro_file_name)

    robot_desc_path = os.path.join(get_package_share_directory(package_description), 'xacro', xacro_file_name)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_name_1 = "morty"
    robot_name_2 = "rick"

    robot_state_publisher_robot_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=robot_name_1,
        emulate_tty=True,
        output='screen',
        parameters=[{'frame_prefix': robot_name_1+'/', 'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_1])}]
    )

    robot_state_publisher_robot_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=robot_name_2,
        emulate_tty=True,
        output='screen',
        parameters=[{'frame_prefix': robot_name_2+'/', 'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_2])}]
    )

    # Static transform world to odom
    static_tf_pub_robot_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="".join(['static_transform_publisher_', robot_name_1, '_odom']),
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', robot_name_1 + '/odom']  # parent, child
    )

    static_tf_pub_robot_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="".join(['static_transform_publisher_', robot_name_2, '_odom']),
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', robot_name_2 + '/odom']  # parent, child
    )

    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'barista_two_robots.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz2_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_dir]
    )

    entity_name_1 = robot_name_1 + "-" + str(int(random.random() * 100000))
    entity_name_2 = robot_name_2 + "-" + str(int(random.random() * 100000))

    # Spawn the robot in Gazebo
    spawn_robot_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name_1,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-topic', robot_name_1+'/robot_description'
                   ]
    )

    spawn_robot_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name_2,
                   '-x', '1.5',
                   '-y', '1.5',
                   '-z', '0.0',
                   '-topic', robot_name_2+'/robot_description'
                   ]
    )

    return LaunchDescription([
        robot_state_publisher_robot_1,
        robot_state_publisher_robot_2,
        static_tf_pub_robot_1,
        static_tf_pub_robot_2,
        rviz,
        gazebo_argument,
        gazebo,
        spawn_robot_1,
        spawn_robot_2
    ])
