import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackagePrefix
from launch.actions import DeclareLaunchArgument

from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    # pkg_my_slam = get_package_share_directory("my_slam")
    # "warehouse.world".
    pkg_dir = get_package_share_directory('robot_description')
    rviz_config_dir = os.path.join(get_package_share_directory('my_slam'), 'rviz2', 'icp_slam.rviz')
    # os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, "model")
    #os.environ["GAZEBO_MODEL_PATH"] = "/home/kong/dev_ws/src/robot_description/model"
    # world = os.path.join(pkg_dir, 'warehouse.world')
    # world = os.path.join(pkg_dir, 'turtlebot3.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'gazebo_lab_world.launch.py')
        ),
    )

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                   arguments=['-entity', 'demo', "-topic", "robot_description"],
    #                   output='screen')
    #
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config_dir],
        output='screen',
    )

    teleop_key = Node(
        package='teleop_key_control',
        executable='teleop_key_control',
        name='teleop_key_control',
        output='screen',
    )

    icp = Node(
        package='my_slam',
        executable='icp',
        name='icp',
        output='screen',
    )
    # robot_pub = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[
    #                 {'use_sim_time': use_sim_time},
    #                 {'robot_description': Command([
    #                     PathJoinSubstitution([FindPackagePrefix('xacro'), "bin", "xacro"]),
    #                     ' ',
    #                  PathJoinSubstitution(
    #                         [get_package_share_directory('robot_description'), 'my_robot.xacro']),
    #                 ])
    #                 }],
    # )
    # gazebo launch
    return LaunchDescription([

        gazebo,
        rviz2,
        # teleop_key,
        # icp
    ])
