import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackagePrefix
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

from launch.actions import OpaqueFunction
from launch.substitutions import PythonExpression

from launch.actions import ExecuteProcess
from launch.substitutions import PythonExpression
from launch.substitutions import ThisLaunchFileDir



def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    # "warehouse.world".

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-entity', 'demo', "-topic", "robot_description"],
                      output='screen')
    #
    robot_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                    {'use_sim_time': use_sim_time},
                    {'robot_description': Command([
                        PathJoinSubstitution([FindPackagePrefix('xacro'), "bin", "xacro"]),
                        ' ',
                        PathJoinSubstitution(
                            [get_package_share_directory('robot_description'), 'my_robot.xacro']),
                    ])
                    }],
    )
    # gazebo launch
    return LaunchDescription([
        gazebo,
        robot_pub,
        spawn_entity,
    ])
