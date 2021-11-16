import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node                                                             
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    # "example1"
    urdf_file = "/home/kong/dev_ws/src/robot_description/urdf/example1.urdf"
    # urdf_file = os.path.join(get_package_share_directory('robot_description'), 'example1.urdf')
    # urdf_file = os.path.join(get_package_share_directory('robot_description'), 'example2.urdf')
    # urdf_file = os.path.join(get_package_share_directory('robot_description'), 'example3.urdf')
    # urdf_file = os.path.join(get_package_share_directory('robot_description'), 'example4.urdf')
    os.environ["GAZEBO_MODEL_PATH"] = "/home/kong"
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),  
    )   

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-entity', 'demo', "-file", urdf_file],
                      output='screen')
                          
    return LaunchDescription([
        DeclareLaunchArgument('pause', default_value='true',
                             description="pause the world"),
        gazebo,
        spawn_entity,
    ])  

