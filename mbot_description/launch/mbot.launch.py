import os
from launch.substitutions import LaunchConfiguration
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = "mbot_base.urdf.xml"
    #aurdf_file_name = "r2d2.urdf.xml"
    
    urdf = os.path.join(
            get_package_share_directory('mbot_description'),
            urdf_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
        Node(
            package='rviz2', 
            executable='rviz2', 
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]),
])
