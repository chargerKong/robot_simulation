import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
# from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    # pkg_dir = get_package_share_directory('robot_description')
    rviz_config_file= os.path.join(get_package_share_directory('my_slam'), 'rviz2', 'carto_slam.rviz')

   

    lua_configuration_directory = os.path.join(
        get_package_share_directory('my_slam'), 'lua')
    lua_configuration_name = 'my_slam_indoor_2d.lua'

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        arguments=['-configuration_directory', lua_configuration_directory,
                   '-configuration_basename', lua_configuration_name ],
        remappings = [('scan', 'scan')],
        output='screen',
    )

    cartographer_map = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        arguments=['-resolution', '0.05'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        on_exit=actions.Shutdown()
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/kong/bag/gazebo_scan_tf_odom.bag/'],
            output='screen',
        ),
        cartographer,
        cartographer_map,
        rviz,
    ])
