import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    declare_laser_scan = DeclareLaunchArgument(
        'name_1',
        default_value='lidar_1')

    config = os.path.join(get_package_share_directory('neo_sick_s300-2'),'launch','s300_1.yaml')
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='neo_sick_s300-2', executable='neo_sick_s300_node', output='screen',
            name='lidar_1', parameters = [config])
    ,
     launch_ros.actions.Node(
            package='neo_sick_s300-2', executable='neo_scan_filter_node', output='screen',
            name='lidar_1')
    ])