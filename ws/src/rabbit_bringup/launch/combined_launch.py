from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    
    rabbit_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rabbit_bringup'), 'launch', 'localization_launch.py')
        ])
    )
    
    navigate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rabbit_bringup'), 'launch', 'navigate_launch.py')
        ])
    )
    
    ld.add_action(rabbit_bringup_launch)
    ld.add_action(navigate_launch)
    
    return ld

