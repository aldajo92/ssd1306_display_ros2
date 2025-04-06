from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file in the share directory
    config_file = os.path.join(
        get_package_share_directory('ssd1306_display'),
        'config',
        'display_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='ssd1306_display',
            executable='display_node',
            name='oled_display_node',
            parameters=[config_file]
        )
    ])
