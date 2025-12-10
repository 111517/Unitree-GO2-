from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # 获取包路径
    #get_packages = get_package_share_directory('go2_camera')
    #config_file = os.path.join(get_packages, 'config', 'partol_config.yaml')

    go2_camera = Node(
        package="go2_camera",
        executable="camera_opencv",
    )
    return LaunchDescription([
      go2_camera
    ])