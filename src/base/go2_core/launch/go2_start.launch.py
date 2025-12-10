from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction 
def generate_launch_description():
    ld = LaunchDescription()
    #获取各功能包
    go2_driver_pkg = get_package_share_directory("go2_driver")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_slam_pkg = get_package_share_directory("go2_slam")
    go2_perception_pkg = get_package_share_directory("go2_perception")
    go2_navigation_pkg= get_package_share_directory("go2_navigation")
    go2_cartographer_pkg= get_package_share_directory("go2_cartographer")
    go2_application_pkg= get_package_share_directory("go2_application")
    go2_camera_pkg= get_package_share_directory("go2_camera")
    # 添加启动开关
    use_slamtoolbox = DeclareLaunchArgument(
        name="use_slamtoolbox",
        default_value="true"
    )

    # 里程计融合imu
    go2_robot_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_core_pkg, "launch", "go2_robot_localization.launch.py")
            )
        )

    # 启动驱动包
    go2_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_driver_pkg, "launch", "driver.launch.py")
        )   
    )

    # 点云处理
    go2_pointcloud_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_perception_pkg, "launch", "go2_pointcloud.launch.py")
            )
        )

    # slam-toolbox 配置
    go2_slamtoolbox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_slam_pkg, "launch", "go2_slamtoolbox.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration('use_slamtoolbox'))
        )
    
    # nav2 配置
    go2_navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_navigation_pkg, "launch", "go2_nav2.launch.py")
            )
        )
    
    go2_cartographer_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_cartographer_pkg, "launch", "go2_cartographer.launch.py")
            )
        )
    
    go2_application_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_application_pkg, "launch", "go2_application.launch.py")
            )
        )
    go2_camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_camera_pkg, "launch", "go2_camera.launch.py")
            )
        )

    # 包含rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(go2_core_pkg, "rviz2", "display.rviz")],
        output='screen'
    )
    #ld.add_action(use_slamtoolbox)
    ld.add_action(go2_driver_launch)
    ld.add_action(go2_robot_localization) 
    ld.add_action(go2_pointcloud_launch)
    #ld.add_action(TimerAction(period=5.0,actions=[go2_cartographer_launch]))
    #ld.add_action(TimerAction(period=2.0,actions=[go2_slamtoolbox_launch]))
    ld.add_action(TimerAction(period=20.0,actions=[go2_navigation_launch]))
    #ld.add_action(TimerAction(period=2.0,actions=[rviz_node]))
    #ld.add_action(TimerAction(period=1.0,actions=[go2_camera_launch]))
    ld.add_action(TimerAction(period=21.0,actions=[go2_application_launch]))

    return ld