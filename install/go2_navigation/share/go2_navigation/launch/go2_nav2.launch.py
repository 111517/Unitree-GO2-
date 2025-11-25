import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo  # 新增LogInfo用于调试
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 新增路径拼接工具
from launch_ros.actions import Node


def generate_launch_description():
    go2_navigation_dir = get_package_share_directory('go2_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    map_yaml_path = LaunchConfiguration(
        'map', 
        default=os.path.join(go2_navigation_dir, 'maps', 'lab_map_2.yaml')
    )
    nav2_param_path = LaunchConfiguration(
        'params_file', 
        default=os.path.join(go2_navigation_dir, 'config', 'nav2_params.yaml')
    )
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 新增：打印参数文件路径（调试用，确认加载的是正确文件）
    log_param_path = LogInfo(msg=[
        "加载的Nav2参数文件路径: ", 
        nav2_param_path
    ])

    # 包含Nav2官方bringup_launch.py（已内置代价地图节点）
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])
        ]),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items(),
    )

    # RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 启动描述：添加调试日志
    return LaunchDescription([
        log_param_path,  # 先打印参数路径
        nav2_bringup_launch,
        rviz_node
    ])