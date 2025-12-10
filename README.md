感谢你找到这个开源仓库，并有可能认真看我完成的代码。
本项目所需硬件：只需宇树机械狗GO2 EDU版本
环境依赖
        操作系统：Ubuntu 22.04
        ROS2 版本：Humble
需自己根据宇树官方的教程配置好环境，连接好主机和狗子即可
本仓库存储了针对宇树 GO2 机械狗的二次开发源码
实现了 
      src/base/go2_core            --融合所有功能，在go2_start.launch.py里面调用所有功能
      src/go2_description          --导入官方机械狗的URDF，发布静态链接TF，在Rviz2中可视化机械狗
      src/base/go2_driver          --官方没有发布标准的Odom和IMU消息，因此也是实现了发布Odom和IMU话题以及
      src/base/go2_twist_bridge    --速度消息桥接Twist，可用于后面的建图和导航
      src/go2_perception           --3D点云数据转为2D点云
      src/base/go2_core            --多传感器融合（EKF）,输入是Odom和IMU话题，输出是"/odometry/filtered"话题，用于后边的建图
      src/go2_slam                 --Slam_ToolBox建图
      src/go2_cartographer         --Cartographer建图
      src/go2_navigation           --Navigation2+Rviz2手动导航
      src/go2_application          --Python调用Nav2的API进行自主导航
      src/unitree_sdk2_python      --DDS获取机械狗前置摄像头图像
      src/go2_camera               --使用YOLOV8进行图像识别

使用教程(逐行运行)
cd unitree_go2_ws/
colcon build
source install/setup.bash
ros2 launch go2_core go2_start.launch.py

后面会对该项目持续不定期更新，欢迎联系邮箱2510263036@mails.szu.edu.cn
