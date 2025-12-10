from setuptools import find_packages, setup
from glob import glob
package_name = 'go2_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'unitree-sdk2py',  # 声明需要安装 SDK
    ],
    zip_safe=True,
    maintainer='ayi',
    maintainer_email='1322164126@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "camera_opencv=go2_camera.camera_opencv:main",
            "go2_video=go2_camera.go2_video:main"
        ],
    },
)
