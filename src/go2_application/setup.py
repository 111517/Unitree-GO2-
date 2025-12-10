from setuptools import find_packages, setup
from glob import glob
package_name = 'go2_application'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", glob("launch/*.launch.py")),
        ('share/' + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayi',
    maintainer_email='ayi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "init_go2_pose = go2_application.init_go2_pose:main",
            "go2_partol = go2_application.go2_partol:main",
            "waypoint = go2_application.waypoint_flollower:main",
        ],
    },
)
