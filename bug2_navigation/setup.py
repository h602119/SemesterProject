from setuptools import setup
import os
from glob import glob

package_name = 'bug2_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.xacro'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocotics',
    maintainer_email='rocotics@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = bug2_navigation.wall_follower:main',
            'go_to_point = bug2_navigation.go_to_point:main',
            'bug2_controller = bug2_navigation.bug2_controller:main',
            'robot_controller = bug2_navigation.robot_controller:main',
            'test = bug2_navigation.test:main',
            'con_test = bug2_navigation.con_test:main',
            'goTest = bug2_navigation.goTest:main',
            'cTest = bug2_navigation.cTest:main',        
            '9Point = bug2_navigation.9point:main',
            'MapTest = bug2_navigation.map:main',         
        ],
    },
)
