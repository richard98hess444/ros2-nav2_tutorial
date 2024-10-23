import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'nav2_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsuhanjaya',
    maintainer_email='ahanjaya@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amclPoseSubscriber = nav2_tutorial.amcl_pose_sub:main',
            'tf2Echo = nav2_tutorial.tf2_echo:main',
            'timestamp_corrector = nav2_tutorial.timestamp_corrector:main',
            'cmd_vel_sub = nav2_tutorial.cmd_vel_sub:main',
            'initial_pose_publisher = nav2_tutorial.initial_pose_pub:main'
        ],
    },
)
