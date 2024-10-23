import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource

use_sim_time = LaunchConfiguration('use_sim_time', default='false')

slam_toolbox_config = os.path.join(
        get_package_share_directory('nav2_tutorial'),
        'config',
        'slam_params.yaml'
    )

robot_localization_config = os.path.join(
        get_package_share_directory('nav2_tutorial'),
        'config',
        'ekf.yaml'
    )

def generate_launch_description():
    
    lidar_group = GroupAction([

        # Livox Mid 360
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'rviz_MID360_launch.py')
            ]),
        ),

        # point cloud to laser scan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'livox_frame',
                'max_height': 0.5,
                'use_sim_time': use_sim_time
            }],
            output='screen',
        ),

        # odom to base_link
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='fake_static_tf_pub_odom_to_base_link',
        #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link'],
        #     parameters=[{'use_sim_time': use_sim_time}],
        # ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=['/home/hsuhanjaya/Richard/ros2_ws/src/nav2_tutorial/config/ekf.yaml']
        # ),

        # base_link to lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_base_link_to_livox_frame',
            arguments=['0.2', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'livox_frame'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # base_link to base_footprint (needed for slam)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_base_link_to_base_footprint',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'base_footprint'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 
                             'launch', 
                             'navigation_launch.py') # navigation_launch
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        # slam_toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'params_file': slam_toolbox_config,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
    
    
    ld = LaunchDescription()
    ld.add_action(lidar_group)
    return ld