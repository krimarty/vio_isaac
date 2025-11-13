# example launch file.
#
# Created by Milos Cihlar on 13.10.2025

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
from typing import Optional, List


def launch_setup(context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:

    #example_dir_config = os.path.join(get_package_share_directory('example_package'), 'config')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    robot_number = LaunchConfiguration('robot_number')
    config = LaunchConfiguration('config')
    #odometry = LaunchConfiguration('odometry')
    image = LaunchConfiguration('image')
    #cmd_vel = LaunchConfiguration('cmd_vel')

    indexed_robot_name = [robot_name.perform(context), '_', robot_number.perform(context)] if robot_number.perform(context) else [robot_name.perform(context)]
    indexed_robot_name = ''.join(indexed_robot_name)

    config_substitutions = { 
            #"odom_frame": indexed_robot_name + "_odom",
            "isaac_vio_frame": indexed_robot_name + "_isaac_vio",
    }

    #config_dir = os.path.join(example_dir_config, robot_name.perform(context))
    #os.chdir(config_dir)

    visual_slam_node = ComposableNode(
        package = 'isaac_ros_visual_slam',
        name = 'visual_slam_node',
        namespace=indexed_robot_name + '/isaac_vio',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': False,
            'rectified_images': True,
            'enable_imu_fusion': False,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 22.00,
            'base_frame': 'f450_1_base_link',
            'imu_frame': 'camera_gyro_optical_frame',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'camera_optical_frames': [
                'f450_1_realsense_d435_infra1_optical_frame',
                'f450_1_realsense_d435_infra2_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/image_0',
            f'/{indexed_robot_name}/sensors/realsense_d435/realsense_camera_node/infra1/image_rect_raw'),
            
            ('visual_slam/camera_info_0',
            f'/{indexed_robot_name}/sensors/realsense_d435/realsense_camera_node/infra1/camera_info'),
            
            ('visual_slam/image_1',
            f'/{indexed_robot_name}/sensors/realsense_d435/realsense_camera_node/infra2/image_rect_raw'),
            
            ('visual_slam/camera_info_1',
            f'/{indexed_robot_name}/sensors/realsense_d435/realsense_camera_node/infra2/camera_info'),
        ],
    )

    visual_slam_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='visual_slam_launch_container',
        namespace=indexed_robot_name + '/isaac_vio',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )
    return [LaunchDescription([visual_slam_container])]


def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true.'
        ),
        DeclareLaunchArgument(
            name='robot_name',
            description='Robot name (ex: "robot").'
        ),
        DeclareLaunchArgument(
            name='robot_number',
            default_value='',
            description='Robot number, which will be appended to the robot name if defined ("<name>_<number>", '
                        'default is empty).'
        ),
        DeclareLaunchArgument(
            name='config',
            description='Example package configuration.'
        ),
        #DeclareLaunchArgument(
        #    name='cmd_vel',
        #    default_value='cmd_vel',
        #    description='Input twist topic.'
        #),
        DeclareLaunchArgument(
            name='image',
            default_value='image',
            description='Input image topic.'
        ),
        # Perform the launch setup
        OpaqueFunction(function=launch_setup)
    ])
