#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for all parameters
        DeclareLaunchArgument(
            'hud',
            default_value='True',
            description='Display the HUD overlay'
        ),
        DeclareLaunchArgument(
            'crosshair',
            default_value='True',
            description='Display a crosshair in the image'
        ),
        DeclareLaunchArgument(
            'show_temp',
            default_value='True',
            description='Display the temperature text'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='Camera device number'
        ),
        DeclareLaunchArgument(
            'scale',
            default_value='3',
            description='Image scale factor'
        ),
        DeclareLaunchArgument(
            'alpha',
            default_value='1.0',
            description='Contrast adjustment factor'
        ),
        DeclareLaunchArgument(
            'colormap',
            default_value='0',
            description='OpenCV colormap index'
        ),
        DeclareLaunchArgument(
            'blur_radius',
            default_value='0',
            description='Blur radius to apply'
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='2',
            description='Temperature threshold in Celsius'
        ),
        DeclareLaunchArgument(
            'orientation',
            default_value='0',
            description='Image orientation (allowed values: 0, 90, 180, 270)'
        ),

        # Node configuration with parameters set via LaunchConfiguration
        Node(
            package='thermal_camera',
            executable='thermal_camera_node',
            name='thermal_camera_node',
            output='screen',
            parameters=[{
                'hud': LaunchConfiguration('hud'),
                'crosshair': LaunchConfiguration('crosshair'),
                'show_temp': LaunchConfiguration('show_temp'),
                'device': LaunchConfiguration('device'),
                'scale': LaunchConfiguration('scale'),
                'alpha': LaunchConfiguration('alpha'),
                'colormap': LaunchConfiguration('colormap'),
                'blur_radius': LaunchConfiguration('blur_radius'),
                'threshold': LaunchConfiguration('threshold'),
                'orientation': LaunchConfiguration('orientation'),
            }]
        )
    ])
