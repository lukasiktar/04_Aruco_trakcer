import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('marker_size', default_value='0.1',
                              description='Printed marker side length in metres'),
        DeclareLaunchArgument('dictionary_family', default_value='4x4'),
        DeclareLaunchArgument('dictionary', default_value='1000'),
        DeclareLaunchArgument('view', default_value='true',
                              description='Show the annotated image in an OpenCV window'),

        # D435i colour stream -> /camera/camera/color/{image_raw,camera_info}
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'camera_name': 'camera',
                'camera_namespace': 'camera',
                'enable_color': 'true',
                'enable_depth': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'rgb_camera.color_profile': '1280x720x30',
            }.items(),
        ),

        Node(
            package='aruco_tracker',
            executable='aruco_tracker_node',
            output='screen',
            parameters=[{
                'marker_size': ParameterValue(LaunchConfiguration('marker_size'), value_type=float),
                'dictionary_family': ParameterValue(
                    LaunchConfiguration('dictionary_family'), value_type=str),
                'dictionary': ParameterValue(LaunchConfiguration('dictionary'), value_type=int),
                'show_window': ParameterValue(LaunchConfiguration('view'), value_type=bool),
            }],
        ),
    ])
