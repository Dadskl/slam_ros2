"""Stereo-inertial ORB-SLAM3; requires a calibrated IMU-enabled YAML and synchronized data."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('settings_file', description='Stereo YAML with calibrated IMU.T_b_c1 and noise values'),
        DeclareLaunchArgument('left_topic', default_value='/kitti/left/image_raw'),
        DeclareLaunchArgument('right_topic', default_value='/kitti/right/image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/kitti/imu'),
        DeclareLaunchArgument('enable_pangolin', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('trajectory_output_prefix', default_value=''),
        DeclareLaunchArgument('experiment_output_dir', default_value='results'),
        DeclareLaunchArgument('image_reliability', default_value='best_effort'),
        DeclareLaunchArgument('sequence_name', default_value='sequence'),
        Node(package='orbslam3_dense_ros2', executable='orb_slam3_main', output='screen', parameters=[{
            'experiment_output_dir': ParameterValue(LaunchConfiguration('experiment_output_dir'), value_type=str),
            'sequence_name': ParameterValue(LaunchConfiguration('sequence_name'), value_type=str),
            'image_reliability': ParameterValue(LaunchConfiguration('image_reliability'), value_type=str),
            'sensor_mode': 'stereo_inertial',
            'settings_file': ParameterValue(LaunchConfiguration('settings_file'), value_type=str),
            'left_topic': LaunchConfiguration('left_topic'),
            'right_topic': LaunchConfiguration('right_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'camera_frame': 'kitti_left_optical_frame',
            'ground_frame': 'map',
            'enable_pangolin': ParameterValue(LaunchConfiguration('enable_pangolin'), value_type=bool),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'trajectory_output_prefix': ParameterValue(LaunchConfiguration('trajectory_output_prefix'), value_type=str),
        }]),
    ])
