"""Launch the installed RGB-D node; camera intrinsics must match your camera."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('orbslam3_dense_ros2')
    defaults = {
        'experiment_output_dir': 'results',
        'sequence_name': 'sequence',
        'vocabulary_file': share + '/Vocabulary/ORBvoc.txt.bin',
        'settings_file': share + '/config/RGB-D/RealSense_D435i.yaml',
        'enable_pangolin': 'false',
        'rgb_topic': '/camera/camera/color/image_raw',
        'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
        'use_sim_time': 'false',
        'ground_frame': 'odom',
        'camera_frame': 'camera_color_optical_frame',
        'vehicle_frame': 'base_footprint',
        'publish_rate_hz': '30.0',
        'sync_queue_size': '10',
        'image_reliability': 'best_effort',
        'sync_max_interval': '0.05',
    }
    types = {'enable_pangolin': bool, 'use_sim_time': bool,
             'publish_rate_hz': float, 'sync_queue_size': int, 'sync_max_interval': float}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        Node(package='orbslam3_dense_ros2', executable='orb_slam3_main', output='screen',
             parameters=[{name: ParameterValue(LaunchConfiguration(name),
                                              value_type=types.get(name, str))
                          for name in defaults}]),
    ])
