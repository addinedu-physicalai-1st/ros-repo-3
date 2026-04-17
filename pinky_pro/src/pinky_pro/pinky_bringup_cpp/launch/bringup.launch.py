import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Paths
    pinky_bringup_cpp_dir = get_package_share_directory('pinky_bringup_cpp')
    pinky_description_dir = get_package_share_directory('pinky_description')
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # EKF Config
    ekf_config = os.path.join(pinky_bringup_cpp_dir, 'params', 'ekf.yaml')

    # 1. Robot Description (URDF)
    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pinky_description_dir, 'launch', 'upload_robot.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Lidar (SLLidar C1)
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_dir, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyAMA0',
            'frame_id': 'rplidar_link',
            'inverted': 'false',
            'angle_compensate': 'true',
            'scan_mode': 'DenseBoost'
        }.items()
    )

    # 3. Pinky Bringup Node (C++)
    pinky_bringup = Node(
        package='pinky_bringup_cpp',
        executable='pinky_bringup_node',
        name='pinky_bringup_cpp',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'control_rate': 50.0,
            'publish_tf': False, # EKF will publish TF
        }]
    )

    # 4. IMU Node (C++)
    pinky_imu = Node(
        package='pinky_imu_bno055',
        executable='main_node',
        name='pinky_imu_bno055',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'interface': '/dev/i2c-0',
            'frame_id': 'imu_link',
            'rate': 100.0
        }]
    )

    # 5. EKF Node (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    # 6. Battery Publisher (Python)
    battery_pub = Node(
        package='pinky_bringup',
        executable='battery_publisher',
        name='battery_publisher',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        upload_robot,
        lidar,
        pinky_bringup,
        pinky_imu,
        ekf_node,
        battery_pub
    ])
