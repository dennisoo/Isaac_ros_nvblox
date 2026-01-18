import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo') 
    default_world = os.path.join(pkg_share, 'worlds', 'tugboat.sdf')

    world = LaunchConfiguration('world')

    
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tugbot_camera_bridge',
        output='screen',
        arguments=[
            # Clock
            '/world/world_demo/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # TF 
            '/model/tugbot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # Odometry
            '/model/tugbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # Camera front - Color
            '/world/world_demo/model/tugbot/link/camera_front/sensor/color/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/world_demo/model/tugbot/link/camera_front/sensor/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # Camera front - Depth
            '/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/world_demo/model/tugbot/link/camera_front/sensor/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # IMU
            '/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # LiDAR - Omni (360°)
            '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/model/tugbot/tf', '/tf'),
            ('/model/tugbot/odometry', '/odom'),
            ('/world/world_demo/model/tugbot/link/camera_front/sensor/color/image', '/camera/color/image'),
            ('/world/world_demo/model/tugbot/link/camera_front/sensor/color/camera_info', '/camera/color/camera_info'),
            ('/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image', '/camera/depth/image'),
            ('/world/world_demo/model/tugbot/link/camera_front/sensor/depth/camera_info', '/camera/depth/camera_info'),
            ('/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu', '/imu'),
            ('/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan/points', '/pointcloud'),
            ('/world/world_demo/clock', '/clock'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to the SDF world file.'
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(get_package_share_directory('gazebo'), 'models')
        ),
        gz_sim,
        bridge,
    ])