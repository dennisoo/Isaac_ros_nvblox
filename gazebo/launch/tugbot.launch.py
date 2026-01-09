import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo')  # <-- dein Package-Name
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
            '/world/world_demo/model/tugbot/link/camera_front/sensor/color/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/world_demo/model/tugbot/link/camera_front/sensor/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/world_demo/model/tugbot/link/camera_front/sensor/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
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