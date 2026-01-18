
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown

def save_mesh_as_glb(context):
    import subprocess
    import os
    import trimesh
    import tempfile
    output_path = LaunchConfiguration('output_mesh').perform(context)
    print("Saving mesh to:", output_path, " Conversion starting...")
    temp_ply = tempfile.mktemp(suffix='.ply')
    result = subprocess.run([
        'ros2', 'service', 'call', 
        '/nvblox_node/save_ply',
        'nvblox_msgs/srv/FilePath', 
        f'{{file_path: "{temp_ply}"}}'
    ], capture_output=True, text=True)
    if result.returncode != 0:
        print("Error exporting mesh:", result.stderr)
        return
    print("Mesh exported to temporary PLY, converting to GLB...")
    try:
        mesh = trimesh.load(temp_ply)    
        mesh.export(output_path, file_type='glb')
        print("Mesh successfully saved as GLB to:", output_path)
    except Exception as e:
        print("Error during mesh conversion:", str(e))
    finally:
        if os.path.exists(temp_ply):
            os.remove(temp_ply)    
    print("Triggering global shutdown...")
    return [EmitEvent(event=Shutdown(reason='Bag finished and mesh saved'))]        


def generate_launch_description():
    
    # === ARGUMENTE ===
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/workspaces/isaac_ros-dev/bags/tugbot_slam_bag',
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='5',  # Can be faster with preprocessed bag
        description='Playback rate'
    )
    
    use_nvblox_human_arg = DeclareLaunchArgument(
        'use_nvblox_human',
        default_value='false',
        description='Use NvbloxHumanNode instead of NvbloxNode'
    )
    output_mesh_arg = DeclareLaunchArgument(
        'output_mesh',
        default_value='/workspaces/isaac_ros-dev/meshes/semantic_mesh.glb',
        description='Output mesh path (.glb)'
    )
    
    # Static TF: map -> odom (da Visual SLAM auskommentiert)
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )
    
    # Static TF: base_link -> color camera (Link: 0.0553,0,0.4323 + Sensor: 0,0.0325,0.0125)
    # Optical frame rotation: Z forward, X right, Y down (qx=-0.5, qy=0.5, qz=-0.5, qw=0.5)
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'tugbot/camera_front/color',
            '--x', '0.0553', '--y', '0.0325', '--z', '0.4448',
            '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )
    
    # Static TF: base_link -> depth camera (Link: 0.0553,0,0.4323 + Sensor: 0,0.0175,0.0125)
    # Optical frame rotation: Z forward, X right, Y down (qx=-0.5, qy=0.5, qz=-0.5, qw=0.5)
    static_tf_base_to_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_depth',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'tugbot/camera_front/depth',
            '--x', '0.0553', '--y', '0.0175', '--z', '0.4448',
            '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )
    
    # Static TF: base_link -> lidar (scan_omni at -0.1855,0,0.5318)
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'tugbot/scan_omni/scan_omni',
            '--x', '-0.1855', '--y', '0', '--z', '0.5318',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )
    
    # === VISUAL SLAM (Composable) === für Simulation auskommentiert
    # visual_slam_container = ComposableNodeContainer(
    #     name='visual_slam_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container_mt',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='isaac_ros_visual_slam',
    #             plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
    #             name='visual_slam_node',
    #             parameters=[{
    #                 'use_sim_time': True,
    #                 'enable_image_denoising': False,
    #                 'rectified_images': True,
    #                 'enable_slam_visualization': True,
    #                 'enable_observations_view': True,
    #                 'enable_landmarks_view': True,
    #                 'map_frame': 'map',
    #                 'odom_frame':  'odom',
    #                 'base_frame': 'base_link',
    #                 'enable_localization_n_mapping':  True,
    #                 'publish_odom_to_base_tf': False,
    #                 'publish_map_to_odom_tf': True,
    #             }],
    #             remappings=[
    #                 ('visual_slam/image_0', '/camera/color/image'),
    #                 ('visual_slam/camera_info_0', '/camera/color/camera_info'),
    #                 ('visual_slam/imu', '/imu'),
    #             ]
    #         ),
    #     ],
    #     output='screen'
    # )
    
    # === DINO + SAM === DISABLED (using preprocessed bag with semantic topics)
    # semantic_dino_node = Node(
    #     package='my_dino_package',
    #     executable='dino_nvblox_node',
    #     name='semantic_dino_node',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'box_threshold': 0.35,
    #         'text_threshold': 0.25,
    #     }],
    #     remappings=[
    #         ('image', '/camera/color/image'),
    #         ('camera_info', '/camera/color/camera_info'),
    #     ]
    # )
    
    # === NVBLOX ===
    nvblox_container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='nvblox_ros',
                plugin='nvblox::NvbloxNode',
                name='nvblox_node',
                parameters=[{
                    'use_sim_time': True,
                    'global_frame': 'odom',
                    'pose_frame': 'base_link',
                    'use_tf_transforms': True,
                    'voxel_size': 0.05,

                    'mapping_type': 'static_tsdf',
                    'tick_period_ms': 10,
                    'integrate_depth_rate_hz': 40.0,
                    'integrate_color_rate_hz': 5.0,
                    'update_mesh_rate_hz': 5.0,
                    'update_esdf_rate_hz': 10.0,
                    
                    # ESDF
                    'esdf_2d': True,
                    'esdf_2d_min_height': 0.0,
                    'esdf_2d_max_height': 2.0,
                    
                    # Inputs
                    'use_depth': True,
                    'use_color': True,
                    'use_lidar': True,
                    'use_segmentation': False,
                    
                    # LiDAR settings (from SDF: 900 horizontal x 16 vertical samples)
                    'lidar_width': 900,
                    'lidar_height': 16,
                    'lidar_min_valid_range_m': 0.2,
                    'lidar_max_valid_range_m': 100.0,
                    
                    # QoS für Isaac Sim/Gazebo bags
                    'input_qos': 'DEFAULT',
                    
                    # Increase queue for time sync (DINO/SAM adds latency)
                    'maximum_input_queue_length': 30,
                    
                    # Map clearing
                    'map_clearing_radius_m': 15.0,
                    'map_clearing_frame_id': 'base_link',
                }],
                remappings=[
                    ('camera_0/depth/image', '/camera/depth/image'),
                    ('camera_0/depth/camera_info', '/camera/depth/camera_info'),
                    # Use semantic RGB from DINO - needs slow playback rate!
                    #('camera_0/color/image', '/camera/color/image'),
                    #('camera_0/color/camera_info', '/camera/color/camera_info'),
                    ('camera_0/color/image', '/semantic/image_rgb8'),
                    ('camera_0/color/camera_info', '/semantic/camera_info'),
                    ('pointcloud', '/pointcloud'),
                ]
            ),
        ],
        output='screen'
    )
    
    # === BAG PLAYER ===
    bag_player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--clock',
            '--rate', LaunchConfiguration('rate'),
            # Added semantic topics for preprocessed bag
            '--topics', '/camera/color/image', '/camera/color/camera_info', '/camera/depth/camera_info', '/camera/depth/image', '/semantic/image_rgb8', '/semantic/image_mono8', '/semantic/camera_info', '/tf', '/odom', '/imu', '/pointcloud'
        ],
        output='screen',
    )
    save_mesh_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player,
            on_exit = [
                OpaqueFunction(function=save_mesh_as_glb)
            ]
        )
    )
    # === LAUNCH SEQUENCE ===
    # Bag player starts FIRST so /clock is available for use_sim_time nodes
    return LaunchDescription([
        bag_path_arg,
        rate_arg,
        use_nvblox_human_arg,
        output_mesh_arg,
        # Bag player starts first to publish /clock
        bag_player,
        
        # Static TFs start after bag (needs /clock for sim time)
        TimerAction(period=1.0, actions=[static_tf_map_to_odom]),
        TimerAction(period=1.0, actions=[static_tf_base_to_camera]),
        TimerAction(period=1.0, actions=[static_tf_base_to_depth]),
        TimerAction(period=1.0, actions=[static_tf_base_to_lidar]),
        
        # DINO and semantic bridge DISABLED (using preprocessed bag)
        # TimerAction(period=1.5, actions=[semantic_dino_node]),
        # TimerAction(period=1.5, actions=[semantic_bridge_node]),
        
        # nvblox starts after TFs are ready
        TimerAction(period=2.0, actions=[nvblox_container]),
        
        # Processing (currently disabled)
        # visual_slam_container,  # Auskommentiert - Gazebo liefert perfekte Odometry
        save_mesh_on_exit
       
    ])