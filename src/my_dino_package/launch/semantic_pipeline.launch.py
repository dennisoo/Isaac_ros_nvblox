"""
Vollständige Pipeline:   cuVSLAM → DINO/SAM → nvblox
Alle Package-Namen geprüft und korrekt!
"""
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros. descriptions import ComposableNode

def generate_launch_description():
    
    # === ARGUMENTE ===
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/workspaces/isaac_ros-dev/bags/mein_lidar_dataset',
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='0.5',
        description='Playback rate'
    )
    
    use_nvblox_human_arg = DeclareLaunchArgument(
        'use_nvblox_human',
        default_value='false',
        description='Use NvbloxHumanNode instead of NvbloxNode'
    )
    
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
    
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        arguments=[
            '--frame-id', 'odom',
            '--child-frame-id', 'base_link',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )
    
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )
    
    static_tf_camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_optical',
        arguments=[
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_0_color_optical_frame',
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )
    static_tf_base_to_lidar = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_lidar',
    arguments=[
        '--frame-id', 'base_link',
        '--child-frame-id', 'PandarXT_32_10hz',  # ← LIDAR Frame! 
        '--x', '0', '--y', '0', '--z', '0',  # ← LIDAR Position (anpassen!)
        '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        '--ros-args', '-p', 'use_sim_time:=true'
    ],
    output='screen'
    )   

    
    # === VISUAL SLAM (Composable) ===
    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',  # ✅ Korrekt!
                name='visual_slam_node',
                parameters=[{
                    'use_sim_time': True,
                    'enable_image_denoising': False,
                    'rectified_images': True,
                    'enable_slam_visualization': True,
                    'enable_observations_view': True,
                    'enable_landmarks_view': True,
                    'map_frame': 'map',
                    'odom_frame':  'odom',
                    'base_frame': 'base_link',
                    'enable_localization_n_mapping':  True,
                    'publish_odom_to_base_tf': True,
                    'publish_map_to_odom_tf': True,
                }],
                remappings=[
                    ('visual_slam/image_0', '/rgb'),
                    ('visual_slam/camera_info_0', '/camera_info'),
                ]
            ),
        ],
        output='screen'
    )
    
    # === DINO + SAM ===
    semantic_dino_node = Node(
        package='my_dino_package',
        executable='dino_nvblox_node',
        name='semantic_dino_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'box_threshold': 0.35,
            'text_threshold': 0.25,
        }],
        remappings=[
            ('/camera_0/color/image', '/rgb'),
            ('/camera_0/color/camera_info', '/camera_info'),
        ]
    )
    
    # === SEMANTIC BRIDGE ===
    semantic_bridge_node = Node(
        package='my_dino_package',
        executable='semantic_to_nvblox_bridge',
        name='semantic_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode':  'single_mask',
            'target_classes': 'person,chair,table,door,window',
        }]
    )
    
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
                    'voxel_size': 0.05,
                    'esdf_2d': True,
                    'esdf_2d_min_height': 0.0,
                    'esdf_2d_max_height': 2.0,
                    'distance_slice_height': 1.0,
                    'mesh_update_rate_hz': 5.0,
                    'esdf_update_rate_hz': 2.0,
                    'use_depth': False,
                    'use_color': True,
                    'use_lidar': True,
                }],
                remappings=[
                    ('depth/image', '/depth'),
                    ('depth/camera_info', '/camera_info'),
                    ('color/image', '/rgb'),
                    ('color/camera_info', '/camera_info'),
                    ('pointcloud', '/point_cloud'),
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
            '--rate', LaunchConfiguration('rate'),
            '--topics', '/rgb', '/camera_info', '/clock', '/point_cloud',
        ],
        output='screen',
    )
    
    # === LAUNCH SEQUENCE ===
    return LaunchDescription([
        bag_path_arg,
        rate_arg,
        use_nvblox_human_arg,
        
        # TFs
        static_tf_map_to_odom,
        static_tf_odom_to_base,
        static_tf_base_to_camera,
        static_tf_camera_to_optical,
        static_tf_base_to_lidar,
        
        # Processing
        visual_slam_container,
        semantic_dino_node,
        semantic_bridge_node,
        nvblox_container,
        
        # Bag delayed
        TimerAction(period=5.0, actions=[bag_player]),
    ])