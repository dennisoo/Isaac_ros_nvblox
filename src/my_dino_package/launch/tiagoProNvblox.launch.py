import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory

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
    nvblox_config = os.path.join(
        get_package_share_directory('my_dino_package'),
        'config',
        'nvblox_params.yaml'
    )

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/workspaces/isaac_ros-dev/bags/tiago_semantic_bag_new',
    )
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1',
        description='Playback rate'
    )
    output_mesh_arg = DeclareLaunchArgument(
        'output_mesh',
        default_value='/workspaces/isaac_ros-dev/meshes/semantic_tiago_mesh.glb',
        description='Output mesh path (.glb)'
    )
    # Static Tf from torso_base_link to torso_lift_link
    torso_base_to_lift = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='torso_base_to_lift',
        arguments=[
            '--frame-id', 'torso_base_link',
            '--child-frame-id', 'torso_lift_link',
            '--x', '0', '--y', '0', '--z', '0.9',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen'
    )
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
                    nvblox_config,
                }],
                remappings=[
                    ('camera_0/depth/image', '/head_front_camera/depth/image_rect_raw'),
                    ('camera_0/depth/camera_info', '/head_front_camera/depth/camera_info'),
                    #('camera_0/color/image', '/head_front_camera/color/image_raw'),
                    #('camera_0/color/camera_info', '/head_front_camera/color/camera_info'),

                    # We are not using segmentation for now
                    ('camera_0/color/image', '/semantic/image_rgb8'),
                    ('camera_0/color/camera_info', '/semantic/camera_info'),
                    ('pointcloud', '/merged_cloud'),
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
    return LaunchDescription([
        bag_path_arg,
        rate_arg,
        output_mesh_arg,
        # Bag player starts first to publish /clock
        bag_player,
        TimerAction(period=1.0, actions=[nvblox_container]),
        # Processing (currently disabled)
        save_mesh_on_exit
    ])
