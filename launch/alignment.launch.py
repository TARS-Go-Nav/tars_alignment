import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    alignment_pck_path = get_package_share_directory('alignment')
    alignment_params = os.path.join(alignment_pck_path, 'config', 'config.yaml')

    cmd_init_tf_publisher = Node(
        package    = 'alignment',
        executable = 'init_tf_publisher_node',
        name       = 'init_tf_publisher',
        parameters = [alignment_params],
        output='screen'
    )

    cmd_init_pose_alignment = Node(
        package    = 'alignment',
        executable = 'init_pose_alignment_node',
        name       = 'init_pose_alignment',
        parameters = [alignment_params],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(cmd_init_tf_publisher)
    ld.add_action(cmd_init_pose_alignment)

    return ld
