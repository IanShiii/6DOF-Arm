from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    controllers_file = os.path.join(get_package_share_directory('arm_control'), 'config', 'ros2_controllers.yaml')
    
    ros2_controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers_file]
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager-timeout',
            '300',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hand_controller',
            '--controller-manager-timeout',
            '300',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout',
            '300',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    return LaunchDescription([
        ros2_controller_manager_node,
        arm_controller_spawner,
        hand_controller_spawner,
        joint_state_broadcaster_spawner
    ])
