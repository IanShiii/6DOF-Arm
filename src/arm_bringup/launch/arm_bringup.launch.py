from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
import os

def generate_launch_description():
    arm_description_package = get_package_share_directory("arm_description")
    urdf_file = os.path.join(arm_description_package, 'urdf', 'arm.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

    moveit_config = (
        MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
        .robot_description(file_path=os.path.join(arm_description_package, "urdf", "arm.urdf.xacro"))
        .robot_description_semantic(file_path=os.path.join(arm_description_package, "srdf", "arm.srdf"))
        .to_moveit_configs()
    )

    moveit_servo_params = {
        "moveit_servo": ParameterBuilder("arm_moveit_config")
        .yaml("config/servo.yaml")
        .to_dict()
    }

    # Foxglove launch is wrapped in a GroupAction to isolate its global parameters.
    # Foxglove sets a `capabilities` parameter globally, which conflicts with
    # MoveIt's `move_group` capabilities if not scoped. GroupAction prevents
    # parameter leakage while still allowing Foxglove to start correctly.
    foxglove_bridge_launch = GroupAction([
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('foxglove_bridge'),
                    'launch',
                    'foxglove_bridge_launch.xml'
                )
            )
        )
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    arm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('arm_control'),
            '/launch',
            '/arm_control.launch.py'
        ])
    )

    move_group_launch = generate_move_group_launch(moveit_config)

    acceleration_filter_update_period = {"update_period": 0.02}
    planning_group_name = {"planning_group_name": "arm"}

    moveit_servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            moveit_servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    arm_exeuction_manager = Node(
        package='arm_execution_manager',
        executable='arm_execution_manager',
        output='screen'
    )

    operator_node = Node(
        package='arm_operator',
        executable='operator',
        output='screen'
    )

    return LaunchDescription([
        foxglove_bridge_launch,
        robot_state_publisher_node,
        arm_control,
        move_group_launch,
        moveit_servo_node,
        arm_exeuction_manager,
        operator_node
    ])
