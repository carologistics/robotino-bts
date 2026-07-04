from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    launch_gray_box_controller = LaunchConfiguration("launch_gray_box_controller")
    gray_box_show_gui = LaunchConfiguration("gray_box_show_gui")
    gray_box_enable_motion = LaunchConfiguration("gray_box_enable_motion")
    gray_box_use_motor_move = LaunchConfiguration("gray_box_use_motor_move")
    gray_box_run_only_during_action = LaunchConfiguration("gray_box_run_only_during_action")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Top-level namespace for the robotino tree server.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robotino_behavior_tree"),
                        "config",
                        "tree_execution_server.yaml",
                    ]
                ),
                description="Path to the TreeExecutionServer parameters file.",
            ),
            DeclareLaunchArgument(
                "launch_gray_box_controller",
                default_value="true",
                description="Launch the gray box plane alignment action server.",
            ),
            DeclareLaunchArgument(
                "gray_box_show_gui",
                default_value="true",
                description="Show the OpenCV debug window for gray box alignment.",
            ),
            DeclareLaunchArgument(
                "gray_box_enable_motion",
                default_value="false",
                description=(
                    "Enable continuous gray controller motion outside an active "
                    "gray_box_align_action goal."
                ),
            ),
            DeclareLaunchArgument(
                "gray_box_use_motor_move",
                default_value="true",
                description="Use motor_move_action goals instead of cmd_vel for gray box alignment.",
            ),
            DeclareLaunchArgument(
                "gray_box_run_only_during_action",
                default_value="true",
                description="Only process camera/cloud frames while gray_box_align_action is active.",
            ),
            Node(
                package="robotino_behavior_tree",
                executable="robotino_tree_server",
                namespace=namespace,
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="robotino_behavior_tree",
                executable="gray_box_plane_controller.py",
                namespace=namespace,
                output="screen",
                condition=IfCondition(launch_gray_box_controller),
                parameters=[
                    {
                        "show_gui": gray_box_show_gui,
                        "enable_motion": gray_box_enable_motion,
                        "use_motor_move": gray_box_use_motor_move,
                        "run_only_during_action": gray_box_run_only_during_action,
                    }
                ],
            ),
        ]
    )
