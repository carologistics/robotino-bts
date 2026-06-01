from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")

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
            Node(
                package="robotino_behavior_tree",
                executable="robotino_tree_server",
                namespace=namespace,
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
