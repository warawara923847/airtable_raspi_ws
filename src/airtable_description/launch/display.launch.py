from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import FindExecutable


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("airtable_description"), "urdf", "airtable.urdf.xacro"]
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
        ]
    )
