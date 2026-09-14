from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    teleop_params = PathJoinSubstitution(
        [FindPackageShare("airtable_teleop"), "config", "teleop.yaml"]
    )

    controller_params = PathJoinSubstitution(
        [FindPackageShare("airtable_controller"), "config", "thrusters.yaml"]
    )

    joy_node = Node(
        package="joy",
        executable="game_controller_node",
        name="game_controller_node",
        output="screen",
    )

    teleop_node = Node(
        package="airtable_teleop",
        executable="airtable_teleop_node",
        name="airtable_teleop",
        output="screen",
        parameters=[teleop_params],
    )

    controller_node = Node(
        package="airtable_controller",
        executable="airtable_controller_node",
        name="airtable_controller",
        output="screen",
        parameters=[controller_params],
    )

    hardware_node = Node(
        package="airtable_hardware",
        executable="airtable_hardware",
        name="airtable_hardware",
        output="screen",
    )

    return LaunchDescription(
        [
            joy_node,
            teleop_node,
            controller_node,
            hardware_node,
        ]
    )
