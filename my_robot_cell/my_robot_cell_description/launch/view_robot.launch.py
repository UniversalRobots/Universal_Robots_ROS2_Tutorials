from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    description_package = FindPackageShare("my_robot_cell_description")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "my_robot_cell.urdf.xacro"]
    )
    rvizconfig_file = PathJoinSubstitution([description_package, "rviz", "urdf.rviz"])

    robot_description = ParameterValue(
        Command(["xacro ", description_file, " ", "ur_type:=", ur_type]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            description="Typo/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur20",
        )
    ]

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
