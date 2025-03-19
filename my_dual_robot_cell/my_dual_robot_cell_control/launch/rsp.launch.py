from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    alice_ur_type = LaunchConfiguration("alice_ur_type")
    bob_ur_type = LaunchConfiguration("bob_ur_type")

    alice_robot_ip = LaunchConfiguration("alice_robot_ip")
    bob_robot_ip = LaunchConfiguration("bob_robot_ip")

    alice_use_mock_hardware = LaunchConfiguration("alice_use_mock_hardware")
    alice_mock_sensor_commands = LaunchConfiguration("alice_mock_sensor_commands")
    bob_use_mock_hardware = LaunchConfiguration("bob_use_mock_hardware")
    bob_mock_sensor_commands = LaunchConfiguration("bob_mock_sensor_commands")

    headless_mode = LaunchConfiguration("headless_mode")

    alice_kinematics_parameters_file = LaunchConfiguration(
        "alice_kinematics_parameters_file"
    )
    bob_kinematics_parameters_file = LaunchConfiguration(
        "bob_kinematics_parameters_file"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "urdf",
                    "my_dual_robot_cell_controlled.urdf.xacro",
                ]
            ),
            " ",
            "alice_robot_ip:=",
            alice_robot_ip,
            " ",
            "bob_robot_ip:=",
            bob_robot_ip,
            " ",
            "alice_ur_type:=",
            alice_ur_type,
            " ",
            "bob_ur_type:=",
            bob_ur_type,
            " ",
            "alice_use_mock_hardware:=",
            alice_use_mock_hardware,
            " ",
            "bob_use_mock_hardware:=",
            bob_use_mock_hardware,
            " ",
            "alice_kinematics_parameters_file:=",
            alice_kinematics_parameters_file,
            " ",
            "bob_kinematics_parameters_file:=",
            bob_kinematics_parameters_file,
            " ",
            "alice_mock_sensor_commands:=",
            alice_mock_sensor_commands,
            " ",
            "bob_mock_sensor_commands:=",
            bob_mock_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_ur_type",
            description="Type/series of used UR robot.",
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
            default_value="ur3e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_ur_type",
            description="Type/series of used UR robot.",
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
            default_value="ur3e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_robot_ip",
            default_value="192.168.0.101",
            description="IP address by which alice can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_robot_ip",
            default_value="192.168.0.100",
            description="IP address by which bob can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "alice_calibration.yaml",
                ]
            ),
            description="The calibration configuration of alice.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "bob_calibration.yaml",
                ]
            ),
            description="The calibration configuration of bob.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_use_mock_hardware",
            default_value="false",
            description="Start alice with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_use_mock_hardware",
            default_value="false",
            description="Start bob with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for alice's sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for bob's sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
        ]
    )
