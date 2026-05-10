from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("feixi_ros2_control")
    urdf_file = PathJoinSubstitution([pkg, "urdf", "feixi_mock.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "ros2_controllers.yaml"])
    jtc = PathJoinSubstitution([pkg, "config", "feixi_joint_trajectory_arm7_mock.yaml"])

    robot_description = ParameterValue(Command(["xacro ", urdf_file]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controllers,
            jtc,
        ],
    )

    spawners_after = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager-timeout",
                    "60",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "feixi_trajectory_controller",
                    "--controller-manager-timeout",
                    "60",
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            spawners_after,
        ]
    )
