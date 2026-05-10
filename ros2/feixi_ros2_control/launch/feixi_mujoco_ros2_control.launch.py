import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.utilities import perform_substitutions


def _bringup_nodes(context, *_args, **_kwargs):
    import logging

    log = logging.getLogger("feixi_mujoco_ros2_control.launch")

    pkg_share = get_package_share_directory("feixi_ros2_control")
    mjcf = perform_substitutions(context, [LaunchConfiguration("mjcf_model_path")])
    mode = context.launch_configurations.get("control_mode", "position")
    joint_cmds = context.launch_configurations.get("joint_commands", "trajectory")
    init_pose_q = perform_substitutions(context, [LaunchConfiguration("init_pose_q")])
    init_lock_writes = perform_substitutions(context, [LaunchConfiguration("init_lock_writes")])
    mujoco_joint_actuation = perform_substitutions(
        context, [LaunchConfiguration("mujoco_joint_actuation")]
    )
    viv = perform_substitutions(context, [LaunchConfiguration("enable_mujoco_viewer")])
    vw = perform_substitutions(context, [LaunchConfiguration("viewer_width")])
    vh = perform_substitutions(context, [LaunchConfiguration("viewer_height")])

    cmd_iface = "effort" if mode == "effort" else "position"
    jtc_name = (
        "feixi_joint_trajectory_effort.yaml"
        if mode == "effort"
        else "feixi_joint_trajectory_controller.yaml"
    )

    use_stream = joint_cmds == "stream"
    if use_stream and mode == "effort":
        log.warning(
            "joint_commands:=stream expects position hardware PD; control_mode is effort — "
            "falling back to feixi_trajectory_controller."
        )
        use_stream = False

    urdf_xacro = os.path.join(pkg_share, "urdf", "feixi.urdf.xacro")
    urdf_proc = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_xacro,
                " mjcf_model_path:=",
                mjcf,
                " command_interface:=",
                cmd_iface,
                " enable_mujoco_viewer:=",
                viv,
                " viewer_width:=",
                vw,
                " viewer_height:=",
                vh,
                " init_pose_q:=",
                init_pose_q,
                " init_lock_writes:=",
                init_lock_writes,
                " mujoco_joint_actuation:=",
                mujoco_joint_actuation,
            ]
        ),
        value_type=str,
    )

    controllers = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    forward_yaml = os.path.join(pkg_share, "config", "feixi_forward_position_controller.yaml")
    traj_yaml = os.path.join(pkg_share, "config", jtc_name)

    ros2_params = [{"robot_description": urdf_proc}, controllers, traj_yaml, forward_yaml]

    if use_stream:
        main_controller = "feixi_forward_position_controller"
    else:
        main_controller = "feixi_trajectory_controller"

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_proc}],
    )

    ros2_cm = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=ros2_params,
    )

    spawners = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager-timeout",
                    "120",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    main_controller,
                    "--controller-manager-timeout",
                    "120",
                ],
            ),
        ],
    )

    out = [rsp, ros2_cm, spawners]

    run_test = context.launch_configurations.get("run_test", "false").lower() in ("1", "true", "yes")
    if run_test:
        # After spawners (t=6s), give controllers a moment to activate.
        if use_stream:
            tester = Node(
                package="feixi_ros2_control",
                executable="stream_feixi_joint_commands",
                name="feixi_test_stream",
                output="screen",
            )
        else:
            tester = Node(
                package="feixi_ros2_control",
                executable="send_feixi_demo_trajectory",
                name="feixi_test_trajectory",
                output="screen",
            )
        out.append(TimerAction(period=8.0, actions=[tester]))

    return out


def generate_launch_description():
    pkg_share = get_package_share_directory("feixi_ros2_control")
    default_mjcf = os.path.join(pkg_share, "scenes", "arm_with_gripper.xml")

    mjcf_decl = DeclareLaunchArgument(
        "mjcf_model_path",
        default_value=default_mjcf,
        description="Absolute MJCF path (typically install/share/feixi_ros2_control/scenes/arm_with_gripper.xml).",
    )

    control_decl = DeclareLaunchArgument(
        "control_mode",
        default_value="position",
        description="'position': PD inside hardware plugin; 'effort': torque passthrough hardware + PID in JointTrajectoryController.",
        choices=["position", "effort"],
    )

    joint_cmds_decl = DeclareLaunchArgument(
        "joint_commands",
        default_value="trajectory",
        description=(
            "'trajectory': spawn feixi_trajectory_controller (FollowJointTrajectory). "
            "'stream': spawn feixi_forward_position_controller + publish Float64MultiArray for realtime loops."
        ),
        choices=["trajectory", "stream"],
    )

    init_pose_decl = DeclareLaunchArgument(
        "init_pose_q",
        default_value="0,-0.5,0,1.2,0,0.8,0,0.05",
        description=(
            "8 comma-separated joint angles (rad) seeded in MuJoCo on load: joint1..7, gripper_finger_joint."
        ),
    )

    init_lock_decl = DeclareLaunchArgument(
        "init_lock_writes",
        default_value="500",
        description=(
            "After each ros2_control write during startup, snap arm+gripper to init_pose_q this many times "
            "(kinematic hold; at 500 Hz update_rate ~1 s). Use 0 to disable snap (still seeds once on load)."
        ),
    )

    mujoco_actuation_decl = DeclareLaunchArgument(
        "mujoco_joint_actuation",
        default_value="pd_torque",
        description=(
            "MuJoCo inner realization for position commands (control_mode=position only): "
            "'pd_torque' = motor torques from plugin PD; "
            "'direct' = after mj_step, set joint angles to commands (kinematic / ideal, weak physical contact)."
        ),
        choices=["pd_torque", "direct"],
    )

    viewer_decl = DeclareLaunchArgument(
        "enable_mujoco_viewer",
        default_value="false",
        description="If true, open GLFW MuJoCo native viewer (needs DISPLAY / GPU).",
        choices=["true", "false"],
    )

    vw_decl = DeclareLaunchArgument(
        "viewer_width",
        default_value="960",
        description="MuJoCo viewer framebuffer width (pixels).",
    )

    vh_decl = DeclareLaunchArgument(
        "viewer_height",
        default_value="720",
        description="MuJoCo viewer framebuffer height (pixels).",
    )

    run_test_decl = DeclareLaunchArgument(
        "run_test",
        default_value="false",
        description=(
            "If true, after controllers start, run a tiny built-in test: "
            "stream mode → stream_feixi_joint_commands; trajectory → send_feixi_demo_trajectory."
        ),
        choices=["true", "false"],
    )

    return LaunchDescription(
        [
            mjcf_decl,
            control_decl,
            joint_cmds_decl,
            init_pose_decl,
            init_lock_decl,
            mujoco_actuation_decl,
            viewer_decl,
            vw_decl,
            vh_decl,
            run_test_decl,
            OpaqueFunction(function=_bringup_nodes),
        ]
    )
