import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.utilities import perform_substitutions


def _resolve_mjcf_path(pkg_share: str, raw: str) -> str:
    raw = raw.strip()
    if os.path.isabs(raw):
        return raw
    rel = raw.replace("\\", "/")
    candidate = os.path.join(pkg_share, rel)
    if os.path.isfile(candidate):
        return candidate
    return os.path.normpath(os.path.join(pkg_share, "scenes", os.path.basename(rel)))


def _resolve_hw_actuation(control_mode: str, mja_launch: str) -> str:
    if control_mode in ("effort", "torque"):
        return "torque"
    if mja_launch in ("direct", "kinematic", "qpos"):
        return "direct"
    return "pd_torque"


def _bringup_nodes(context, *_args, **_kwargs):
    import logging

    log = logging.getLogger("feixi_mujoco_ros2_control.launch")

    pkg_share = get_package_share_directory("feixi_mujoco_control")
    mjcf_raw = perform_substitutions(context, [LaunchConfiguration("mjcf_model_path")])
    mjcf = _resolve_mjcf_path(pkg_share, mjcf_raw)

    control_mode = context.launch_configurations.get("control_mode", "effort").lower().strip()
    mja_launch = perform_substitutions(
        context, [LaunchConfiguration("mujoco_joint_actuation")]
    ).lower().strip()
    hw_actuation = _resolve_hw_actuation(control_mode, mja_launch)

    joint_cmds = context.launch_configurations.get("joint_commands", "trajectory")
    init_pose_q = perform_substitutions(context, [LaunchConfiguration("init_pose_q")])
    init_lock_writes = perform_substitutions(context, [LaunchConfiguration("init_lock_writes")])
    viv = perform_substitutions(context, [LaunchConfiguration("enable_mujoco_viewer")])
    vw = perform_substitutions(context, [LaunchConfiguration("viewer_width")])
    vh = perform_substitutions(context, [LaunchConfiguration("viewer_height")])

    dynamics_mode_raw = context.launch_configurations.get("dynamics_mode", "none")
    pinocchio_urdf_arg = perform_substitutions(context, [LaunchConfiguration("pinocchio_urdf_path")])
    expose_vel_acc = perform_substitutions(context, [LaunchConfiguration("expose_joint_vel_acc_commands")])

    pinocchio_urdf_default = os.path.join(pkg_share, "urdf", "feixi_pinocchio_model.urdf")
    pinocchio_urdf = pinocchio_urdf_arg.strip() if pinocchio_urdf_arg else ""
    if not pinocchio_urdf:
        pinocchio_urdf = pinocchio_urdf_default

    dynamics_mode_lc = dynamics_mode_raw.lower().strip()
    use_pinocchio_cmds = dynamics_mode_lc in ("trajectory", "acceleration")
    if use_pinocchio_cmds and expose_vel_acc.lower() not in ("1", "true", "yes"):
        log.warning(
            "dynamics_mode=%s requires velocity/acceleration command interfaces; using expose_joint_vel_acc_commands:=true.",
            dynamics_mode_raw,
        )
        expose_vel_acc = "true"
    if dynamics_mode_lc not in ("none", "off", "0", "") and control_mode != "position":
        log.warning(
            "dynamics_mode=%s requires control_mode:=position; Pinocchio torques are disabled.",
            dynamics_mode_raw,
        )
    if dynamics_mode_lc not in ("none", "off", "0", "") and hw_actuation == "direct":
        log.warning(
            "dynamics_mode=%s is incompatible with mujoco_joint_actuation:=direct; use pd_torque.",
            dynamics_mode_raw,
        )

    if hw_actuation == "torque":
        cmd_iface = "effort"
        jtc_name = "feixi_joint_trajectory_effort.yaml"
    else:
        cmd_iface = "position"
        if use_pinocchio_cmds:
            jtc_name = "feixi_joint_trajectory_pinocchio.yaml"
        else:
            jtc_name = "feixi_joint_trajectory_controller.yaml"

    use_stream = joint_cmds == "stream"
    if use_stream and control_mode != "position":
        log.warning(
            "joint_commands:=stream requires control_mode:=position — "
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
                hw_actuation,
                " dynamics_mode:=",
                dynamics_mode_raw,
                " pinocchio_urdf_path:=",
                pinocchio_urdf,
                " expose_joint_vel_acc_commands:=",
                expose_vel_acc,
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
    run_ref = context.launch_configurations.get("reference_trajectory_test", "false").lower() in (
        "1",
        "true",
        "yes",
    )
    if run_test and run_ref:
        log.warning("run_test and reference_trajectory_test both true; running reference test only.")
        run_test = False

    if run_test:
        if use_stream:
            tester = Node(
                package="feixi_trajectory",
                executable="stream_feixi_joint_commands",
                name="feixi_test_stream",
                output="screen",
            )
        else:
            tester = Node(
                package="feixi_trajectory",
                executable="send_feixi_demo_trajectory",
                name="feixi_test_trajectory",
                output="screen",
            )
        out.append(TimerAction(period=8.0, actions=[tester]))

    if run_ref:
        if joint_cmds != "trajectory":
            log.warning("reference_trajectory_test ignored (joint_commands is not trajectory).")
        elif use_stream:
            log.warning("reference_trajectory_test ignored in stream mode.")
        else:
            ref_tester = Node(
                package="feixi_trajectory",
                executable="send_feixi_reference_trajectory",
                name="feixi_reference_traj_test",
                output="screen",
            )
            out.append(TimerAction(period=10.0, actions=[ref_tester]))

    return out


def generate_launch_description():
    mjcf_decl = DeclareLaunchArgument(
        "mjcf_model_path",
        default_value="scenes/arm_with_gripper.xml",
        description=(
            "MuJoCo scene path. Relative paths resolve under install/share/feixi_mujoco_control/ "
            "(default: scenes/arm_with_gripper.xml)."
        ),
    )

    control_decl = DeclareLaunchArgument(
        "control_mode",
        default_value="effort",
        description=(
            "'position': joint position commands via JointTrajectoryController / forward controller; "
            "'effort': joint torque commands (PD in JTC gains, hardware passthrough)."
        ),
        choices=["position", "effort"],
    )

    mujoco_actuation_decl = DeclareLaunchArgument(
        "mujoco_joint_actuation",
        default_value="pd_torque",
        description=(
            "MuJoCo inner actuation when control_mode=position: "
            "'pd_torque' = plugin PD on position error; "
            "'direct' = kinematic qpos snap after mj_step. Ignored when control_mode=effort."
        ),
        choices=["pd_torque", "direct"],
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
        default_value="0,0,0,0,0,0,0,0.05",
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

    viewer_decl = DeclareLaunchArgument(
        "enable_mujoco_viewer",
        default_value="true",
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

    dynamics_decl = DeclareLaunchArgument(
        "dynamics_mode",
        default_value="none",
        description=(
            "Pinocchio torque law in hardware: none | acceleration | position | trajectory | cartesian_wrench. "
            "Use trajectory + reference_trajectory_test for multi-point spline test."
        ),
    )

    pinocchio_urdf_decl = DeclareLaunchArgument(
        "pinocchio_urdf_path",
        default_value="",
        description=(
            "Absolute URDF for Pinocchio (no ros2_control tags). "
            "If empty, uses share/feixi_mujoco_control/urdf/feixi_pinocchio_model.urdf."
        ),
    )

    expose_vel_acc_decl = DeclareLaunchArgument(
        "expose_joint_vel_acc_commands",
        default_value="false",
        description=(
            "If true, add velocity+acceleration command interfaces on arm joints in URDF (needed for "
            "dynamics_mode=trajectory / acceleration with JointTrajectoryController)."
        ),
        choices=["true", "false"],
    )

    ref_traj_decl = DeclareLaunchArgument(
        "reference_trajectory_test",
        default_value="false",
        description=(
            "If true, ~10 s after launch send send_feixi_reference_trajectory (3 waypoints, 8 s motion). "
            "If both run_test and this are true, the demo trajectory test is skipped."
        ),
        choices=["true", "false"],
    )

    return LaunchDescription(
        [
            mjcf_decl,
            control_decl,
            mujoco_actuation_decl,
            joint_cmds_decl,
            init_pose_decl,
            init_lock_decl,
            viewer_decl,
            vw_decl,
            vh_decl,
            run_test_decl,
            dynamics_decl,
            pinocchio_urdf_decl,
            expose_vel_acc_decl,
            ref_traj_decl,
            OpaqueFunction(function=_bringup_nodes),
        ]
    )
