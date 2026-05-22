#!/usr/bin/env python3
"""Hold trajectories for feixi_trajectory_controller.

Moves smoothly to a fixed joint pose and stops there (stationary hold via JTC).
Default joint list matches MuJoCo URDF (includes gripper_finger_joint).
Use --seven-dof with feixi_mock_ros2_control (7 joints only).

Optional --resend-period periodically resends the same goal (same setpoint) to
keep tracking a static command; use Ctrl+C to exit.
"""

import argparse
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS_FULL = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
    "gripper_finger_joint",
]

JOINTS_ARM7 = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]

HOME_FULL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
TARGET_FULL = [0.35, -0.45, 0.25, -0.35, 0.5, 0.65, -0.55, 0.35]

HOME_ARM7 = HOME_FULL[:7]
TARGET_ARM7 = TARGET_FULL[:7]

# Default pose to hold at rest (same as previous demo “target”).
HOLD_FULL = list(TARGET_FULL)
HOLD_ARM7 = list(TARGET_ARM7)


def dur(sec: float) -> Duration:
    d = Duration()
    d.sec = int(sec)
    d.nanosec = int(round((sec - int(sec)) * 1e9))
    return d


def parse_positions(s: str, n: int) -> list[float]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if len(parts) != n:
        raise argparse.ArgumentTypeError(f"expected {n} comma-separated numbers, got {len(parts)}")
    out: list[float] = []
    for p in parts:
        try:
            out.append(float(p))
        except ValueError as e:
            raise argparse.ArgumentTypeError(f"invalid float: {p!r}") from e
    return out


def build_hold_goal(joints: list[str], hold: list[float], move_sec: float) -> FollowJointTrajectory.Goal:
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(joints)
    p = JointTrajectoryPoint()
    p.positions = list(hold)
    p.time_from_start = dur(move_sec)
    goal.trajectory.points = [p]
    return goal


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action_name",
        nargs="?",
        default="/feixi_trajectory_controller/follow_joint_trajectory",
        help="FollowJointTrajectory action name",
    )
    parser.add_argument(
        "--seven-dof",
        action="store_true",
        help="Omit gripper (matches feixi_mock stack)",
    )
    parser.add_argument(
        "--move-sec",
        type=float,
        default=5.0,
        help="Seconds to reach the hold pose from the current state (default: 5)",
    )
    parser.add_argument(
        "--positions",
        type=str,
        default=None,
        help=(
            "Comma-separated hold joint positions (radians). "
            "Must be 8 values (or 7 with --seven-dof). Default: built-in hold pose."
        ),
    )
    parser.add_argument(
        "--resend-period",
        type=float,
        default=0.0,
        help=(
            "If > 0, after each successful goal wait this many seconds and resend "
            "the same hold trajectory; Ctrl+C to stop. 0 = send once and exit (default)."
        ),
    )
    args = parser.parse_args()

    if args.move_sec <= 0.0:
        parser.error("--move-sec must be positive")

    action_name = args.action_name
    if args.seven_dof:
        joints = list(JOINTS_ARM7)
        hold = HOLD_ARM7
        n_expected = 7
    else:
        joints = list(JOINTS_FULL)
        hold = HOLD_FULL
        n_expected = 8

    if args.positions is not None:
        hold = parse_positions(args.positions, n_expected)

    rclpy.init()
    node = rclpy.create_node("feixi_demo_trajectory_sender")
    client = ActionClient(node, FollowJointTrajectory, action_name)

    node.get_logger().info(f"Waiting for action server {action_name} ...")
    if not client.wait_for_server(timeout_sec=90.0):
        node.get_logger().error("Action server not available.")
        rclpy.shutdown()
        return 2

    try:
        while rclpy.ok():
            goal = build_hold_goal(joints, hold, args.move_sec)
            node.get_logger().info("Sending hold trajectory goal...")
            send_future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, send_future)
            goal_handle = send_future.result()
            if not goal_handle.accepted:
                node.get_logger().error("Goal rejected.")
                return 3

            node.get_logger().info("Goal accepted; waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            res = result_future.result().result
            node.get_logger().info(f"Done (error_code={res.error_code}).")

            if args.resend_period <= 0.0:
                break

            node.get_logger().info(f"Resend in {args.resend_period} s (Ctrl+C to stop).")
            time.sleep(float(args.resend_period))
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted; exiting.")
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
