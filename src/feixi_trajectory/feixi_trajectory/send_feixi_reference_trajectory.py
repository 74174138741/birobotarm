#!/usr/bin/env python3
from __future__ import annotations

"""Send a multi-point joint reference to feixi_trajectory_controller (Pinocchio test).

Reference trajectory (joint1..7, gripper_finger_joint), radians:
  Point A  t=0.0 s  [ 0.00, -0.50,  0.00,  1.20,  0.00,  0.80,  0.00, 0.05 ]
  Point B  t=4.0 s  [ 0.20, -0.60,  0.15,  1.00,  0.10,  0.90, -0.20, 0.10 ]
  Point C  t=8.0 s  [ 0.35, -0.45,  0.25,  0.95,  0.20,  0.85, -0.35, 0.15 ]

Use with dynamics_mode:=trajectory and expose_joint_vel_acc_commands:=true so hardware
receives position, velocity, acceleration from the controller spline.

Example:
  ros2 run feixi_trajectory send_feixi_reference_trajectory
"""

import argparse
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
    "gripper_finger_joint",
]

# Start matches default init_pose_q in feixi.urdf.xacro
P_REF_A = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
P_REF_B = [0.2, -0.6, 0.15, 1.0, 0.1, 0.9, -0.2, 0.10]
P_REF_C = [0.35, -0.45, 0.25, 0.95, 0.2, 0.85, -0.35, 0.15]

T_AB = 4.0
T_BC = 4.0


def dur(sec: float) -> Duration:
    d = Duration()
    d.sec = int(sec)
    d.nanosec = int(round((sec - d.sec) * 1e9))

    # 处理溢出
    if d.nanosec >= 1_000_000_000:
        d.sec += 1
        d.nanosec -= 1_000_000_000

    return d


def build_goal() -> FollowJointTrajectory.Goal:
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(JOINTS)
    p0 = JointTrajectoryPoint()
    p0.positions = list(P_REF_A)
    p0.velocities = [0.0] * 8
    p0.accelerations = [0.0] * 8
    p0.time_from_start = dur(0.0)

    p1 = JointTrajectoryPoint()
    p1.positions = list(P_REF_B)
    p1.velocities = [0.0] * 8
    p1.accelerations = [0.0] * 8
    p1.time_from_start = dur(T_AB)

    p2 = JointTrajectoryPoint()
    p2.positions = list(P_REF_C)
    p2.velocities = [0.0] * 8
    p2.accelerations = [0.0] * 8
    p2.time_from_start = dur(T_AB + T_BC)

    goal.trajectory.points = [p0, p1, p2]
    return goal


def print_reference_table() -> None:
    hdr = " | ".join(JOINTS)
    print("Reference trajectory (rad)\n")
    print(f"{'point':8} | {'t(s)':>6} | {hdr}")
    print("-" * (20 + len(hdr)))
    print(f"{'A':8} | {0.0:6.1f} | " + ", ".join(f"{x:6.3f}" for x in P_REF_A))
    print(f"{'B':8} | {T_AB:6.1f} | " + ", ".join(f"{x:6.3f}" for x in P_REF_B))
    print(f"{'C':8} | {T_AB + T_BC:6.1f} | " + ", ".join(f"{x:6.3f}" for x in P_REF_C))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action_name",
        nargs="?",
        default="/feixi_trajectory_controller/follow_joint_trajectory",
        help="FollowJointTrajectory action server",
    )
    parser.add_argument(
        "--print-only",
        action="store_true",
        help="Print the reference table and exit (no ROS).",
    )
    parser.add_argument(
        "--wait-server-sec",
        type=float,
        default=120.0,
        help="Seconds to wait for the action server (default: 120).",
    )
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    if args.print_only:
        print_reference_table()
        return 0

    rclpy.init()
    node = rclpy.create_node("feixi_reference_trajectory_sender")
    client = ActionClient(node, FollowJointTrajectory, args.action_name)

    node.get_logger().info("Waiting for action %s ..." % args.action_name)
    if not client.wait_for_server(timeout_sec=args.wait_server_sec):
        node.get_logger().error("Action server not available.")
        rclpy.shutdown()
        return 2

    goal = build_goal()
    node.get_logger().info("Sending 3-point reference trajectory (0s, %.1fs, %.1fs)." % (T_AB, T_AB + T_BC))
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    gh = send_future.result()
    if gh is None or not gh.accepted:
        node.get_logger().error("Goal rejected.")
        rclpy.shutdown()
        return 3

    node.get_logger().info("Goal accepted; waiting for completion...")
    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    res = result_future.result().result
    ec = int(res.error_code)
    ok = ec == FollowJointTrajectory.Result.SUCCESSFUL
    node.get_logger().info("Finished (error_code=%d, successful=%s)." % (ec, ok))
    # Brief pause so last joint_states are published before teardown (optional).
    time.sleep(0.2)
    rclpy.shutdown()
    return 0 if ok else 4


if __name__ == "__main__":
    sys.exit(main())
