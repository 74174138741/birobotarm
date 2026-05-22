#!/usr/bin/env python3
"""Stream joint position commands to feixi_forward_position_controller (Float64MultiArray).

Use with:
  ros2 launch feixi_mujoco_control feixi_mujoco_ros2_control.launch.py joint_commands:=stream

Default publish rate matches hardware update (see ros2_controllers.yaml update_rate).
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# joint1..7 + gripper_finger_joint (matches feixi_forward_position_controller.yaml)
DEFAULT_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
N_JOINTS = 8


def parse_positions(s: str) -> list[float]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if len(parts) != N_JOINTS:
        raise argparse.ArgumentTypeError(f"expected {N_JOINTS} comma-separated numbers, got {len(parts)}")
    out: list[float] = []
    for p in parts:
        try:
            out.append(float(p))
        except ValueError as e:
            raise argparse.ArgumentTypeError(f"invalid float: {p!r}") from e
    return out


class Streamer(Node):
    def __init__(self, topic: str, hz: float, command: list[float]) -> None:
        super().__init__("feixi_joint_command_stream")
        self._pub = self.create_publisher(Float64MultiArray, topic, 10)
        self._cmd = list(command)
        period = 1.0 / hz if hz > 0.0 else 0.002
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(f"Publishing {len(self._cmd)} positions at ~{hz} Hz on {topic}")

    def _tick(self) -> None:
        msg = Float64MultiArray()
        msg.data = self._cmd
        self._pub.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hz",
        type=float,
        default=500.0,
        help="Publish rate (Hz); should be >= controller_manager update_rate for best tracking (default: 500)",
    )
    parser.add_argument(
        "--topic",
        default="/feixi_forward_position_controller/commands",
        help="ForwardCommandController command topic",
    )
    parser.add_argument(
        "--positions",
        type=parse_positions,
        default=None,
        help=f"Comma-separated radians, {N_JOINTS} values (joint1..7, gripper). Default: built-in pose.",
    )
    args = parser.parse_args()

    if args.hz <= 0.0:
        parser.error("--hz must be positive")

    cmd = list(args.positions) if args.positions is not None else list(DEFAULT_POS)

    rclpy.init()
    node = Streamer(args.topic, args.hz, cmd)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
