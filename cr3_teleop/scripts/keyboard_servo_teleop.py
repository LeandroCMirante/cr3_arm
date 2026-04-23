#!/usr/bin/env python3
"""
Keyboard teleop for MoveIt Servo using TwistStamped commands.

Typical flow:
1. Start your robot + MoveIt Servo.
2. Switch Servo to TWIST command mode if needed.
3. Run this node.
4. Use the keyboard to jog the end effector in XYZ and orientation.

Default topic/frame:
- Topic: /servo_node/delta_twist_cmds
- Frame: world

You can override them with ROS parameters:
- cartesian_command_in_topic
- command_frame
- linear_speed
- angular_speed
- publish_rate_hz

Suggested keys:
  Movement (XYZ):
    w/s : +X / -X
    a/d : +Y / -Y
    r/f : +Z / -Z

  Orientation (roll/pitch/yaw):
    i/k : +roll / -roll
    j/l : +pitch / -pitch
    u/o : +yaw / -yaw

  Speed:
    +   : increase speeds by 20%
    -   : decrease speeds by 20%

  Other:
    space : send zero twist
    h     : print help
    q     : quit

Notes:
- This node publishes a TwistStamped continuously at publish_rate_hz.
- Each key press updates the commanded twist; space zeros it immediately.
- For now, keep command_frame equal to your MoveIt planning frame unless you
  intentionally configured Servo otherwise.
"""

import select
import sys
import termios
import tty
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


@dataclass
class TwistCommand:
    lx: float = 0.0
    ly: float = 0.0
    lz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0

    def zero(self) -> None:
        self.lx = self.ly = self.lz = 0.0
        self.ax = self.ay = self.az = 0.0


HELP_TEXT = """
Keyboard teleop for MoveIt Servo

Movement (linear):
  w/s : +X / -X
  a/d : +Y / -Y
  r/f : +Z / -Z

Orientation (angular):
  i/k : +roll / -roll
  j/l : +pitch / -pitch
  u/o : +yaw / -yaw

Speed:
  +   : increase linear and angular speed by 20%
  -   : decrease linear and angular speed by 20%

Other:
  space : zero command
  h     : help
  q     : quit
"""


class KeyboardServoTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_servo_teleop")

        self.declare_parameter("cartesian_command_in_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("command_frame", "world")
        self.declare_parameter("linear_speed", 0.10)   # m/s
        self.declare_parameter("angular_speed", 0.50)  # rad/s
        self.declare_parameter("publish_rate_hz", 30.0)

        self.topic = self.get_parameter("cartesian_command_in_topic").value
        self.command_frame = self.get_parameter("command_frame").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.publisher_ = self.create_publisher(TwistStamped, self.topic, 10)
        self.current_cmd = TwistCommand()

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_twist)

        self.get_logger().info(f"Publishing TwistStamped to {self.topic}")
        self.get_logger().info(f"Command frame: {self.command_frame}")
        self.get_logger().info(
            f"Linear speed: {self.linear_speed:.3f} m/s | Angular speed: {self.angular_speed:.3f} rad/s"
        )
        print(HELP_TEXT)

    def publish_twist(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame

        msg.twist.linear.x = self.current_cmd.lx
        msg.twist.linear.y = self.current_cmd.ly
        msg.twist.linear.z = self.current_cmd.lz
        msg.twist.angular.x = self.current_cmd.ax
        msg.twist.angular.y = self.current_cmd.ay
        msg.twist.angular.z = self.current_cmd.az

        self.publisher_.publish(msg)

    def set_linear(self, x: float, y: float, z: float) -> None:
        self.current_cmd.lx = x
        self.current_cmd.ly = y
        self.current_cmd.lz = z

    def set_angular(self, x: float, y: float, z: float) -> None:
        self.current_cmd.ax = x
        self.current_cmd.ay = y
        self.current_cmd.az = z

    def handle_key(self, key: str) -> bool:
        # Return False to quit
        if key == "q":
            self.current_cmd.zero()
            return False

        if key == " ":
            self.current_cmd.zero()
            return True

        if key == "h":
            print(HELP_TEXT)
            return True

        if key == "+":
            self.linear_speed *= 1.2
            self.angular_speed *= 1.2
            self.get_logger().info(
                f"Speeds increased -> linear={self.linear_speed:.3f}, angular={self.angular_speed:.3f}"
            )
            return True

        if key == "-":
            self.linear_speed *= 0.8
            self.angular_speed *= 0.8
            self.get_logger().info(
                f"Speeds decreased -> linear={self.linear_speed:.3f}, angular={self.angular_speed:.3f}"
            )
            return True

        # Reset one axis family before applying new command
        if key in ("w", "s", "a", "d", "r", "f"):
            self.set_linear(0.0, 0.0, 0.0)
        if key in ("i", "k", "j", "l", "u", "o"):
            self.set_angular(0.0, 0.0, 0.0)

        # Linear XYZ
        if key == "w":
            self.current_cmd.lx = self.linear_speed
        elif key == "s":
            self.current_cmd.lx = -self.linear_speed
        elif key == "a":
            self.current_cmd.ly = self.linear_speed
        elif key == "d":
            self.current_cmd.ly = -self.linear_speed
        elif key == "r":
            self.current_cmd.lz = self.linear_speed
        elif key == "f":
            self.current_cmd.lz = -self.linear_speed

        # Angular RPY
        elif key == "i":
            self.current_cmd.ax = self.angular_speed
        elif key == "k":
            self.current_cmd.ax = -self.angular_speed
        elif key == "j":
            self.current_cmd.ay = self.angular_speed
        elif key == "l":
            self.current_cmd.ay = -self.angular_speed
        elif key == "u":
            self.current_cmd.az = self.angular_speed
        elif key == "o":
            self.current_cmd.az = -self.angular_speed

        return True


class KeyboardReader:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout: float = 0.1):
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        if readable:
            return sys.stdin.read(1)
        return None


def main() -> None:
    rclpy.init()
    node = KeyboardServoTeleop()

    try:
        with KeyboardReader() as reader:
            running = True
            while running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                key = reader.read_key(timeout=0.05)
                if key is not None:
                    running = node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.current_cmd.zero()
        # Publish one last zero command
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.01)
            node.publish_twist()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
