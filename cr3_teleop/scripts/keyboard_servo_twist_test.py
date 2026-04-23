#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

HELP = '''
Twist keyboard teleop

  w/s : +X / -X
  a/d : +Y / -Y
  r/f : +Z / -Z

  i/k : +roll / -roll
  j/l : +pitch / -pitch
  u/o : +yaw / -yaw

  +   : increase speeds by 20%
  -   : decrease speeds by 20%
  space : zero command
  h   : help
  x   : quit
'''

class KeyboardReader:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout=0.05):
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        if readable:
            return sys.stdin.read(1)
        return None

class TwistTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_servo_twist_test')

        self.declare_parameter('cartesian_command_in_topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('command_frame', 'world')
        self.declare_parameter('linear_speed', 0.01)
        self.declare_parameter('angular_speed', 0.05)
        self.declare_parameter('publish_rate_hz', 30.0)

        self.topic = self.get_parameter('cartesian_command_in_topic').value
        self.command_frame = self.get_parameter('command_frame').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.publisher_ = self.create_publisher(TwistStamped, self.topic, 10)

        self.lx = self.ly = self.lz = 0.0
        self.ax = self.ay = self.az = 0.0

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_twist)

        self.get_logger().info(f'Publishing TwistStamped to {self.topic}')
        self.get_logger().info(f'Command frame: {self.command_frame}')
        self.get_logger().info(
            f'Linear speed: {self.linear_speed:.3f} m/s | Angular speed: {self.angular_speed:.3f} rad/s'
        )
        print(HELP)

    def zero(self):
        self.lx = self.ly = self.lz = 0.0
        self.ax = self.ay = self.az = 0.0

    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.twist.linear.x = self.lx
        msg.twist.linear.y = self.ly
        msg.twist.linear.z = self.lz
        msg.twist.angular.x = self.ax
        msg.twist.angular.y = self.ay
        msg.twist.angular.z = self.az
        self.publisher_.publish(msg)

    def handle_key(self, key):
        if key == 'x':
            self.zero()
            return False

        if key == ' ':
            self.zero()
            return True

        if key == 'h':
            print(HELP)
            return True

        if key == '+':
            self.linear_speed *= 1.2
            self.angular_speed *= 1.2
            self.get_logger().info(
                f'Speeds increased -> linear={self.linear_speed:.3f}, angular={self.angular_speed:.3f}'
            )
            return True

        if key == '-':
            self.linear_speed *= 0.8
            self.angular_speed *= 0.8
            self.get_logger().info(
                f'Speeds decreased -> linear={self.linear_speed:.3f}, angular={self.angular_speed:.3f}'
            )
            return True

        if key in ('w', 's', 'a', 'd', 'r', 'f'):
            self.lx = self.ly = self.lz = 0.0
        if key in ('i', 'k', 'j', 'l', 'u', 'o'):
            self.ax = self.ay = self.az = 0.0

        if key == 'w':
            self.lx = self.linear_speed
        elif key == 's':
            self.lx = -self.linear_speed
        elif key == 'a':
            self.ly = self.linear_speed
        elif key == 'd':
            self.ly = -self.linear_speed
        elif key == 'r':
            self.lz = self.linear_speed
        elif key == 'f':
            self.lz = -self.linear_speed
        elif key == 'i':
            self.ax = self.angular_speed
        elif key == 'k':
            self.ax = -self.angular_speed
        elif key == 'j':
            self.ay = self.angular_speed
        elif key == 'l':
            self.ay = -self.angular_speed
        elif key == 'u':
            self.az = self.angular_speed
        elif key == 'o':
            self.az = -self.angular_speed

        return True

def main():
    rclpy.init()
    node = TwistTeleop()

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
        node.zero()
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.01)
            node.publish_twist()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
