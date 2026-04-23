#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog

HELP = '''
JointJog keyboard teleop

  1/q : +joint1 / -joint1
  2/w : +joint2 / -joint2
  3/e : +joint3 / -joint3
  4/r : +joint4 / -joint4
  5/t : +joint5 / -joint5
  6/y : +joint6 / -joint6

  +   : increase speed by 20%
  -   : decrease speed by 20%
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

class JointJogTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_servo_joint_jog')

        self.declare_parameter('joint_command_in_topic', '/servo_node/delta_joint_cmds')
        self.declare_parameter('command_frame', 'world')
        self.declare_parameter('joint_speed', 0.1)
        self.declare_parameter('publish_rate_hz', 30.0)

        self.topic = self.get_parameter('joint_command_in_topic').value
        self.command_frame = self.get_parameter('command_frame').value
        self.joint_speed = float(self.get_parameter('joint_speed').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.publisher_ = self.create_publisher(JointJog, self.topic, 10)

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.current_velocities = [0.0] * 6

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_joint_jog)

        self.get_logger().info(f'Publishing JointJog to {self.topic}')
        self.get_logger().info(f'Command frame: {self.command_frame}')
        self.get_logger().info(f'Joint speed: {self.joint_speed:.3f} rad/s')
        print(HELP)

    def publish_joint_jog(self):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.joint_names = list(self.joint_names)
        msg.velocities = list(self.current_velocities)
        self.publisher_.publish(msg)

    def zero(self):
        self.current_velocities = [0.0] * 6

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
            self.joint_speed *= 1.2
            self.get_logger().info(f'Joint speed increased -> {self.joint_speed:.3f} rad/s')
            return True

        if key == '-':
            self.joint_speed *= 0.8
            self.get_logger().info(f'Joint speed decreased -> {self.joint_speed:.3f} rad/s')
            return True

        mapping = {
            '1': (0, +1.0), 'q': (0, -1.0),
            '2': (1, +1.0), 'w': (1, -1.0),
            '3': (2, +1.0), 'e': (2, -1.0),
            '4': (3, +1.0), 'r': (3, -1.0),
            '5': (4, +1.0), 't': (4, -1.0),
            '6': (5, +1.0), 'y': (5, -1.0),
        }

        if key in mapping:
            idx, sign = mapping[key]
            self.zero()
            self.current_velocities[idx] = sign * self.joint_speed

        return True

def main():
    rclpy.init()
    node = JointJogTeleop()

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
            node.publish_joint_jog()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
