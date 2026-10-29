import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


# Make a robot that moves using WASD controls
class KeyboardControl(Node):
    def __init__(self):
        super().__init__("keyboard_control")
        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.create_timer(0.05, self.interpret_key)

        self.get_logger().info("W A S D seperti biasa")

    # Get and return keys
    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            return sys.stdin.read(1)
        return ""

    def interpret_key(self):
        key = self.get_key()

        twist = Twist()

        match key:
            case "w" | "W":
                twist.linear.x = 0.5
            case "a" | "A":
                twist.angular.z = 0.4
            case "s" | "S":
                twist.linear.x = -5.0
            case "d" | "D":
                twist.angular.z = -0.4
            case " ":
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            case _:
                return

        self.cmd_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()