#!/usr/bin/env python3
"""
Custom keyboard teleop dengan 2 mode
LOCAL -> Spti joystik/video game
GLOBAL -> Atau absolute, mengikuti axis x dan y yang tetap tanpa melihat arah robot

M -> Ganti Mode
Space -> Stop
"""

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

# Units pakai m, s, rad, dan SI lainnya
SPEED = 0.5
KEY_TIMEOUT = 0.3
STEP_DISTANCE = 0.5
CONTROL_HZ = 20.0
DISTANCE_TOLERANCE = 0.02

LOCAL_KEYS = {
    "w": (SPEED, 0.0),
    "s": (-SPEED, 0.0),
    "a": (0.0, SPEED),
    "d": (0.0, -SPEED),
}

GLOBAL_KEYS = {
    "w": (0.0, STEP_DISTANCE),
    "s": (0.0, -STEP_DISTANCE),
    "a": (-STEP_DISTANCE, 0.0),
    "d": (STEP_DISTANCE, 0.0),
}


def quaternion_converter(q):
    # Reference: https://tom-howard.github.io/ros2/course/part2/#pose
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class KeyboardControl(Node):
    def __init__(self):
        super().__init__("keyboard_control")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.mode_pub = self.create_publisher(String, "/robot_mode", latched)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.mode = "Gkibak"

        # ODOM pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ODOM target GLOBAL mode
        self.target = None

        # Velocity
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        self.last_key_time = time.monotonic()

        # Terminal input
        self.fd = sys.stdin.fileno()
        self.old_term_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.publish_mode()
        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)

        self.get_logger().info(
            "W: forward | S: backward | A: left | D: right | SPACE: stop | M: toggle mode"
        )

    def publish_mode(self):
        self.mode_pub.publish(String(data=self.mode))

    def read_key(self):
        latest = ""
        while select.select([self.fd], [], [], 0)[0]:
            char = os.read(self.fd, 1).decode(errors="ignore")
            if not char:
                break
            latest = char
        return latest

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_converter(msg.pose.pose.orientation)

    def stop(self):
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        self.target = None

    # Key Handling
    def handle_key(self, key, now):
        key = key.lower()

        if key == "m":
            self.mode = "GLOBAL" if self.mode == "LOCAL" else "LOCAL"
            self.stop()
            self.publish_mode()
            self.get_logger().info(f"Mode changed to: {self.mode}")
        elif key == " ":
            self.stop()
        elif self.mode == "LOCAL" and key in LOCAL_KEYS:
            self.vel_x, self.vel_yaw = LOCAL_KEYS[key]
            self.last_key_time = now
        elif self.mode == "GLOBAL" and key in GLOBAL_KEYS and self.target is None:
            # Ignore new steps until the current one finishes.
            dx, dy = GLOBAL_KEYS[key]
            self.target = (self.x + dx, self.y + dy)

    
    #### PERGERAKAN

    def drive_to_target(self):
        """Head toward self.target at constant speed, then stop on arrival."""
        dx = self.target[0] - self.x
        dy = self.target[1] - self.y
        distance = math.hypot(dx, dy)

        if distance <= DISTANCE_TOLERANCE:
            self.stop()
            return

        # Desired velocity in the odom frame...
        vx_world = SPEED * dx / distance
        vy_world = SPEED * dy / distance

        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        self.vel_x = vx_world * cos_yaw + vy_world * sin_yaw
        self.vel_y = -vx_world * sin_yaw + vy_world * cos_yaw
        self.vel_yaw = 0.0

    def control_loop(self):
        now = time.monotonic()

        key = self.read_key()
        if key:
            self.handle_key(key, now)

        if self.mode == "LOCAL":
            if now - self.last_key_time > KEY_TIMEOUT:
                self.stop()
        elif self.target is not None:
            self.drive_to_target()

        self.publish_velocity()

    def publish_velocity(self):
        cmd = Twist()
        cmd.linear.x = self.vel_x
        cmd.linear.y = self.vel_y
        cmd.angular.z = self.vel_yaw
        self.cmd_pub.publish(cmd)

    #### BERSIH2 AKHIR

    def close(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term_settings)
        self.stop()
        self.publish_velocity()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = KeyboardControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.close()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
