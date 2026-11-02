#!/usr/bin/env python3
"""
Custom keyboard teleop dengan 2 mode
LOCAL -> Seperti joystik/video game
GLOBAL -> Atau absolute, mengikuti axis x dan y yang tetap tanpa melihat arah robot

M -> Ganti Mode
Space -> Stop

Input lain (dari Foxglove):
/cmd_vel_global          -> Twist kecepatan dalam frame odom (world)
/move_base_simple/goal   -> PoseStamped target posisi
"""

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
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

KP = 1.5  # gain proporsional, makin besar makin agresif
MIN_SPEED = 0.05  # supaya tidak merayap di dekat target

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

        self.was_moving = False
        self.mode_pub = self.create_publisher(String, "/robot_mode", latched)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Twist, "/cmd_vel_global", self.global_cmd_callback, 10)
        self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self.goal_callback, 10
        )

        self.mode = "LOCAL"

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

    def set_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            self.publish_mode()
            self.get_logger().info(f"Mode changed to: {self.mode}")

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

    def world_to_body(self, vx_world, vy_world):
        """Putar kecepatan frame odom ke frame robot."""
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        vx = vx_world * cos_yaw + vy_world * sin_yaw
        vy = -vx_world * sin_yaw + vy_world * cos_yaw
        return vx, vy

    # Input dari Foxglove
    def global_cmd_callback(self, msg):
        """Kecepatan frame world dari Teleop panel (/cmd_vel_global)."""
        self.set_mode("GLOBAL")
        self.vel_x, self.vel_y = self.world_to_body(msg.linear.x, msg.linear.y)
        self.vel_yaw = msg.angular.z
        self.target = None
        self.last_key_time = time.monotonic()

    def goal_callback(self, msg):
        """Target posisi dari klik di panel 3D Foxglove."""
        self.set_mode("GLOBAL")
        self.target = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f"Goal ({msg.header.frame_id}): {self.target[0]:.2f}, {self.target[1]:.2f}"
        )

    # Key Handling
    def handle_key(self, key, now):
        key = key.lower()

        if key == "m":
            self.stop()
            self.set_mode("GLOBAL" if self.mode == "LOCAL" else "LOCAL")
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
        """Head toward self.target, slowing down as it gets close."""
        dx = self.target[0] - self.x
        dy = self.target[1] - self.y
        distance = math.hypot(dx, dy)

        if distance <= DISTANCE_TOLERANCE:
            self.stop()
            return

        # Proportional control: pelan saat dekat, maksimal SPEED
        speed = max(MIN_SPEED, min(SPEED, KP * distance))
        vx_world = speed * dx / distance
        vy_world = speed * dy / distance

        self.vel_x, self.vel_y = self.world_to_body(vx_world, vy_world)
        self.vel_yaw = 0.0

    def control_loop(self):
        now = time.monotonic()

        key = self.read_key()
        if key:
            self.handle_key(key, now)

        if self.target is not None:
            self.drive_to_target()
        elif now - self.last_key_time > KEY_TIMEOUT:
            self.stop()

        self.publish_velocity()

    def publish_velocity(self):
        moving = any((self.vel_x, self.vel_y, self.vel_yaw))
        if not moving and not self.was_moving:
            return  # idle: stay quiet so other publishers can drive
        self.was_moving = moving

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
