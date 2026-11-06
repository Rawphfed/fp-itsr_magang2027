#!/usr/bin/env python3

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


# ==========================================
# PID CONTROLLER CLASS
# ==========================================
def wrap_angle(angle):
    """Wraps angle between -pi and pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    """Clamps value within [-limit, limit]."""
    return max(-limit, min(limit, value))


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, o_lim=math.inf, i_lim=math.inf, wrap=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.o_lim = abs(o_lim)
        self.i_lim = abs(i_lim)
        self.wrap = wrap
        self.reset()

    def reset(self):
        self.i_term = 0.0
        self.prev_measurement = None

    def _diff(self, a, b):
        diff = a - b
        return wrap_angle(diff) if self.wrap else diff

    def update(self, setpoint, measurement, dt):
        error = self._diff(setpoint, measurement)
        valid_dt = dt > 0.0

        p_term = self.kp * error

        d_term = 0.0
        if valid_dt and self.prev_measurement is not None:
            rate = self._diff(measurement, self.prev_measurement) / dt
            d_term = -self.kd * rate
        self.prev_measurement = measurement

        new_i = self.i_term
        if valid_dt:
            new_i = clamp(self.i_term + self.ki * error * dt, self.i_lim)

        raw = p_term + new_i + d_term

        pushing_into_saturation = abs(raw) > self.o_lim and error * raw > 0.0
        if not pushing_into_saturation:
            self.i_term = new_i

        output = p_term + self.i_term + d_term
        return clamp(output, self.o_lim)


# ==========================================
# CONFIGURATION & CONSTANTS
# ==========================================
SPEED = 1.0
ANGULAR_SPEED = 0.6
DIST_TOL = 0.02
YAW_TOL = 0.05
KEY_TIMEOUT = 0.3
CONTROL_HZ = 20.0

GOAL_SPEED = 0.8
GOAL_YAW_SPEED = 0.8

# PID Gains
KP_XY = 5.0
KI_XY = 0.1
KD_XY = 0.1
I_LIMIT_XY = 0.2

YAW_KP = 1.0
YAW_KI = 0.05
YAW_KD = 0.0
YAW_I_LIMIT = 0.2

# Keymaps
LOCAL_KEYS = {
    "w": (SPEED, 0.0),
    "s": (-SPEED, 0.0),
    "a": (0.0, SPEED),
    "d": (0.0, -SPEED),
}

GLOBAL_KEYS = {
    "w": (0.0, SPEED, 0.0),
    "s": (0.0, -SPEED, 0.0),
    "a": (-SPEED, 0.0, 0.0),
    "d": (SPEED, 0.0, 0.0),
    "q": (0.0, 0.0, ANGULAR_SPEED),
    "e": (0.0, 0.0, -ANGULAR_SPEED),
}


def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ==========================================
# MAIN TR CONTROL NODE
# ==========================================
class KeyboardController(Node):
    def __init__(self):
        super().__init__("keyboard_controller")

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers (Compatible with WebUI / Foxglove Studio)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "/robot_mode", latched_qos)

        # Subscriptions
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Twist, "/cmd_vel_global", self.global_cmd_callback, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

        self.mode = "LOCAL"
        self.was_moving = False

        # Semi-Autonomous PID Controllers
        self.pid_x = PID(KP_XY, KI_XY, KD_XY, o_lim=GOAL_SPEED, i_lim=I_LIMIT_XY)
        self.pid_y = PID(KP_XY, KI_XY, KD_XY, o_lim=GOAL_SPEED, i_lim=I_LIMIT_XY)
        self.pid_yaw = PID(
            YAW_KP,
            YAW_KI,
            YAW_KD,
            o_lim=GOAL_YAW_SPEED,
            i_lim=YAW_I_LIMIT,
            wrap=True,
        )

        # State Variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.target = None  # (x, y, yaw)
        self.global_vel_x = 0.0
        self.global_vel_y = 0.0
        self.global_vel_yaw = 0.0

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0

        self.last_loop_time = time.monotonic()
        self.last_key_time = time.monotonic()

        # Custom Low-level Terminal Key Reader (Criteria b)
        self.fd = sys.stdin.fileno()
        self.old_term_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.publish_mode()
        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)

        self.get_logger().info(
            "TR Controller Active!\n"
            "   - WASD: Directional movement\n"
            "   - Q/E: Rotation (Global mode)\n"
            "   - M: Toggle GLOBAL / LOCAL mode\n"
            "   - SPACE: Emergency stop\n"
            "   - P: Print current pose\n"
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
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def stop(self):
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        self.global_vel_x = 0.0
        self.global_vel_y = 0.0
        self.global_vel_yaw = 0.0
        self.target = None
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_yaw.reset()

    def world_to_body(self, vx_world, vy_world):
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        vx = vx_world * cos_yaw + vy_world * sin_yaw
        vy = -vx_world * sin_yaw + vy_world * cos_yaw
        return vx, vy

    def global_cmd_callback(self, msg):
        """Allows direct teleoperation in global coordinates via WebUI/Foxglove."""
        self.set_mode("GLOBAL")
        self.global_vel_x = msg.linear.x
        self.global_vel_y = msg.linear.y
        self.global_vel_yaw = msg.angular.z
        self.target = None
        self.last_key_time = time.monotonic()

    def goal_callback(self, msg):
        """Processes semi-autonomous position target (Criteria e)."""
        self.set_mode("GLOBAL")
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_yaw.reset()

        self.target = (
            msg.pose.position.x,
            msg.pose.position.y,
            quaternion_to_yaw(msg.pose.orientation),
        )
        self.last_key_time = time.monotonic()
        self.get_logger().info(f"New Target Received: {self.target}")

    def handle_key(self, key, now):
        key = key.lower()

        if key == "m":
            self.stop()
            self.set_mode("GLOBAL" if self.mode == "LOCAL" else "LOCAL")

        elif key == " ":
            self.stop()

        elif key == "p":
            self.get_logger().info(
                f"Position -> X: {self.x:.3f}, Y: {self.y:.3f}, Yaw: {self.yaw:.3f}"
            )

        elif self.mode == "LOCAL" and key in LOCAL_KEYS:
            self.vel_x, self.vel_yaw = LOCAL_KEYS[key]
            self.vel_y = 0.0
            self.last_key_time = now

        elif self.mode == "GLOBAL" and key in GLOBAL_KEYS:
            self.target = None
            (
                self.global_vel_x,
                self.global_vel_y,
                self.global_vel_yaw,
            ) = GLOBAL_KEYS[key]
            self.last_key_time = now

    def drive_to_target(self, dt):
        """Semi-autonomous PID control targeting drive (Criteria e)."""
        dx = self.target[0] - self.x
        dy = self.target[1] - self.y
        yaw_error = wrap_angle(self.target[2] - self.yaw)

        if math.hypot(dx, dy) <= DIST_TOL and abs(yaw_error) <= YAW_TOL:
            self.get_logger().info("Target reached!")
            self.stop()
            return

        vx_world = self.pid_x.update(self.target[0], self.x, dt)
        vy_world = self.pid_y.update(self.target[1], self.y, dt)

        speed = math.hypot(vx_world, vy_world)
        if speed > SPEED:
            vx_world *= SPEED / speed
            vy_world *= SPEED / speed

        self.vel_x, self.vel_y = self.world_to_body(vx_world, vy_world)
        self.vel_yaw = self.pid_yaw.update(self.target[2], self.yaw, dt)

    def control_loop(self):
        now = time.monotonic()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        key = self.read_key()

        if key:
            self.handle_key(key, now)
        elif self.target is not None:
            self.drive_to_target(dt)
        elif now - self.last_key_time > KEY_TIMEOUT:
            self.stop()
        elif self.mode == "GLOBAL":
            self.vel_x, self.vel_y = self.world_to_body(
                self.global_vel_x, self.global_vel_y
            )
            self.vel_yaw = self.global_vel_yaw

        self.publish_velocity()

    def publish_velocity(self):
        moving = any((self.vel_x, self.vel_y, self.vel_yaw))

        if not moving and not self.was_moving:
            return
        self.was_moving = moving

        cmd = Twist()
        cmd.linear.x = self.vel_x
        cmd.linear.y = self.vel_y
        cmd.angular.z = self.vel_yaw
        self.cmd_pub.publish(cmd)

    def close(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term_settings)
        self.stop()
        self.publish_velocity()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = KeyboardController()
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
