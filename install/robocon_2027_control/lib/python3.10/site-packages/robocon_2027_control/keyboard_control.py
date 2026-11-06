#!/usr/bin/env python3

import rclpy
import math
import os
import select
import sys
import termios
import time
import tty


from robocon_2027_control.pid import PID, wrap_angle
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

## MOVEMENT CONFIG
SPEED = 1.0
ANGULAR_SPEED = 1.0
DIST_TOL = 0.02
YAW_TOL = 0.05
KEY_TIMEOUT = 0.3
CONTROL_HZ = 20.0
MIN_SPEED = 0.05

## STUCK / REALIGN CONFIG
STUCK_TIMEOUT = 2.0  
DIST_PROGRESS = 0.01 
YAW_PROGRESS = 0.05  
REALIGN_TIMEOUT = 2.0
YAW_SLOWDOWN = math.pi / 3 

## PID CONFIG
# X Y
KP = 5
KI = 0.1
KD = 0.1
I_LIMIT = 0.2
# YAW
YAW_KP = 2
YAW_KI = 0.0
YAW_KD = 0.0
YAW_I_LIMIT = 0.2

## KEY MAPPING
LOCAL_KEYS = {
    "w": (SPEED, 0.0),
    "s": (-SPEED, 0.0),
    "a": (0.0, ANGULAR_SPEED),
    "d": (0.0, -ANGULAR_SPEED),
}

GLOBAL_KEYS = {
    "w": (0.0, SPEED, 0.0),
    "s": (0.0, -SPEED, 0.0),
    "a": (-SPEED, 0.0, 0.0),
    "d": (SPEED, 0.0, 0.0),
    "q": (0.0, 0.0, ANGULAR_SPEED),
    "e": (0.0, 0.0, -ANGULAR_SPEED),
}


def quaternion_converter(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class KeyboardControl(Node):
    def __init__(self):
        super().__init__("keyboard_control")

        ## SETUP
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        ## PUB SUBS
        self.was_moving = False
        self.mode_pub = self.create_publisher(String, "/robot_mode", latched)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Twist, "/cmd_vel_global", self.global_cmd_callback, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

        self.mode = "LOCAL"

        ## PID SETUP
        self.pid_y = PID(KP, KI, KD, o_lim=SPEED, i_lim=I_LIMIT)
        self.pid_x = PID(KP, KI, KD, o_lim=SPEED, i_lim=I_LIMIT)
        self.pid_yaw = PID(
            YAW_KP,
            YAW_KI,
            YAW_KD,
            o_lim=ANGULAR_SPEED,
            i_lim=YAW_I_LIMIT,
            yaw=True,
        )
        self.last_loop_time = time.monotonic()

        ## PROGRESS CHECKER
        self.progress_t = time.monotonic()
        self.best_dist = float("inf")
        self.best_yaw_err = float("inf")
        self.realigning = False
        self.realign_t = 0.0

        ## POSE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        ## VELOC
        # GLOBAL
        self.target = None  # (x, y, yaw)
        self.global_vel_x = 0.0
        self.global_vel_y = 0.0
        self.global_vel_yaw = 0.0
        # LOCAL
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0

        # TICKS AND OTHERS
        self.last_key_time = time.monotonic()

        self.fd = sys.stdin.fileno()
        self.old_term_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.publish_mode()
        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)

        self.get_logger().info(
            "W/A/S/D: move | Q/E: rotate (GLOBAL only) | "
            "P: print position | SPACE: stop | M: toggle mode"
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

    # GET FROM /ODOM
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_converter(msg.pose.pose.orientation)

    def stop(self):
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        self.global_vel_x = 0.0
        self.global_vel_y = 0.0
        self.global_vel_yaw = 0.0
        self.target = None
        self.realigning = False
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_yaw.reset()

    ########### FOR FOXGLOVE ###########
    def world_to_body(self, vx_world, vy_world):
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        vx = vx_world * cos_yaw + vy_world * sin_yaw
        vy = -vx_world * sin_yaw + vy_world * cos_yaw
        return vx, vy

    def global_cmd_callback(self, msg):
        self.set_mode("GLOBAL")
        self.global_vel_x = msg.linear.x
        self.global_vel_y = msg.linear.y
        self.global_vel_yaw = msg.angular.z
        self.target = None
        self.realigning = False
        self.last_key_time = time.monotonic()

    def goal_callback(self, msg):
        now = time.monotonic()
        self.progress_t = now
        self.best_dist = float("inf")
        self.best_yaw_err = float("inf")
        self.realigning = False

        self.set_mode("GLOBAL")
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_yaw.reset()

        # A goal sent without orientation (all-zero quaternion) keeps the
        # current heading instead of snapping to world yaw = 0.
        q = msg.pose.orientation
        if q.x == q.y == q.z == q.w == 0.0:
            goal_yaw = self.yaw
        else:
            goal_yaw = quaternion_converter(q)

        self.target = (msg.pose.position.x, msg.pose.position.y, goal_yaw)
        self.last_key_time = now
        self.get_logger().info(f"Target: {self.target}")

    ########### FOR FOXGLOVE ###########

    def handle_key(self, key, currentTime):
        key = key.lower()

        if key == "m":
            self.stop()
            self.set_mode("GLOBAL" if self.mode == "LOCAL" else "LOCAL")

        elif key == " ":
            self.stop()

        elif key == "p":
            self.get_logger().info(
                f"Position: {self.x:.3f}, {self.y:.3f}, yaw {self.yaw:.3f}"
            )

        elif self.mode == "LOCAL" and key in LOCAL_KEYS:
            self.vel_x, self.vel_yaw = LOCAL_KEYS[key]
            self.last_key_time = currentTime

        elif self.mode == "GLOBAL" and key in GLOBAL_KEYS:
            self.target = None
            self.realigning = False
            (
                self.global_vel_x,
                self.global_vel_y,
                self.global_vel_yaw,
            ) = GLOBAL_KEYS[key]
            self.last_key_time = currentTime

    ## MAIN DRIVER
    def drive_to_target(self, dt):
        now = time.monotonic()
        dx = self.target[0] - self.x
        dy = self.target[1] - self.y
        yaw_error = wrap_angle(self.target[2] - self.yaw)
        dist = math.hypot(dx, dy)

        if self.realigning:
            self.vel_x = 0.0
            self.vel_y = 0.0
            self.vel_yaw = self.pid_yaw.update(self.target[2], self.yaw, dt)
            if abs(yaw_error) <= YAW_TOL or now - self.realign_t > REALIGN_TIMEOUT:
                self.stop()
            return

        if dist <= DIST_TOL and abs(yaw_error) <= YAW_TOL:
            self.stop()
            return

        yaw_abs = abs(yaw_error)
        if (
            dist < self.best_dist - DIST_PROGRESS
            or yaw_abs < self.best_yaw_err - YAW_PROGRESS
        ):
            self.best_dist = min(self.best_dist, dist)
            self.best_yaw_err = min(self.best_yaw_err, yaw_abs)
            self.progress_t = now
        elif now - self.progress_t > STUCK_TIMEOUT:
            self.get_logger().warn("Blocked, realigning heading")
            self.realigning = True
            self.realign_t = now
            self.pid_yaw.reset()
            return

        vx_world = self.pid_x.update(self.target[0], self.x, dt)
        vy_world = self.pid_y.update(self.target[1], self.y, dt)

        if yaw_abs > YAW_SLOWDOWN:   
            vx_world = vy_world = 0.0

        speed = math.hypot(vx_world, vy_world)
        if speed > SPEED:
            vx_world *= SPEED / speed
            vy_world *= SPEED / speed

        self.vel_x, self.vel_y = self.world_to_body(vx_world, vy_world)
        self.vel_yaw = self.pid_yaw.update(self.target[2], self.yaw, dt)
        
        self.get_logger().info(
            f"yaw={self.yaw:.2f} target={self.target[2]:.2f} "
            f"err={yaw_error:.2f} cmd_w={self.vel_yaw:.2f}",
            throttle_duration_sec=0.2,
        )

    ## MAIN LOOP
    def control_loop(self):
        currentTime = time.monotonic()
        dt = currentTime - self.last_loop_time
        self.last_loop_time = currentTime
        key = self.read_key()

        if key:
            self.handle_key(key, currentTime)
        elif self.target is not None:
            self.drive_to_target(dt)
        elif currentTime - self.last_key_time > KEY_TIMEOUT:
            self.stop()
        elif self.mode == "GLOBAL":
            self.vel_x, self.vel_y = self.world_to_body(
                self.global_vel_x, self.global_vel_y
            )
            self.vel_yaw = self.global_vel_yaw

        self.publish_velocity()

    # PUB TO /CMD_VEL FOR MOVEMENT
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
