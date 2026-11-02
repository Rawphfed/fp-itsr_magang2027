import os
import sys
import select
import termios
import tty
import time
import math

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

SPEED = 0.5
ANGULAR_SPEED = 1.0  # Turn speed for autonomous orientation
KEY_TIMEOUT = 0.3
STEP_DISTANCE = 0.5  # Distance to move in meters per keypress in GLOBAL mode
STEP_ANGLE = math.pi / 4  # Angle to rotate in radians (45 degrees) per keypress in GLOBAL mode


class KeyboardControl(Node):
    def __init__(self):
        super().__init__("keyboard_control")

        # Mode of the movement
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_publisher = self.create_publisher(String, "/robot_mode", qos_latched)
        self.mode = "LOCAL"

        # Publisher subscribers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Odometry setups
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Global navigation targets
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.global_target_active = False
        self.global_move_type = None  # "linear" or "angular"

        # Others
        self.fd = sys.stdin.fileno()
        self.settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_key_time = time.monotonic()

        self.cmd = Twist()

        self.publish_mode()
        
        # Changed timer from 1.0 to 0.05 (20Hz) for responsive control loop execution
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            "W: forward | S: backward | A: left | D: right | SPACE: stop | M: toggle mode"
        )

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_publisher.publish(msg)

    def read_key(self):
        latest = ""

        while select.select([self.fd], [], [], 0)[0]:
            char = os.read(self.fd, 1).decode(errors="ignore")
            if not char:
                break
            latest = char

        return latest

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternions
        ## Reference: https://tom-howard.github.io/ros2/course/part2/#pose
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def update_motion(self, key, now):
        if key.lower() == "m":
            self.mode = "GLOBAL" if self.mode == "LOCAL" else "LOCAL"
            self.publish_mode()
            self.stop_motion()
            self.global_target_active = False
            self.get_logger().info(f"Switched mode to: {self.mode}")
            return

        if key == " ":
            self.stop_motion()
            self.global_target_active = False
            return

        # --- LOCAL MODE: Standard continuous velocity control ---
        if self.mode == "LOCAL":
            match key.lower():
                case "w":
                    self.linear_x = SPEED
                    self.angular_z = 0.0
                    self.last_key_time = now
                case "s":
                    self.linear_x = -SPEED
                    self.angular_z = 0.0
                    self.last_key_time = now
                case "a":
                    self.linear_x = 0.0
                    self.angular_z = SPEED
                    self.last_key_time = now
                case "d":
                    self.linear_x = 0.0
                    self.angular_z = -SPEED
                    self.last_key_time = now

        # --- GLOBAL MODE: Trajectory targets relative to absolute frame ---
        elif self.mode == "GLOBAL":
            # Prevent altering the trajectory midway if already executing a goal
            if self.global_target_active:
                return

            self.global_target_active = True
            self.last_key_time = now

            match key.lower():
                case "w":  # Absolute forward along current heading
                    self.target_x = self.current_x + STEP_DISTANCE * math.cos(self.current_yaw)
                    self.target_y = self.current_y + STEP_DISTANCE * math.sin(self.current_yaw)
                    self.global_move_type = "linear"
                case "s":  # Absolute backward along current heading
                    self.target_x = self.current_x - STEP_DISTANCE * math.cos(self.current_yaw)
                    self.target_y = self.current_y - STEP_DISTANCE * math.sin(self.current_yaw)
                    self.global_move_type = "linear"
                case "a":  # Turn 45 degrees left absolute
                    self.target_yaw = self.current_yaw + STEP_ANGLE
                    self.global_move_type = "angular"
                case "d":  # Turn 45 degrees right absolute
                    self.target_yaw = self.current_yaw - STEP_ANGLE
                    self.global_move_type = "angular"
                case _:
                    self.global_target_active = False

    def stop_motion(self):
        self.linear_x = 0.0
        self.angular_z = 0.0

    def control_loop(self):
        now = time.monotonic()
        key = self.read_key()

        if key:
            self.update_motion(key, now)

        if self.mode == "LOCAL":
            # Timeout logic: stop if no key pressed within window
            if now - self.last_key_time > KEY_TIMEOUT:
                self.stop_motion()

        elif self.mode == "GLOBAL" and self.global_target_active:
            if self.global_move_type == "linear":
                # Compute distance error remaining to coordinate goal
                distance_err = math.sqrt((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)
                
                if distance_err > 0.02:  # 2 cm tolerance
                    # Determine whether we are tracking a forward step or reverse step
                    # Look ahead angle error to detect if target is behind us
                    heading_to_target = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
                    angle_diff = math.atan2(math.sin(heading_to_target - self.current_yaw), math.cos(heading_to_target - self.current_yaw))
                    
                    if abs(angle_diff) > math.pi / 2:
                        self.linear_x = -SPEED  # Moving backward
                    else:
                        self.linear_x = SPEED   # Moving forward
                    self.angular_z = 0.0
                else:
                    self.stop_motion()
                    self.global_target_active = False

            elif self.global_move_type == "angular":
                # Compute absolute angular shift error remaining, normalized between [-pi, pi]
                angle_err = math.atan2(math.sin(self.target_yaw - self.current_yaw), math.cos(self.target_yaw - self.current_yaw))
                
                if abs(angle_err) > 0.03:  # ~1.7 degrees tolerance
                    self.linear_x = 0.0
                    self.angular_z = ANGULAR_SPEED if angle_err > 0 else -ANGULAR_SPEED
                else:
                    self.stop_motion()
                    self.global_target_active = False

        self.cmd.linear.x = self.linear_x
        self.cmd.angular.z = self.angular_z
        self.cmd_pub.publish(self.cmd)

    def close(self):
        self.stop_motion()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.cmd_pub.publish(self.cmd)

        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)


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
