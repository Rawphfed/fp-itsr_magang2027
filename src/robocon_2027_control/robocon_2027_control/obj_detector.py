#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

## DETECTOR CONFIG (defaults, overridden by detector_params.yaml)
IMAGE_TOPIC = "/camera/image_color"
MIN_AREA = 300  
MIN_FILL = 0.4 
KERNEL_SIZE = 5

# [H_low, S_low, V_low, H_high, S_high, V_high] (OpenCV hue: 0-179)
BLUE_RANGE = [100, 120, 50, 130, 255, 255]
RED_RANGE_1 = [0, 120, 50, 10, 255, 255] 
RED_RANGE_2 = [170, 120, 50, 179, 255, 255]

COLORS = ("red", "blue")
DRAW_BGR = {"red": (0, 0, 255), "blue": (255, 0, 0)}


class BlockDetector(Node):
    def __init__(self):
        super().__init__("block_detector")

        self.declare_parameter("image_topic", IMAGE_TOPIC)
        self.declare_parameter("min_area", MIN_AREA)
        self.declare_parameter("min_fill", MIN_FILL)
        self.declare_parameter("kernel_size", KERNEL_SIZE)
        
        self.declare_parameter("blue_range", BLUE_RANGE)
        self.declare_parameter("red_range_1", RED_RANGE_1)
        self.declare_parameter("red_range_2", RED_RANGE_2)

        self.bridge = CvBridge()

        ## PUB SUBS
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.debug_pub = self.create_publisher(Image, "/detector/debug_image", 10)
        
        self.det_pubs = {
            color: self.create_publisher(PointStamped, f"/detector/{color}", 10)
            for color in COLORS
        }

        self.get_logger().info(
            f"Block detector listening on {self.get_parameter('image_topic').value}"
        )

    ## MAIN PIPELINE (once per frame)
    def image_callback(self, msg):
        # cv_bridge converts Webots' bgra8 to bgr8 for us
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        annotated = bgr.copy()
        for color in COLORS:
            blob = self.find_largest_blob(self.make_mask(hsv, color))
            if blob is None:
                continue
            self.publish_detection(color, blob, msg.header)
            self.annotate(annotated, color, blob)

        debug = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        debug.header = msg.header
        self.debug_pub.publish(debug)

    ## VISION HELPERS (plain arrays in/out, testable on saved images)
    def in_range(self, hsv, param_name):
        v = self.get_parameter(param_name).value
        return cv2.inRange(hsv, np.array(v[:3], np.uint8), np.array(v[3:], np.uint8))

    def make_mask(self, hsv, color):
        if color == "blue":
            mask = self.in_range(hsv, "blue_range")
        else:
            mask = cv2.bitwise_or(
                self.in_range(hsv, "red_range_1"),
                self.in_range(hsv, "red_range_2"),
            )

        k = self.get_parameter("kernel_size").value
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) 
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) 
        return mask

    def find_largest_blob(self, mask):
        min_area = self.get_parameter("min_area").value
        min_fill = self.get_parameter("min_fill").value

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = min_area
        for c in contours:
            area = cv2.contourArea(c)
            if area < best_area:
                continue

            x, y, w, h = cv2.boundingRect(c)
            if w * h == 0 or area / (w * h) < min_fill:
                continue

            m = cv2.moments(c)
            if m["m00"] == 0:
                continue

            best_area = area
            best = {
                "bbox": (x, y, w, h),
                "centroid": (m["m10"] / m["m00"], m["m01"] / m["m00"]),
                "area": area,
            }
        return best

    def annotate(self, image, color, blob):
        x, y, w, h = blob["bbox"]
        cx, cy = blob["centroid"]
        bgr = DRAW_BGR[color]
        cv2.rectangle(image, (x, y), (x + w, y + h), bgr, 2)
        cv2.circle(image, (int(cx), int(cy)), 4, bgr, -1)
        cv2.putText(
            image,
            color,
            (x, max(y - 6, 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            bgr,
            2,
        )

    ## OUTPUT
    def publish_detection(self, color, blob, header):
        out = PointStamped()
        out.header = header
        out.point.x, out.point.y = blob["centroid"]
        out.point.z = float(blob["area"])
        self.det_pubs[color].publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = BlockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
