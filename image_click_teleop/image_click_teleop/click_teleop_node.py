#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge
import cv2


class ClickTeleopNode(Node):
    def __init__(self):
        super().__init__('click_teleop_node')

        # Publish TwistStamped on /cmd_vel
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.current_image = None
        self.center_y = None

        self.window_name = "Click to Control TurtleBot3"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info('ClickTeleopNode started. Click above/below center line.')

    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV image
        self.current_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )
        h, w = self.current_image.shape[:2]
        self.center_y = h // 2

        # Draw center line and show
        img = self.current_image.copy()
        cv2.line(img, (0, self.center_y), (w, self.center_y), (0, 255, 0), 2)
        cv2.putText(img, "FORWARD", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(img, "BACKWARD", (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        # Only react on left button click
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.current_image is None or self.center_y is None:
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if y < self.center_y:
            msg.twist.linear.x = 0.2
            self.get_logger().info(
                f'FORWARD: click y={y}, center={self.center_y}'
            )
        else:
            msg.twist.linear.x = -0.2
            self.get_logger().info(
                f'BACKWARD: click y={y}, center={self.center_y}'
            )

        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClickTeleopNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC to quit
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

