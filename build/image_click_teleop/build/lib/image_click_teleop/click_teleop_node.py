#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np


class ClickTeleopNode(Node):

    def __init__(self):
        super().__init__('click_teleop_node')

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.current_image = None
        self.center_y = None

        self.window_name = "Aruco Teleop TurtleBot3"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        self.target_id = 0
        self.use_aruco = True  # ArUco control enabled

        self.get_logger().info(
            'ClickTeleopNode started. '
            'Mode: ArUco (marker id 0) above/below center line -> forward/backward.'
        )

    def image_callback(self, msg: Image):
        self.current_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        h, w = self.current_image.shape[:2]
        self.center_y = h // 2

        img = self.current_image.copy()
        cv2.line(img, (0, self.center_y), (w, self.center_y), (0, 255, 0), 2)
        cv2.putText(img, "FORWARD", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(img, "BACKWARD", (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if self.use_aruco:
            gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )

            if ids is not None:
                self.get_logger().info(
                    f"Detected markers: {ids.flatten().tolist()}"
                )

                for c, i in zip(corners, ids.flatten()):
                    if i != self.target_id:
                        continue

                    pts = c[0]
                    cx = int(pts[:, 0].mean())
                    cy = int(pts[:, 1].mean())

                    aruco.drawDetectedMarkers(img, [c], np.array([[i]]))
                    cv2.circle(img, (cx, cy), 5, (255, 0, 0), -1)

                    msg_twist = TwistStamped()
                    msg_twist.header.stamp = self.get_clock().now().to_msg()
                    msg_twist.header.frame_id = 'base_link'

                    if cy < self.center_y:
                        msg_twist.twist.linear.x = 0.2
                        self.get_logger().info(
                            f"FORWARD (marker y={cy}, center={self.center_y})"
                        )
                    else:
                        msg_twist.twist.linear.x = -0.2
                        self.get_logger().info(
                            f"BACKWARD (marker y={cy}, center={self.center_y})"
                        )

                    self.cmd_vel_pub.publish(msg_twist)
                    break

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if self.use_aruco:
            return

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
            if key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()