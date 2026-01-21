#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from builtin_interfaces.msg import Duration


class ClickTeleopNode(Node):

    def __init__(self):
        super().__init__('click_teleop_node')

        # Publisher for UR5 joint trajectory controller
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.bridge = CvBridge()
        self.current_image = None
        self.center_y = None
        self.current_joints = None
        self.joints_received = False

        # ArUco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        self.target_id = 0  
        self.use_aruco = True

        # UR5 joints
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        # Window for display
        self.window_name = "ArUco Control UR5e"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.get_logger().info('='*60)
        self.get_logger().info('ArUco UR5e Control started.')
        self.get_logger().info('Show ArUco marker ID 0 to the camera')
        self.get_logger().info('Marker ABOVE center line = rotate base +0.5 rad')
        self.get_logger().info('Marker BELOW center line = rotate base -0.5 rad')
        self.get_logger().info('='*60)

    # ================= IMAGE CALLBACK =================
    def image_callback(self, msg: Image):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
            
        h, w = self.current_image.shape[:2]
        self.center_y = h // 2

        img = self.current_image.copy()
        
        # Central line
        cv2.line(img, (0, self.center_y), (w, self.center_y), (0, 255, 0), 3)
        
        # Instructions
        cv2.putText(img, "ArUco ABOVE - Positive Rotation", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(img, "ArUco BELOW - Negative Rotation", (10, h - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Status indicator
        if self.current_joints is not None:
            cv2.putText(img, "Robot: READY", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            base_angle = self.current_joints.get('shoulder_pan_joint', 0.0)
            cv2.putText(img, f"Base: {base_angle:.2f} rad", (10, 85), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        else:
            cv2.putText(img, "Robot: WAITING...", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # ArUco detection
        if self.use_aruco:
            gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )

            if ids is not None:
                for c, marker_id in zip(corners, ids.flatten()):
                    if marker_id != self.target_id:
                        continue

                    # Calculate marker center
                    pts = c[0]
                    cx = int(pts[:, 0].mean())
                    cy = int(pts[:, 1].mean())

                    # Draw marker and center
                    aruco.drawDetectedMarkers(img, [c], np.array([[marker_id]]))
                    cv2.circle(img, (cx, cy), 5, (255, 0, 0), -1)

                    # Display marker position
                    cv2.putText(img, f"Marker {marker_id} detected", (10, 110),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                    # Send command based on marker position
                    if self.current_joints is not None:
                        self.send_robot_command(cy)
                    break

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    # ================= JOINT STATE CALLBACK =================
    def joint_state_callback(self, msg: JointState):
        self.current_joints = dict(zip(msg.name, msg.position))
        
        if not self.joints_received:
            self.joints_received = True
            self.get_logger().info('='*60)
            self.get_logger().info('JOINT STATES RECEIVED - Robot Ready!')
            self.get_logger().info(f'Available joints: {list(self.current_joints.keys())}')
            self.get_logger().info('='*60)

    # ================= SEND ROBOT COMMAND =================
    def send_robot_command(self, marker_y):
        """Send trajectory command based on marker Y position"""
        try:
            positions = [self.current_joints[name] for name in self.joint_names]
        except KeyError as e:
            self.get_logger().error(f'Missing joint: {e}')
            return

        original_base = positions[0]
        delta_base = 0.5 

        if marker_y < self.center_y:
            positions[0] += delta_base
            direction = "FORWARD (+0.5 rad)"
        else:
            positions[0] -= delta_base
            direction = "BACKWARD (-0.5 rad)"

        self.get_logger().info('='*60)
        self.get_logger().info(f'ArUco detected: {direction}')
        self.get_logger().info(f'Marker Y: {marker_y}, Center: {self.center_y}')
        self.get_logger().info(f'Base joint: {original_base:.3f} → {positions[0]:.3f} rad')

        # Create and publish trajectory
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        traj.points.append(point)
        
        self.traj_pub.publish(traj)
        self.get_logger().info('✓ Trajectory published!')
        self.get_logger().info('='*60)

    # ================= MOUSE CALLBACK =================
    def mouse_callback(self, event, x, y, flags, param):
        """Fallback: Click control if ArUco is disabled"""
        if self.use_aruco:
            return

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.current_joints is None:
            self.get_logger().warn('Joint states not available!')
            return

        try:
            positions = [self.current_joints[name] for name in self.joint_names]
        except KeyError as e:
            self.get_logger().error(f'Missing joint: {e}')
            return

        original_base = positions[0]
        delta_base = 0.5

        if y < self.center_y:
            positions[0] += delta_base
            direction = "FORWARD (+0.5 rad)"
        else:
            positions[0] -= delta_base
            direction = "BACKWARD (-0.5 rad)"

        self.get_logger().info('='*60)
        self.get_logger().info(f'Click: {direction}')
        self.get_logger().info(f'Base joint: {original_base:.3f} → {positions[0]:.3f} rad')

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        traj.points.append(point)
        
        self.traj_pub.publish(traj)
        self.get_logger().info('✓ Trajectory published!')
        self.get_logger().info('='*60)

    # ================= CLEANUP =================
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ClickTeleopNode()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                node.get_logger().info('ESC pressed, shutting down...')
                break
            elif key == ord('q'):
                node.get_logger().info('Q pressed, shutting down...')
                break
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
