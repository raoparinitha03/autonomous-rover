#!/usr/bin/env python3
"""
ROS2 node: captures camera frames, runs ONNX object-detection inference,
and sends pick-and-place goals to the arm's PickPlace action server
(action name: /pick_place) instead of publishing a plain string command.
"""
import cv2
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from robot_vision_interfaces.action import PickPlace


class YoloVisionNode(Node):
    def __init__(self):
        super().__init__('yolo_vision_node')

        # --- Parameters (overridable via launch file or CLI) ---
        self.declare_parameter('model_path', 'best.onnx')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('inference_period_sec', 0.05)  # ~20 Hz

        model_path = self.get_parameter('model_path').value
        camera_index = self.get_parameter('camera_index').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        period = self.get_parameter('inference_period_sec').value

        # --- Model ---
        self.get_logger().info(f'Loading ONNX model from {model_path}')
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name

        # --- Camera ---
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open camera index {camera_index}')

        self.bridge = CvBridge()
        # True while a PickPlace goal is active. The action server also
        # tracks its own busy state internally and will reject a goal sent
        # while one is running, but tracking it here too avoids spamming
        # goal requests that we already know will be rejected.
        self.robot_busy = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)
        self.image_pub = (
            self.create_publisher(Image, '/camera/image_annotated', qos)
            if self.publish_debug_image else None
        )

        self._action_client = ActionClient(self, PickPlace, 'pick_place')

        self.timer = self.create_timer(period, self.inference_step)
        self.get_logger().info('YoloVisionNode ready.')

    def inference_step(self):
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warn('Camera frame read failed.')
            return

        # Pre-process (matches original export: 640x640, NCHW, 0-1 float)
        img = cv2.resize(frame, (640, 640))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = np.transpose(np.expand_dims(img, 0), (0, 3, 1, 2))

        outputs = self.session.run(None, {self.input_name: img})
        data = np.squeeze(outputs[0])  # shape [6, 8400]

        scores = np.max(data[4:, :], axis=0)
        best_idx = np.argmax(scores)
        max_conf = scores[best_idx]

        if max_conf > self.conf_thresh and not self.robot_busy:
            raw_x = float(data[0, best_idx])  # normalized 0.0-1.0

            label_id = int(np.argmax(data[4:, best_idx]))
            label = 'box' if label_id == 0 else 'cylinder'

            # 0.0 (left) -> 120 deg, 0.5 (center) -> 90 deg, 1.0 (right) -> 60 deg
            angle = int(120 - (raw_x * 60))
            angle = max(min(angle, 120), 60)

            if self.publish_debug_image:
                line_x = int(raw_x * frame.shape[1])
                cv2.line(frame, (line_x, 0), (line_x, frame.shape[0]), (0, 255, 0), 3)
                cv2.putText(frame, f'ANGLE: {angle}', (line_x + 10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            self.send_pick_goal(angle, label)

        if self.publish_debug_image and self.image_pub is not None:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera'
            self.image_pub.publish(img_msg)

    def send_pick_goal(self, angle, label):
        if not self._action_client.server_is_ready():
            self.get_logger().warn('Arm action server not available yet; skipping detection.')
            return

        self.robot_busy = True
        goal_msg = PickPlace.Goal()
        goal_msg.angle = angle
        goal_msg.label = label

        self.get_logger().info(f'TRIGGER: sending goal -> {label} at {angle} degrees')
        send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by arm action server (likely already busy).')
            self.robot_busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback):
        fb = feedback.feedback
        self.get_logger().info(f'  stage={fb.stage} progress={fb.progress:.2f}')

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Goal finished: {result.message}')
        else:
            self.get_logger().warn(f'Goal failed: {result.message}')
        self.robot_busy = False

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
