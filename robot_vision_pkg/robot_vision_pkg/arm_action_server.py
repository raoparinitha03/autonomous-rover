#!/usr/bin/env python3
"""
ROS2 action server: exposes the arm's pick-and-place cycle as a ROS2 Action
(PickPlace) instead of a fire-and-forget topic. It drives the cycle by
sending "S <angle> <label>\\n" to the Arduino and waiting for "DONE\\n"
back over serial -- the Arduino sketch and protocol are UNCHANGED.

Why an Action instead of a topic/service:
  - the pick/place cycle takes ~20 seconds -- too long for a service call
    to block on comfortably
  - clients get live feedback while it runs (not just a final result)
  - clients can (attempt to) cancel a goal
  - only one goal is accepted at a time, so "is the arm busy" is handled
    by the action server itself instead of a manual busy flag on the
    client side
"""
import time
import threading

import serial

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from robot_vision_interfaces.action import PickPlace


# Known cumulative timing of the Arduino sketch's fixed delay() sequence
# (point -> pick position -> wait for human handover -> grab -> lift ->
# rotate to bin -> drop -> release -> return home). The Arduino itself only
# reports one final "DONE", so feedback here is an ESTIMATE derived from
# these durations, not real per-stage telemetry from the hardware.
#
# To get genuine per-stage feedback, extend the .ino sketch to
# Serial.println() a short marker (e.g. "STAGE grab") after each step and
# parse that here instead of relying on elapsed time.
STAGE_TIMELINE = [
    ('moving_to_object', 1.0),
    ('waiting_for_handover', 10.0),
    ('grabbing', 2.0),
    ('lifting', 2.0),
    ('rotating_to_bin', 1.5),
    ('dropping', 1.0),
    ('releasing', 2.0),
    ('returning_home', 2.0),
]
TOTAL_ESTIMATED_TIME = sum(duration for _, duration in STAGE_TIMELINE)


class ArmActionServer(Node):
    def __init__(self):
        super().__init__('arm_action_server')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('done_timeout_sec', 30.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.done_timeout = self.get_parameter('done_timeout_sec').value

        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Arduino connected on {port}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Arduino not found on {port}: {e}')

        self._lock = threading.Lock()
        self._busy = False

        # ReentrantCallbackGroup + MultiThreadedExecutor let this node keep
        # processing (e.g. answering goal_callback) while execute_callback
        # is blocked in its own polling loop for an already-accepted goal.
        self._action_server = ActionServer(
            self,
            PickPlace,
            'pick_place',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info('ArmActionServer ready (action: /pick_place).')

    def goal_callback(self, goal_request):
        if self.ser is None:
            self.get_logger().warn('Rejecting goal: no serial connection to Arduino.')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warn('Rejecting goal: arm is already busy.')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info(
            'Cancel requested -- note the Arduino sketch has no abort command, '
            'so the physical motion will keep running to completion regardless.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        with self._lock:
            self._busy = True

        angle = goal_handle.request.angle
        label = goal_handle.request.label
        result = PickPlace.Result()

        command = f'S {angle} {label}\n'
        self.get_logger().info(f'Sending to Arduino: {command.strip()}')
        self.ser.write(command.encode())

        feedback_msg = PickPlace.Feedback()
        start_time = time.monotonic()
        done_received = False

        while (time.monotonic() - start_time) < self.done_timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = ('Cancelled by client (physical arm motion was '
                                   'not actually interrupted).')
                with self._lock:
                    self._busy = False
                return result

            elapsed = time.monotonic() - start_time
            stage, progress = self._estimate_stage(elapsed)
            feedback_msg.stage = stage
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)

            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode(errors='ignore').strip()
                except Exception as e:
                    self.get_logger().warn(f'Serial read error: {e}')
                    line = ''
                if 'DONE' in line:
                    done_received = True
                    break

            time.sleep(0.2)

        with self._lock:
            self._busy = False

        if done_received:
            feedback_msg.stage = 'done'
            feedback_msg.progress = 1.0
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()
            result.success = True
            result.message = f'Placed {label} successfully.'
        else:
            goal_handle.abort()
            result.success = False
            result.message = 'Timed out waiting for Arduino DONE acknowledgement.'
            self.get_logger().error(result.message)

        return result

    @staticmethod
    def _estimate_stage(elapsed_sec):
        cumulative = 0.0
        for stage, duration in STAGE_TIMELINE:
            cumulative += duration
            if elapsed_sec <= cumulative:
                progress = min(elapsed_sec / TOTAL_ESTIMATED_TIME, 1.0)
                return stage, progress
        return 'finishing', 1.0

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
