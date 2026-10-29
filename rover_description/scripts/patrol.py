#!/usr/bin/env python3
"""Send the rover on a patrol of waypoints with Nav2's Python API.

Run (Nav2 must already be running):
  python3 patrol.py --ros-args -p use_sim_time:=true
Needs: sudo apt install ros-humble-nav2-simple-commander
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# (x, y, yaw in radians) in the map frame = Gazebo world frame
WAYPOINTS = [
    (0.0, 3.5, 0.0),      # north room, straight through the middle opening
    (4.5, 3.5, 0.0),      # north-east corner area
    (4.5, -3.5, 0.0),     # south-east, through the gap beside the east wall
    (-4.5, -3.5, 0.0),    # south-west, past the pillars and the stack
    (0.0, 0.0, 0.0),      # back to the start
]


def make_pose(nav, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.setInitialPose(make_pose(nav, 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()

    for i, (x, y, yaw) in enumerate(WAYPOINTS, start=1):
        print(f'Waypoint {i}/{len(WAYPOINTS)}: ({x}, {y})')
        nav.goToPose(make_pose(nav, x, y, yaw))
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None:
                print(f'  distance remaining: {fb.distance_remaining:.2f} m', end='\r')
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('\n  reached')
        else:
            print(f'\n  stopped: {result}')
            break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
