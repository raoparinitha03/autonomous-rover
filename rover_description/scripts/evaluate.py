#!/usr/bin/env python3
"""Measure how well the rover navigates: success rate, goal error, time, path length.

Run with Nav2 already up:
  python3 evaluate.py --trials 20 --seed 1 --ros-args -p use_sim_time:=true

Ground truth comes from the Gazebo p3d plugin (/ground_truth/odom), NOT from
the robot's own localisation, so localisation mistakes are caught.
Results are written to results.csv and summarised on screen.
"""
import argparse
import csv
import math
import random
import statistics
import sys

# (x, y, yaw) goals in the map frame (= Gazebo world frame); all in free space
GOALS = [
    (0.0, 3.5, 1.57),
    (4.5, 3.5, 0.0),
    (-4.5, 3.5, 3.14),
    (4.5, -3.5, -1.57),
    (-4.5, -3.5, 3.14),
    (0.0, -4.0, 0.0),
    (0.0, 0.0, 0.0),
]


# ---------- statistics helpers (no ROS needed) ----------
def wilson_interval(k, n, z=1.96):
    """95% confidence interval for a success rate k/n."""
    if n == 0:
        return 0.0, 0.0
    p = k / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def summarise(rows):
    n = len(rows)
    nav_ok = sum(r['nav_result'] == 'SUCCEEDED' for r in rows)
    ok = [r for r in rows if r['verified']]
    lo, hi = wilson_interval(len(ok), n)
    lines = [
        '',
        '================ RESULTS ================',
        f'trials                       : {n}',
        f'Nav2 reported success        : {nav_ok}/{n}',
        f'verified success (truth)     : {len(ok)}/{n} = {100*len(ok)/max(n,1):.0f}%',
        f'  95% confidence interval    : {100*lo:.0f}% to {100*hi:.0f}%',
    ]
    if ok:
        pe = [r['pos_err_m'] * 100 for r in ok]
        ye = [r['yaw_err_deg'] for r in ok]
        lines += [
            f'goal position error (cm)     : mean {statistics.mean(pe):.1f}, '
            f'median {statistics.median(pe):.1f}, max {max(pe):.1f}',
            f'goal heading error (deg)     : mean {statistics.mean(ye):.1f}, max {max(ye):.1f}',
            f'time per goal (s, sim)       : mean {statistics.mean(r["time_s"] for r in ok):.1f}',
            f'path length per goal (m)     : mean {statistics.mean(r["path_len_m"] for r in ok):.2f}',
            f'localisation error (cm)      : mean '
            f'{statistics.mean(r["amcl_err_m"] for r in ok)*100:.1f}',
            f'recovery behaviours per goal : mean '
            f'{statistics.mean(r["recoveries"] for r in ok):.2f}',
        ]
    failed = [r for r in rows if not r['verified']]
    if failed:
        lines.append('failures:')
        for r in failed:
            lines.append(f'  trial {r["trial"]}: goal ({r["goal_x"]}, {r["goal_y"]}) '
                         f'-> {r["nav_result"]}, final error {r["pos_err_m"]*100:.0f} cm')
    return '\n'.join(lines)


# ---------- start-up check with visible progress ----------
def wait_ready(nav, state, make_initial_pose, timeout):
    """Wait until Nav2 is active and localised, printing what is missing."""
    import time

    import rclpy
    from lifecycle_msgs.srv import GetState

    def node_state(name):
        cli = nav.create_client(GetState, f'/{name}/get_state')
        try:
            if not cli.wait_for_service(timeout_sec=1.0):
                return 'NOT RUNNING'
            fut = cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav, fut, timeout_sec=2.0)
            return fut.result().current_state.label if fut.result() else 'no reply'
        finally:
            nav.destroy_client(cli)

    names = ['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator']
    start, last_pose_pub = time.monotonic(), 0.0
    while time.monotonic() - start < timeout:
        states = {n: node_state(n) for n in names}
        clock_ok = nav.get_clock().now().nanoseconds > 0
        got_pose = state['amcl'] is not None
        print('  Nav2 start-up: ' + ', '.join(f'{n}={v}' for n, v in states.items())
              + f', amcl_pose={"yes" if got_pose else "no"}'
              + ('' if clock_ok else '  [WARNING: no /clock, simulation time is 0]'))
        if all(v == 'active' for v in states.values()) and got_pose:
            print('Nav2 is ready.')
            return
        if states['amcl'] == 'active' and not got_pose \
                and time.monotonic() - last_pose_pub > 3.0:
            nav.setInitialPose(make_initial_pose())      # re-send until AMCL answers
            last_pose_pub = time.monotonic()
        rclpy.spin_once(nav, timeout_sec=1.0)
    sys.exit('\nNav2 did not become ready. The line above shows which part is missing: '
             'a node that is NOT RUNNING or not active, no amcl_pose, or no /clock.')


# ---------- the experiment ----------
def run(args):
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from nav_msgs.msg import Odometry
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)

    rclpy.init()
    nav = BasicNavigator()

    state = {'gt': None, 'amcl': None, 'active': False, 'path': 0.0, 'last': None}

    def on_gt(msg):
        p = msg.pose.pose
        cur = (p.position.x, p.position.y)
        if state['active'] and state['last'] is not None:
            state['path'] += math.dist(cur, state['last'])
        state['last'] = cur
        state['gt'] = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_amcl(msg):
        p = msg.pose.pose
        state['amcl'] = (p.position.x, p.position.y)

    nav.create_subscription(Odometry, args.gt_topic, on_gt, 10)
    amcl_qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
    nav.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl, amcl_qos)

    def pose(x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = nav.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    wait_ready(nav, state, lambda: pose(0.0, 0.0, 0.0), args.startup_timeout)

    # wait for ground truth
    for _ in range(100):
        rclpy.spin_once(nav, timeout_sec=0.1)
        if state['gt'] is not None:
            break
    if state['gt'] is None:
        sys.exit(f'No ground truth on {args.gt_topic}. Add the p3d plugin to the URDF '
                 'and restart the simulation (see the instructions).')

    rng = random.Random(args.seed)
    rows, last_goal = [], (0.0, 0.0, 0.0)
    for trial in range(1, args.trials + 1):
        goal = rng.choice([g for g in GOALS if g != last_goal])
        last_goal = goal
        print(f'\nTrial {trial}/{args.trials}: -> ({goal[0]}, {goal[1]})')

        state['path'], state['last'], state['active'] = 0.0, state['gt'][:2], True
        t0 = nav.get_clock().now()
        nav.goToPose(pose(*goal))
        recoveries = 0
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None:
                recoveries = max(recoveries, fb.number_of_recoveries)
            elapsed = (nav.get_clock().now() - t0).nanoseconds / 1e9
            if elapsed > args.timeout:
                nav.cancelTask()
                break
        result = nav.getResult()
        state['active'] = False
        elapsed = (nav.get_clock().now() - t0).nanoseconds / 1e9
        name = result.name if hasattr(result, 'name') else str(result)
        if elapsed > args.timeout:
            name = 'TIMEOUT'

        for _ in range(10):                      # let the last messages arrive
            rclpy.spin_once(nav, timeout_sec=0.05)
        gx, gy, gyaw = state['gt']
        pos_err = math.dist((gx, gy), goal[:2])
        yaw_err = abs(math.degrees(wrap(gyaw - goal[2])))
        amcl_err = math.dist(state['amcl'], (gx, gy)) if state['amcl'] else float('nan')
        verified = (result == TaskResult.SUCCEEDED and name != 'TIMEOUT'
                    and pos_err <= args.tol_xy and yaw_err <= args.tol_yaw)
        rows.append({
            'trial': trial, 'goal_x': goal[0], 'goal_y': goal[1], 'goal_yaw': goal[2],
            'nav_result': name, 'verified': verified, 'time_s': round(elapsed, 1),
            'path_len_m': round(state['path'], 2), 'pos_err_m': round(pos_err, 3),
            'yaw_err_deg': round(yaw_err, 1), 'amcl_err_m': round(amcl_err, 3),
            'recoveries': recoveries,
        })
        print(f'  {name}, error {pos_err*100:.0f} cm, {yaw_err:.0f} deg, '
              f'{elapsed:.0f} s, verified={verified}')

    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(summarise(rows))
    print(f'\nper-trial data saved to {args.out}')
    rclpy.shutdown()


def main():
    from rclpy.utilities import remove_ros_args
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=20)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=120.0, help='seconds per goal (sim time)')
    ap.add_argument('--tol-xy', type=float, default=0.25, help='verified-success position tolerance (m)')
    ap.add_argument('--tol-yaw', type=float, default=20.0, help='verified-success heading tolerance (deg)')
    ap.add_argument('--gt-topic', default='/ground_truth/odom')
    ap.add_argument('--startup-timeout', type=float, default=90.0, help='seconds to wait for Nav2')
    ap.add_argument('--out', default='results.csv')
    run(ap.parse_args(remove_ros_args(sys.argv)[1:]))


if __name__ == '__main__':
    main()