"""Repeatable house exploration trial. Run inside Pixi after sourcing install/local_setup.bash.

Uses an isolated DDS domain; owns and cleans up only its child process groups.
Laser distances are sensor ranges, not footprint clearance or collision proof.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params', help='Omit for installed Nav2 defaults')
    parser.add_argument('--slam-params', help='SLAM Toolbox YAML')
    parser.add_argument('--frontier-params', help='Omit for installed explorer defaults')
    parser.add_argument('--x', type=float, default=-2.5)
    parser.add_argument('--y', type=float, default=3.5)
    parser.add_argument('--seconds', type=float, default=300, help='Simulation seconds after odometry starts')
    parser.add_argument('--domain', type=int, default=96)
    parser.add_argument('--contacts', action='store_true', help='Monitor Gazebo contacts from first odometry')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    wall_start = time.monotonic()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    os.environ['PYTHONUNBUFFERED'] = '1'
    processes, files = [], []

    def launch(name, command):
        handle = (out / f'{name}.log').open('w')
        files.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)

    rclpy.init()
    node = rclpy.create_node('nav2_trial_probe')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    samples = []
    state = {'distance': 0., 'known_area': 0., 'scan_min': None, 'active': False}
    first_stamp = None
    last_position = None
    last_sample = -1.
    snapshots = {}
    goal_feedback = {}

    def feedback(msg):
        key = bytes(msg.goal_id.uuid).hex()
        t = samples[-1]['t'] if samples else 0.
        entry = goal_feedback.setdefault(key, {'first_t': t, 'first_remaining': msg.feedback.distance_remaining, 'samples': []})
        if not entry['samples'] or t - entry['samples'][-1][0] >= 1.:
            entry['samples'].append([t, msg.feedback.distance_remaining])

    def odom(msg):
        nonlocal first_stamp, last_position, last_sample
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if first_stamp is None:
            first_stamp = stamp
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        state['yaw'] = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if last_position:
            state['distance'] += math.hypot(p.x-last_position[0], p.y-last_position[1])
        last_position = (p.x, p.y)
        if stamp - last_sample >= 1.:
            last_sample = stamp
            samples.append(dict(t=round(stamp-first_stamp, 2), x=p.x, y=p.y, **state))

    def scan(msg):
        values = [x for x in msg.ranges if math.isfinite(x) and msg.range_min <= x <= msg.range_max]
        state['scan_min'] = min(values) if values else None

    def grid(msg):
        state['known_area'] = sum(x >= 0 for x in msg.data) * msg.info.resolution**2
        snapshots['map'] = dict(width=msg.info.width, height=msg.info.height, resolution=msg.info.resolution, origin=[msg.info.origin.position.x, msg.info.origin.position.y], data=list(msg.data))

    def plan(msg):
        snapshots['plan'] = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]

    def velocity(msg):
        state['cmd_v'] = msg.linear.x
        state['cmd_w'] = msg.angular.z

    def status(msg):
        state['active'] = any(x.status in (1, 2) for x in msg.status_list)

    def costmap_snapshot(name, msg):
        snapshots[name] = dict(frame=msg.header.frame_id, width=msg.info.width,
                              height=msg.info.height, resolution=msg.info.resolution,
                              origin=[msg.info.origin.position.x, msg.info.origin.position.y],
                              data=list(msg.data))

    completion_events = []
    def completed(msg):
        completion_events.append(samples[-1]['t'] if samples else 0.)

    subscriptions = [
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: costmap_snapshot('global_costmap', m), 10),
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: costmap_snapshot('local_costmap', m), 10),
        node.create_subscription(Empty, '/exploration_complete', completed, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid, '/map', grid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', status, QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(NavPath, '/plan', plan, 10),
        node.create_subscription(Twist, '/cmd_vel', velocity, 10),
        node.create_subscription(NavigateToPose_FeedbackMessage, '/navigate_to_pose/_action/feedback', feedback, 10),
    ]
    try:
        launch('house', ['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_house.launch.py', f'x_pose:={args.x}', f'y_pose:={args.y}'])
        slam_command = ['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'use_sim_time:=true']
        if args.slam_params:
            slam_command.append(f'slam_params_file:={Path(args.slam_params).resolve()}')
        launch('slam', slam_command)
        command = ['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'use_sim_time:=true']
        if args.params:
            command.append(f'params_file:={Path(args.params).resolve()}')
        launch('navigation', command)
        explorer = ['ros2', 'launch', 'frontier_exploration_ros2', 'frontier_explorer.launch.py', 'use_sim_time:=true', 'autostart:=false', 'control_service_enabled:=true']
        if args.frontier_params:
            explorer.append(f'params_file:={Path(args.frontier_params).resolve()}')
        launch('frontier', explorer)
        launch('subscriber', ['ros2', 'run', 'midterm', 'sub', '--ros-args', '-p', 'use_sim_time:=true'])
        start = time.monotonic()
        report = start
        client = node.create_client(GetParameters, '/controller_server/get_parameters')
        names = ['FollowPath.plugin', 'FollowPath.max_vel_x', 'FollowPath.trans_stopped_velocity', 'FollowPath.BaseObstacle.scale', 'FollowPath.PathAlign.scale', 'progress_checker.required_movement_radius']
        if args.params and 'RegulatedPurePursuitController' in Path(args.params).read_text():
            names = ['FollowPath.plugin', 'FollowPath.desired_linear_vel', 'FollowPath.lookahead_dist', 'FollowPath.use_collision_detection', 'FollowPath.use_regulated_linear_velocity_scaling']
        if args.params and 'RotationShimController' in Path(args.params).read_text():
            names += ['FollowPath.primary_controller', 'FollowPath.angular_dist_threshold', 'FollowPath.forward_sampling_distance', 'FollowPath.rotate_to_heading_angular_vel']
        if args.params and 'ObstacleFootprint' in Path(args.params).read_text():
            names = [n.replace('BaseObstacle', 'ObstacleFootprint') for n in names]
        future = None
        next_query = 0.0
        contact_started = False
        last_stall_snapshot = -100.
        while time.monotonic() - start < max(180, args.seconds * 4):
            rclpy.spin_once(node, timeout_sec=.1)
            if args.contacts and samples and not contact_started:
                launch('contacts', [sys.executable, str(Path(__file__).with_name('contact_probe.py')),
                                    str(out / 'contacts.json'), str(args.seconds * 4 + 60)])
                contact_started = True
            poses = {}
            for frame in ('map', 'odom'):
                try:
                    t = tf_buffer.lookup_transform(frame, 'base_link', rclpy.time.Time()).transform
                    poses[frame] = [t.translation.x, t.translation.y,
                                    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
                except Exception:
                    pass
            snapshots['robot_poses'] = poses
            if len(samples) >= 21 and samples[-1]['t'] - last_stall_snapshot >= 20:
                if math.hypot(samples[-1]['x'] - samples[-21]['x'], samples[-1]['y'] - samples[-21]['y']) < .1:
                    last_stall_snapshot = samples[-1]['t']
                    (out / f'stall-{last_stall_snapshot:.0f}.json').write_text(json.dumps(snapshots))
            if future is None and client.service_is_ready() and time.monotonic() >= next_query:
                future = client.call_async(GetParameters.Request(names=names))
                next_query = time.monotonic() + 1.0
            if future and future.done() and not (out / 'active-controller.json').exists():
                values = future.result().values
                if len(values) == len(names) and all(v.type for v in values):
                    (out / 'active-controller.json').write_text(json.dumps({n: str(v) for n, v in zip(names, values)}, indent=2))
                else:
                    future = None  # Lifecycle node may not be configured yet.
            if time.monotonic() - report >= 30:
                print(json.dumps(dict(output=args.output, sample=samples[-1] if samples else None)), flush=True)
                (out / 'latest-snapshot.json').write_text(json.dumps(snapshots))
                (out / 'goal-feedback.json').write_text(json.dumps(goal_feedback))
                report = time.monotonic()
            if samples and samples[-1]['t'] >= args.seconds:
                break
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('A launch process exited; inspect trial logs')
        if not samples or samples[-1]['t'] < args.seconds:
            raise RuntimeError('Simulation failed to advance to requested duration')
    finally:
        (out / 'samples.json').write_text(json.dumps(samples, indent=2))
        (out / 'completion-events.json').write_text(json.dumps(completion_events, indent=2))
        (out / 'goal-feedback.json').write_text(json.dumps(goal_feedback))
        (out / 'snapshots.json').write_text(json.dumps(snapshots))
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for handle in files:
            handle.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    nav = (out / 'navigation.log').read_text()
    frontier = (out / 'frontier.log').read_text()
    ranges = sorted(s['scan_min'] for s in samples if s['scan_min'] is not None and s['t'] >= 10)
    stalled = sum(s['active'] and math.hypot(s['x']-samples[i-20]['x'], s['y']-samples[i-20]['y']) < .10 for i, s in enumerate(samples) if i >= 20)
    summary = dict(spawn=[args.x, args.y], params=args.params or 'installed defaults', sim_seconds=samples[-1]['t'], distance_m=state['distance'], known_area_m2=state['known_area'], scan_min_m=min(ranges) if ranges else None, scan_p05_m=ranges[int(.05*len(ranges))] if ranges else None, stationary_active_windows=stalled, frontier_reached=frontier.count('Frontier goal reached'), frontier_failed=frontier.count('Frontier goal failed'), progress_failures=nav.count('Failed to make progress'), no_valid_trajectories=nav.count('No valid trajectories'), nav_goal_failed=nav.count('Goal failed'), nav_goal_succeeded=nav.count('Goal succeeded'))
    # The pinned explorer reports aborted actions using the ROS status name.
    summary['frontier_failed'] += frontier.count('frontier finished with status STATUS_ABORTED')
    summary['frontier_failed'] += frontier.count('Frontier goal was rejected')
    summary['nav_goal_canceled'] = nav.count('Goal canceled')
    summary['completion_events'] = completion_events
    summary['max_x'] = max(s['x'] for s in samples)
    summary['wall_seconds'] = time.monotonic() - wall_start
    moving = [s for s in samples if s['t'] >= 25]
    summary['turning_samples'] = sum(abs(s.get('cmd_v', 0)) < .03 and abs(s.get('cmd_w', 0)) > .1 for s in moving)
    summary['post_start_samples'] = len(moving)
    summary['area_times'] = {str(area): next((s['t'] for s in samples if s['known_area'] >= area), None) for area in (10,20,30,40,50,60)}
    (out / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
