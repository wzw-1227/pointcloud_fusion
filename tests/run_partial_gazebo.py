#!/usr/bin/env python3
"""Own a Gazebo test, reproduce the recorded hover location, and score matches.

Never attaches a controller to an existing simulation. Truth is used for physical
test positioning and independent scoring, never as a matcher initialization.
"""
import argparse
from datetime import datetime, timezone, timedelta
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
sys.path.insert(0, str(WORKSPACE / 'src/start_up/scripts'))
from stage3_verify_results import verify
from run_ros_checks import unused_port


def wait(predicate, timeout=90):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.1)
    raise RuntimeError('Gazebo validation timed out')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hold-sim-seconds', type=float, default=60)
    args = parser.parse_args()
    if subprocess.run(['pgrep', '-x', 'gzserver'], capture_output=True).returncode == 0:
        raise RuntimeError('An existing Gazebo is running; this test only controls its own new simulation')
    args.output.mkdir(parents=True, exist_ok=False)
    port, gazebo_port = unused_port(), unused_port()
    os.environ['ROS_MASTER_URI'] = f'http://127.0.0.1:{port}'
    os.environ['GAZEBO_MASTER_URI'] = f'http://127.0.0.1:{gazebo_port}'
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    environment = os.environ.copy()
    log = (args.output / 'launch.log').open('w')
    command = ['roslaunch', '-p', str(port), 'start_up', 'stage3_gazebo_rendezvous.launch',
               'matching_profile:=partial', 'gui:=false', 'rviz:=false', 'record:=false',
               'output_root:=' + str((args.output / 'run').resolve())]
    launch = subprocess.Popen(command, env=environment, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    (args.output / 'process.json').write_text(json.dumps(dict(
        launcher_pid=launch.pid, ros_master_uri=environment['ROS_MASTER_URI'],
        gazebo_master_uri=environment['GAZEBO_MASTER_URI'], command=command), indent=2) + '\n')
    report = dict(start_time=datetime.now(timezone(timedelta(hours=8))).isoformat(),
                  command=command, ros_master_uri=environment['ROS_MASTER_URI'],
                  scope='new owned Gazebo run, same world/spawns/sensors; ideal frontend, not real LIO',
                  truth_role='test positioning and independent scoring only; not a matcher hint',
                  requested_observation_sim_seconds=args.hold_sim_seconds, passed=False)
    report['scoring_window'] = 'startup, approach and 60 s hover, before test landing and shutdown'
    controller = None
    try:
        import rosgraph
        wait(lambda: launch.poll() is not None or rosgraph.is_master_online())
        if launch.poll() is not None:
            raise RuntimeError('Gazebo launch exited during startup')
        import rospy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from ehpr_bridge.msg import InterRobotMatch
        from stage1_uav_teleop import Teleop
        rospy.init_node('partial_gazebo_acceptance', anonymous=True, disable_signals=True)
        matches, status, poses, truth = [], [], {}, {}
        subscribers = [rospy.Subscriber('/stage3/match', InterRobotMatch,
                         lambda m: matches.append(m), queue_size=100),
                       rospy.Subscriber('/stage3/status', String,
                         lambda m: status.append(yaml.load(m.data, Loader=yaml.CSafeLoader)), queue_size=100)]
        for robot in ('ugv', 'uav'):
            subscribers.append(rospy.Subscriber('/algorithm_test/' + robot + '/odometry', Odometry,
                lambda m, robot=robot: poses.__setitem__(robot, m), queue_size=1))
            subscribers.append(rospy.Subscriber('/stage1/' + robot + '/ground_truth/lidar_odom', Odometry,
                lambda m, robot=robot: truth.__setitem__(robot, m), queue_size=1))
        wait(lambda: len(poses) == 2 and len(truth) == 2 and rospy.get_time() > 0)
        wait(lambda: bool(list((args.output / 'run/frontend').glob('*/frontend.json'))))
        frontend_path = next((args.output / 'run/frontend').glob('*/frontend.json'))
        wait(lambda: len(json.loads(frontend_path.read_text()).get('robots', {})) == 2)
        frontend = json.loads(frontend_path.read_text())
        world = {r: np.asarray(frontend['robots'][r]['T_world_reference']) for r in ('ugv', 'uav')}
        controller = Teleop(require_lio_ready=False)
        wait(lambda: controller.pose is not None and controller.state.connected)
        with controller.lock:
            controller.hold_current()
            controller.active = True
        start = rospy.get_time()
        wait(lambda: rospy.get_time() - start >= 3)
        if not controller.mode(custom_mode='OFFBOARD').mode_sent:
            raise RuntimeError('OFFBOARD request rejected')
        wait(lambda: controller.state.mode == 'OFFBOARD', 15)
        if not controller.arm(value=True).success:
            raise RuntimeError('Arming rejected')
        wait(lambda: controller.state.armed, 15)

        # The open corridor at X=1 avoids the walls in the unchanged SDF.
        waypoints = [(1., 0., .89), (1., 8.5, .89), (1.48, 9.06, .89)]
        for waypoint in waypoints:
            goal = (np.linalg.inv(world['uav']) @ np.r_[waypoint, 1])[:3]
            started, previous = time.monotonic(), rospy.get_time()
            while True:
                now = rospy.get_time()
                dt = max(0, min(.1, now - previous))
                previous = now
                p = poses['uav'].pose.pose.position
                actual = np.array([p.x, p.y, p.z])
                if np.linalg.norm(actual - goal) < .12:
                    break
                if time.monotonic() - started > 240:
                    raise RuntimeError('Vehicle failed to reach the recorded hover region')
                with controller.lock:
                    local = controller.pose.pose.position
                    desired = np.array([local.x, local.y, local.z]) + goal - actual
                    delta = desired - np.array(controller.target[:3])
                    if np.linalg.norm(delta) > 0:
                        delta *= min(1., .3 * dt / np.linalg.norm(delta))
                    controller.target[:3] = (np.array(controller.target[:3]) + delta).tolist()
                time.sleep(.05)
            print('WAYPOINT_REACHED', waypoint, flush=True)

        observed_start, next_update = rospy.get_time(), rospy.get_time()
        wall_deadline = time.monotonic() + 600
        while rospy.get_time() - observed_start < args.hold_sim_seconds:
            if launch.poll() is not None or time.monotonic() > wall_deadline:
                raise RuntimeError('Simulation stopped or stalled during the observation window')
            if rospy.get_time() >= next_update:
                print('OBSERVATION', round(rospy.get_time() - observed_start, 1),
                      'sim seconds; accepted', len(matches), flush=True)
                next_update = rospy.get_time() + 10
            time.sleep(.1)
        report['observed_sim_seconds'] = rospy.get_time() - observed_start
        report['accepted_messages'] = len(matches)
        report['worker_queue_drops'] = max((s.get('worker_queue_drops', 0) for s in status), default=0)
        report['positions'] = {}
        for robot, m in truth.items():
            p = m.pose.pose.position
            report['positions'][robot] = [p.x, p.y, p.z]
        scored = []
        for message in matches:
            m = yaml.load(Path(message.result_path).read_text(), Loader=yaml.CSafeLoader)
            expected = np.linalg.inv(world['uav'] @ np.asarray(m['query']['T_reference_key'])) @ (
                world['ugv'] @ np.asarray(m['reference']['T_reference_key']))
            actual = np.asarray(m['T_query_reference_refined'])
            scored.append(dict(path=message.result_path, query_id=m['query']['submap_id'],
                position_error_m=float(np.linalg.norm(actual[:3, 3] - expected[:3, 3])),
                rotation_error_deg=float(np.degrees(Rotation.from_matrix(
                    actual[:3, :3].T @ expected[:3, :3]).magnitude()))))
        report['matches'] = scored
        runs = list((args.output / 'run/matches').glob('*/events.yaml'))
        times = []
        for event_path in runs:
            events = list(yaml.load_all(event_path.read_text(), Loader=yaml.CSafeLoader))
            times.extend(e.get('diagnostics', {}).get('matching_processing_wall_s', 0)
                         for e in events if e.get('status') not in ('database_updated', 'duplicate'))
        report['processing_wall_s_p95'] = float(np.percentile(times, 95)) if times else 0
        report['file_checks'] = [verify(p.parent) for p in runs if list(p.parent.glob('match_*/match.yaml'))]
        report['passed'] = bool(scored) and report['worker_queue_drops'] == 0 and (
            report['processing_wall_s_p95'] <= 3) and all(
            m['position_error_m'] <= .2 and m['rotation_error_deg'] <= 3 for m in scored)
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if controller is not None and controller.state.armed:
            controller.request_land()
            try:
                wait(lambda: not controller.state.armed, 45)
            except Exception:
                pass
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=45)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                launch.wait(timeout=15)
        log.close()
        (args.output / 'checks.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('matches', 'file_checks')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
