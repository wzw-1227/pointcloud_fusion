#!/usr/bin/env python3
"""Real Gazebo ray scans + ideal frontend + configured official EHPR/GICP/fusion."""
import argparse
from datetime import datetime, timezone, timedelta
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import numpy as np
import rosgraph
import yaml
from stage1_tools.geometry import inverse_pose, rotation_error_degrees

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE.parent/'ehpr_bridge/scripts'))
from stage3_truth_check import TruthCheck
from stage3_verify_results import verify


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); return sock.getsockname()[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', action='store_true')
    parser.add_argument('--sensor', choices=['stable', 'wide'], default='stable')
    parser.add_argument('--matching-profile', choices=['legacy', 'enhanced'], default='legacy')
    parser.add_argument('--continuity', action='store_true', help='Require zero input/epoch faults, <=1 s output gaps, and <=0.5 s P95 latency')
    parser.add_argument('--correspondence-distance', type=float, default=1.)
    parser.add_argument('--duration', type=float, default=20., help='minimum simulated collection time')
    parser.add_argument('--timeout', type=float, default=240., help='wall-clock timeout')
    parser.add_argument('--world', type=Path, help='Optional dedicated physical Gazebo world')
    parser.add_argument('--max-range', type=float, default=30., help='Effective diagnostic scan range; raw sensor unchanged')
    parser.add_argument('--expect-no-match', action='store_true', help='Stationary disjoint-scan negative case')
    parser.add_argument('--ugv-x', type=float, default=-3.); parser.add_argument('--ugv-y', type=float, default=-3.)
    parser.add_argument('--uav-x', type=float, default=-2.2); parser.add_argument('--uav-y', type=float, default=-3.)
    args = parser.parse_args()
    if args.world: args.world = str(args.world.resolve())
    if subprocess.run(['pgrep', '-x', 'gzserver'], capture_output=True).returncode == 0:
        parser.error('Stop the existing Gazebo first; this test owns a single isolated instance')
    directory = PACKAGE.parents[1]/'results/stage3_gazebo_algorithm'/datetime.now(
        timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f_checks')
    directory.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy(); env['ROS_MASTER_URI'] = 'http://127.0.0.1:'+str(free_port())
    env['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:'+str(free_port())
    os.environ['ROS_MASTER_URI'] = env['ROS_MASTER_URI']
    config = yaml.safe_load((PACKAGE/'config/stage3_gazebo_algorithm_matching.yaml').read_text())
    config['matching_profile'] = args.matching_profile
    config['correspondence_distance'] = args.correspondence_distance
    config_file = directory/'matching_config.yaml'; config_file.write_text(yaml.safe_dump(config, sort_keys=False))
    result = {'status': 'running', 'mode': 'real_gazebo_rays_ideal_pose_frontend', 'lio_acceptance': False,
              'controls': 'none; neither UAV nor UGV driven', 'parameters': vars(args),
              'matcher_hint': 'none; truth used in frontend and separately for scoring', 'master': env['ROS_MASTER_URI']}
    processes, logs, matches, statuses, frontends, fused = [], [], [], [], [], []
    truth = None
    def start(command, name):
        log = open(directory/(name+'.log'), 'w'); logs.append(log)
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process); return process
    try:
        launch = start(['roslaunch', '-p', env['ROS_MASTER_URI'].rsplit(':', 1)[1], 'start_up',
                        'stage3_gazebo_algorithm.launch', 'gui:=false', 'rviz:=false', 'manual_control:=false',
                        'uav_sensor_profile:='+args.sensor, 'output_root:='+str(directory), 'config:='+str(config_file),
                        'ugv_x:='+str(args.ugv_x), 'ugv_y:='+str(args.ugv_y),
                        'uav_x:='+str(args.uav_x), 'uav_y:='+str(args.uav_y),
                        'max_range:='+str(args.max_range)] + (['world:='+args.world] if args.world else []), 'launch')
        deadline = time.monotonic()+40
        while not rosgraph.is_master_online() and time.monotonic() < deadline and launch.poll() is None:
            time.sleep(.1)
        if not rosgraph.is_master_online(): raise RuntimeError('ROS master did not start')
        import rospy
        from std_msgs.msg import String
        from sensor_msgs.msg import PointCloud2
        from ehpr_bridge.msg import InterRobotMatch, RobotSubmap
        rospy.init_node('gazebo_algorithm_check', anonymous=True, disable_signals=True)
        truth = TruthCheck(directory)
        submap_counts = {'ugv': 0, 'uav': 0}
        output_stamps = {'ugv': [], 'uav': []}
        def count_submap(_, robot): submap_counts[robot] += 1
        subscribers = [rospy.Subscriber('/stage3/match', InterRobotMatch, matches.append, queue_size=100),
                       rospy.Subscriber('/stage3/status', String, lambda m: statuses.append(yaml.safe_load(m.data))),
                       rospy.Subscriber('/algorithm_test/status', String, lambda m: frontends.append(json.loads(m.data))),
                       rospy.Subscriber('/fused_submap', PointCloud2, fused.append, queue_size=10)]
        subscribers.extend(rospy.Subscriber('/stage2/'+r+'/submap', RobotSubmap, count_submap,
                                          callback_args=r, queue_size=100) for r in submap_counts)
        from nav_msgs.msg import Odometry
        subscribers.extend(rospy.Subscriber('/algorithm_test/'+r+'/odometry', Odometry,
                           lambda m, robot: output_stamps[robot].append(m.header.stamp.to_sec()),
                           callback_args=r, queue_size=100, tcp_nodelay=True) for r in output_stamps)
        deadline = time.monotonic()+args.timeout; first_stamp = None; last_reason = None
        continuity_baseline = None
        memory_samples = []; last_memory_sample = 0.
        while time.monotonic() < deadline and launch.poll() is None:
            now = rospy.get_time()
            if args.continuity and time.monotonic()-last_memory_sample >= 5:
                sample = {'wall_time': time.monotonic(), 'sim_stamp': now, 'rss_kib': {}}
                for component in ('frontend', 'matches'):
                    for folder in (directory/component).glob('*'):
                        try:
                            pid = int(folder.name.rsplit('_', 1)[1])
                            fields = dict(line.split(':', 1) for line in Path('/proc/'+str(pid)+'/status').read_text().splitlines() if ':' in line)
                            sample['rss_kib'][component] = int(fields['VmRSS'].split()[0])
                        except (ValueError, FileNotFoundError, KeyError):pass
                memory_samples.append(sample); last_memory_sample = time.monotonic()
            if frontends and first_stamp is None and all(
                    frontends[-1]['counters'][robot].get('published', 0) for robot in ('ugv', 'uav')):
                first_stamp = now
                continuity_baseline = json.loads(json.dumps(frontends[-1]['counters']))
            if statuses:
                reason = str(statuses[-1].get('last_reason', ''))
                if reason != last_reason:
                    print('Matcher:', statuses[-1].get('last_status'), reason, flush=True); last_reason = reason
            if (matches or args.expect_no_match) and first_stamp is not None and now-first_stamp >= args.duration: break
            time.sleep(.2)
        result['frontend'] = frontends[-1] if frontends else None
        result['submap_counts'] = submap_counts
        if args.expect_no_match:
            elapsed = rospy.get_time()-first_stamp if first_stamp is not None else 0
            result['simulated_collection_seconds'] = elapsed
            if elapsed < args.duration or not all(submap_counts.values()):
                raise RuntimeError('Negative case did not collect both warmed-up submaps')
            if matches or fused:
                raise RuntimeError('Disjoint physical observations produced a match or fused message')
            origins = frontends[-1]['robots']
            a = np.array(origins['ugv']['T_world_reference'])[:3, 3]
            b = np.array(origins['uav']['T_world_reference'])[:3, 3]
            distance = float(np.linalg.norm(a-b))
            result['initial_scan_domains'] = {'origin_distance_m': distance, 'effective_range_m': args.max_range,
                                              'separation_lower_bound_m': distance-2*args.max_range}
            if distance-2*args.max_range <= config['overlap_radius']:
                raise RuntimeError('Test spawn/range does not guarantee disjoint scan domains')
            result.update(status='passed', match_count=0, fused_points=0)
        if not args.expect_no_match:
            if not matches:
                raise RuntimeError('No accepted physical-scene match; last status: '+str(statuses[-1] if statuses else None))
            result['simulated_collection_seconds'] = rospy.get_time()-first_stamp if first_stamp is not None else 0
            if result['simulated_collection_seconds'] < args.duration:
                raise RuntimeError('Simulation failed to reach the requested collection duration within the wall timeout')
            match = matches[-1]
            result['truth_all_matches'] = [truth.evaluate(m) for m in matches]
            for check in result['truth_all_matches']:
                check['source'] = 'Gazebo poses used for ideal frontend and relative-transform scoring; never a matcher initial guess'
            if any(check['status'] != 'passed' for check in result['truth_all_matches']):
                raise RuntimeError('At least one accepted match exceeded the relative truth error limits')
            result['truth'] = truth.evaluate(match)
            # This test deliberately supplies truth to the frontend; do not label it independent LIO validation.
            result['truth']['source'] = 'Gazebo poses used for ideal frontend and relative-transform scoring; never a matcher initial guess'
            if result['truth']['status'] != 'passed': raise RuntimeError('Relative pose exceeds 0.50 m / 5 deg')
            result['match_file'] = match.result_path
            result['export_directory'] = str(Path(match.result_path).parent.parent)
            result['file_checks'] = verify(result['export_directory'])
            if not fused or fused[-1].width*fused[-1].height == 0 or fused[-1].header.frame_id != 'algorithm_ugv_odom':
                raise RuntimeError('Expected nonempty fused cloud in algorithm_ugv_odom')
            result['fused_points'] = fused[-1].width*fused[-1].height
            result['frontend'] = frontends[-1] if frontends else None
            result['match_count'] = len(matches)
            result['quality'] = {'inliers': match.ransac_inliers, 'fitness': match.fitness,
                                 'overlap': match.overlap_smaller_cloud, 'residual_rmse': match.residual_rmse}
            if args.snapshot:
                source = yaml.safe_load((PACKAGE/'rviz/stage3_gazebo_algorithm.rviz').read_text())
                vm = source['Visualization Manager']; vm['Global Options']['Fixed Frame'] = 'algorithm_ugv_odom'
                vm['Views']['Current']['Distance'] = 38
                for display in vm['Displays']:
                    if display.get('Class') == 'rviz/PointCloud2':
                        enabled = display.get('Topic') == '/fused_submap'
                        display['Enabled'] = display['Value'] = enabled
                    if display.get('Class') == 'rviz/Grid': display['Reference Frame'] = 'algorithm_ugv_odom'
                path = directory/'fused.rviz'; path.write_text(yaml.safe_dump(source, sort_keys=False))
                rviz = start(['rosrun', 'rviz', 'rviz', '-d', str(path)], 'rviz')
                deadline = time.monotonic()+30; window = None
                while time.monotonic() < deadline and rviz.poll() is None:
                    tree = subprocess.run(['xwininfo', '-root', '-tree'], env=env, capture_output=True, text=True).stdout
                    for line in tree.splitlines():
                        if 'fused.rviz' not in line: continue
                        candidate = line.strip().split()[0]
                        prop = subprocess.run(['xprop', '-id', candidate, '_NET_WM_PID'], env=env, capture_output=True, text=True).stdout
                        if str(rviz.pid) == prop.rsplit('=', 1)[-1].strip(): window = candidate; break
                    if window: break
                    time.sleep(.2)
                if not window: raise RuntimeError('Owned RViz window did not start')
                time.sleep(5)
                screenshot = directory/'rviz_fused.png'
                code = 'from PyQt5.QtWidgets import QApplication; import sys; app=QApplication([]); sys.exit(0 if app.primaryScreen().grabWindow(int(sys.argv[1],16)).save(sys.argv[2]) else 1)'
                subprocess.run(['python3', '-c', code, window, str(screenshot)], env=env, check=True, timeout=15)
                result['screenshot'] = str(screenshot)
            result['status'] = 'passed'
    except KeyboardInterrupt:
        result['status'] = 'aborted'; result['error'] = 'Interrupted; no validation claim'
    except Exception as exc:
        result['status'] = 'failed'; result['error'] = str(exc); print('FAIL:', exc, flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=25)
                except subprocess.TimeoutExpired: process.terminate(); process.wait(timeout=10)
        for log in logs: log.close()
        if truth:
            for subscriber in truth.subscribers: subscriber.unregister()
            with truth.lock: truth.log.close()
        if args.expect_no_match and result['status'] == 'passed' and (matches or fused):
            result['status'] = 'failed'; result['error'] = 'A late match/fused message invalidated the negative case'
        if result['status'] == 'passed' and not args.expect_no_match:
            try:
                result['file_checks'] = verify(result['export_directory'])
                saved_checks = []
                for path in sorted(Path(result['export_directory']).glob('match_*/match.yaml')):
                    meta = yaml.safe_load(path.read_text())
                    expected = inverse_pose(truth.interpolate('uav', meta['query']['keyframe_timestamp'])) @ truth.interpolate('ugv', meta['reference']['keyframe_timestamp'])
                    actual = np.asarray(meta['T_query_reference_refined'])
                    position = float(np.linalg.norm(actual[:3, 3]-expected[:3, 3]))
                    rotation = rotation_error_degrees(expected, actual)
                    saved_checks.append({'match': str(path), 'position_error_m': position,
                                         'rotation_error_deg': rotation, 'passed': position <= .5 and rotation <= 5})
                result['all_saved_matches'] = saved_checks
                if not all(check['passed'] for check in saved_checks):
                    raise RuntimeError('A later accepted match failed the final truth audit')
            except Exception as exc:
                result['status'] = 'failed'; result['error'] = str(exc)
        if args.continuity:
            try:
                (directory/'memory_samples.json').write_text(json.dumps(memory_samples, indent=2)+'\n')
                files = list((directory/'frontend').glob('*/frontend.json'))
                final = json.loads(files[0].read_text()) if len(files) == 1 else result.get('frontend')
                if not final or final.get('truth_source') != 'stamped':
                    raise RuntimeError('Missing final source-stamped frontend record')
                audit = {'frontend': final['counters'], 'robots': {}, 'memory': {},
                         'startup_counters_before_measurement': continuity_baseline,
                         'measurement_start_sim_s': first_stamp}
                faults = []
                for component in ('frontend','matches'):
                    values = [s['rss_kib'][component]/1024 for s in memory_samples if component in s['rss_kib']]
                    if values:
                        n = max(1, len(values)//3)
                        growth = float(np.median(values[-n:])-np.median(values[n:2*n]))
                        audit['memory'][component] = {'peak_mib': max(values), 'last_mib': values[-1],
                            'late_minus_middle_median_mib': growth, 'samples': len(values)}
                        # Permit bounded warmup/caches; flag continuing large late growth.
                        if len(values)>=12 and growth>64:faults.append(component+': sustained RSS growth >64 MiB')
                for robot in ('ugv', 'uav'):
                    counters = final['counters'][robot]
                    stamps = output_stamps[robot]
                    gap = float(np.max(np.diff(stamps))) if len(stamps)>1 else float('inf')
                    audit['robots'][robot] = {'max_output_gap_sim_s': gap, 'samples': len(stamps)}
                    if gap > 1 or counters.get('output_latency_sim_s_p95', float('inf')) > .5:
                        faults.append(robot+': output gap/latency exceeds limit')
                    for key in ('queue_drops','expired_drops','expired_before_publish','interpolation_drops','clock_resets'):
                        baseline = (continuity_baseline or {}).get(robot, {}).get(key, 0)
                        delta = counters.get(key, 0)-baseline
                        if delta:faults.append(robot+': '+key+' during measurement='+str(delta))
                summaries = list((directory/'submaps').glob('*/summary.yaml'))
                if len(summaries) != 1:raise RuntimeError('Missing unique final submap summary')
                submaps = yaml.safe_load(summaries[0].read_text())
                audit['submap_summary'] = submaps
                for robot, entry in submaps['robots'].items():
                    if entry.get('epoch', 0) or entry.get('resets', 0):faults.append(robot+': unexpected epoch reset')
                audit['passed'] = not faults; audit['faults'] = faults
                result['continuity'] = audit
                if faults:raise RuntimeError('; '.join(faults))
            except Exception as exc:
                result['status'] = 'failed'; result['error'] = 'Continuity audit: '+str(exc)
        (directory/'matcher_status_history.yaml').write_text(yaml.safe_dump(statuses))
        (directory/'checks.json').write_text(json.dumps(result, indent=2)+'\n')
        (directory/'REPORT.md').write_text('# Gazebo algorithm-only check\n\nStatus: '+result['status']+
            '\n\nReal ray returns; ideal Gazebo-pose frontend. EHPR/GICP receives no truth initial guess. '
            'This does not validate LIO or flight stability.\n\nDetails: [checks.json](checks.json).\n')
    print('CHECKS_DIR='+str(directory), flush=True)
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__': raise SystemExit(main())
