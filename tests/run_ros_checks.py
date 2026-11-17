#!/usr/bin/env python3
"""Own isolated ROS masters for repeatable synthetic/live checks; no robot controls."""
import argparse
from datetime import datetime, timezone, timedelta
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import rosgraph
import yaml


def unused_port():
    with socket.socket() as connection:
        connection.bind(('127.0.0.1', 0))
        return connection.getsockname()[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['synthetic', 'live', 'io_failure', 'loop_guard', 'shutdown'], required=True)
    parser.add_argument('--hold', type=float, default=0, help='Keep live chain for GUI inspection, wall seconds')
    parser.add_argument('--rviz-snapshot', action='store_true', help='Inspect live RViz on the current display')
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    root = package.parents[1]/'results'/'stage2'
    name = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')+'_checks_'+args.mode
    directory = root/name
    directory.mkdir(parents=True)
    env = os.environ.copy()
    env['ROS_MASTER_URI'] = 'http://127.0.0.1:'+str(unused_port())
    env['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:'+str(unused_port())
    os.environ['ROS_MASTER_URI'] = env['ROS_MASTER_URI']
    processes, files = [], []
    result = {'mode': args.mode, 'master_uri': env['ROS_MASTER_URI'], 'status': 'running', 'robot': 'both'}
    def start(command, filename):
        log = open(directory/filename, 'w')
        files.append(log)
        p = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        processes.append(p)
        return p
    try:
        if args.mode != 'live':
            start(['roscore', '-p', env['ROS_MASTER_URI'].rsplit(':', 1)[1]], 'master.log')
            deadline = time.monotonic()+30
            while not rosgraph.is_master_online() and time.monotonic() < deadline:
                time.sleep(.2)
            master = rosgraph.Master('/stage2_check_driver')
            master.setParam('/use_sim_time', True)
            config = yaml.safe_load((package/'config/stage2.yaml').read_text())
            for r in config['robots'].values():
                prefix = ('' if r['lio_namespace'] == '/' else r['lio_namespace'])+'/lio_sam/'
                master.setParam(prefix+'loopClosureEnableFlag', args.mode == 'loop_guard')
                master.setParam(prefix+'odometryFrame', r['reference_frame'])
                master.setParam(prefix+'odomChildFrame', r['mapping_child'])
            launch = start(['roslaunch', 'ehpr_bridge', 'stage2_submaps.launch',
                            'output_root:='+str(directory/'exports')], 'launch.log')
        else:
            launch = start(['roslaunch', '-p', env['ROS_MASTER_URI'].rsplit(':', 1)[1], 'start_up',
                            'stage2_sim.launch', 'gui:=false', 'rviz:=false'], 'launch.log')
            deadline = time.monotonic()+40
            while not rosgraph.is_master_online() and launch.poll() is None and time.monotonic() < deadline:
                time.sleep(.2)
            import rospy
            from std_msgs.msg import String
            rospy.init_node('stage2_live_driver', anonymous=True, disable_signals=True)
            deadline = time.monotonic()+180
            last = None
            while time.monotonic() < deadline and launch.poll() is None:
                try:
                    health = json.loads(rospy.wait_for_message('/stage1/health', String, timeout=2).data)
                except rospy.ROSException:
                    continue
                if health['status'] != last:
                    print('LIO health:', health['status'], health['errors'], flush=True)
                    last = health['status']
                if health['status'] == 'ready':
                    break
                if health['status'] == 'error':
                    raise RuntimeError('LIO health failed: '+str(health['errors']))
            else:
                raise RuntimeError('Live LIO did not become ready')
        if args.mode in ('io_failure', 'loop_guard', 'shutdown'):
            test = start(['python3', str(package/'tests/check_failure_paths.py'), '--mode', args.mode,
                          '--output-root', str(directory/'exports')], 'ros_test.log')
        else:
            test = start(['rosrun', 'ehpr_bridge', 'stage2_ros_test.py', '--mode', args.mode], 'ros_test.log')
        code = test.wait(timeout=150)
        text = (directory/'ros_test.log').read_text()
        print(text, flush=True)
        if code:
            raise RuntimeError('ROS interface check exited '+str(code))
        line = next(line for line in text.splitlines() if line.startswith('RESULT_DIR='))
        result['export_directory'] = line.split('=', 1)[1]
        if args.mode not in ('io_failure', 'loop_guard'):
            code = subprocess.run(['rosrun', 'ehpr_bridge', 'stage2_verify_exports.py', result['export_directory']], env=env, timeout=30).returncode
            if code:
                raise RuntimeError('Export verification failed')
        if args.rviz_snapshot:
            if args.mode != 'live':
                raise RuntimeError('RViz snapshot requires --mode live')
            # Suppress RViz's modal EOL notice only in this owned inspection process.
            # Otherwise it can prevent latched submap displays from finishing loading.
            env['DISABLE_ROS1_EOL_WARNINGS'] = '1'
            rviz = start(['rosrun', 'rviz', 'rviz', '-d', str(package.parent/'start_up/rviz/stage2_submaps.rviz')], 'rviz.log')
            deadline = time.monotonic()+30
            window_id = None
            while time.monotonic() < deadline and rviz.poll() is None:
                tree = subprocess.run(['xwininfo', '-root', '-tree'], env=env, capture_output=True, text=True).stdout
                for line in tree.splitlines():
                    if 'stage2_submaps.rviz' in line:
                        candidate = line.strip().split()[0]
                        properties = subprocess.run(['xprop', '-id', candidate, '_NET_WM_PID'], env=env, capture_output=True, text=True).stdout
                        if str(rviz.pid) == properties.rsplit('=', 1)[-1].strip():
                            window_id = candidate
                            break
                if window_id:
                    break
                time.sleep(.2)
            if not window_id:
                raise RuntimeError('Cannot find the RViz window owned by this test')
            time.sleep(8)
            screenshot = Path(result['export_directory'])/'rviz_both.png'
            capture = 'from PyQt5.QtWidgets import QApplication; import sys; app=QApplication([]); ok=app.primaryScreen().grabWindow(int(sys.argv[1],16)).save(sys.argv[2]); sys.exit(0 if ok else 1)'
            subprocess.run(['python3', '-c', capture, window_id, str(screenshot)], env=env, check=True, timeout=15)
            result['rviz_screenshot'] = str(screenshot)
        result['status'] = 'passed'
        if args.hold:
            print('Inspection master: '+env['ROS_MASTER_URI']+'; exports: '+result['export_directory'], flush=True)
            deadline = time.monotonic()+args.hold
            while time.monotonic() < deadline:
                time.sleep(.2)
    except Exception as exc:
        result['status'] = 'failed'
        result['error'] = str(exc)
        print('FAIL:', exc, flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=10)
        for log in files:
            log.close()
        if result['status'] == 'passed' and args.mode not in ('io_failure', 'loop_guard'):
            try:
                code = subprocess.run(['rosrun', 'ehpr_bridge', 'stage2_verify_exports.py',
                                       result['export_directory']], env=env, timeout=30).returncode
                if code:
                    raise RuntimeError('Final export verification failed after shutdown')
                result['shutdown_exports_verified'] = True
            except Exception as exc:
                result['status'] = 'failed'
                result['error'] = str(exc)
        (directory/'checks.json').write_text(json.dumps(result, indent=2)+'\n')
    print('CHECKS_DIR='+str(directory), flush=True)
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
