#!/usr/bin/env python3
"""Reject ideal-pose matching outside its explicit simulation entry."""
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import rosgraph
import yaml


def main():
    package = Path(__file__).resolve().parents[1]
    root = package.parents[1]/'results/stage3_gazebo_algorithm'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_guard_checks')
    root.mkdir(parents=True, exist_ok=False)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:'+str(port))
    os.environ['ROS_MASTER_URI'] = env['ROS_MASTER_URI']
    processes, logs, checks = [], [], {}
    def start(command, name):
        log = open(root/(name+'.log'), 'w'); logs.append(log)
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process); return process
    try:
        start(['roscore', '-p', str(port)], 'master')
        deadline = time.monotonic()+20
        while not rosgraph.is_master_online() and time.monotonic() < deadline: time.sleep(.1)
        if not rosgraph.is_master_online(): raise RuntimeError('master startup timeout')
        import rospy
        rospy.init_node('ideal_mode_guard_check', anonymous=True, disable_signals=True)
        config = package/'config/stage3_gazebo_algorithm_matching.yaml'
        for case in ('not_enabled', 'real_clock', 'wrong_frames', 'valid_diagnostic'):
            rospy.set_param('/algorithm_test/enabled', case != 'not_enabled')
            rospy.set_param('/stage1/environment', 'sim')
            rospy.set_param('/use_sim_time', case != 'real_clock')
            chosen = config if case != 'wrong_frames' else package.parent/'ehpr_bridge/config/stage3.yaml'
            output = root/case
            process = start(['rosrun', 'ehpr_bridge', 'stage3_matching_node', '_mode:=ideal_frontend',
                             '_config:='+str(chosen), '_output_root:='+str(output)], case)
            deadline = time.monotonic()+15
            if case == 'valid_diagnostic':
                while time.monotonic() < deadline:
                    if process.poll() is not None: raise RuntimeError('valid diagnostic exited')
                    if list(output.glob('*/summary.yaml')): break
                    time.sleep(.1)
                else: raise RuntimeError('valid diagnostic did not initialize')
                time.sleep(.5)
                process.send_signal(signal.SIGINT); process.wait(timeout=10)
                if process.returncode: raise RuntimeError('valid diagnostic failed to stop')
                summary = yaml.safe_load(next(output.glob('*/summary.yaml')).read_text())
                if summary['status'] != 'stopped': raise RuntimeError('valid diagnostic summary invalid')
            else:
                process.wait(timeout=15)
                summary = yaml.safe_load(next(output.glob('*/summary.yaml')).read_text())
                if process.returncode == 0 or summary['status'] != 'failed':
                    raise RuntimeError('unsafe ideal mode was not rejected: '+case)
            checks[case] = {'status': 'passed', 'matcher_exit_code': process.returncode}
    except Exception as exc:
        checks['error'] = {'status': 'failed', 'reason': str(exc)}
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT); process.wait(timeout=15)
        for log in logs: log.close()
        (root/'checks.json').write_text(json.dumps(checks, indent=2)+'\n')
    print('CHECKS_DIR='+str(root), flush=True)
    return 1 if 'error' in checks else 0


if __name__ == '__main__': raise SystemExit(main())
