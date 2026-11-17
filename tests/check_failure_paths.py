#!/usr/bin/env python3
"""Inject failures only into a fresh isolated test run; never touch user exports."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
import xmlrpc.client
import rosgraph
import rospy
import yaml
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from ehpr_bridge.msg import RobotSubmap


def wait(condition, timeout=30):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        value = condition()
        if value:
            return value
        time.sleep(.05)
    raise RuntimeError('Failure check timed out')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['io_failure', 'loop_guard', 'shutdown'], required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args()
    rospy.init_node('stage2_failure_checks', anonymous=True, disable_signals=True)
    directory = wait(lambda: next(iter(args.output_root.glob('*/summary.yaml')), None)).parent
    def summary():
        return yaml.safe_load((directory/'summary.yaml').read_text())
    messages = {'ugv': [], 'uav': []}
    subscribers = [rospy.Subscriber('/stage2/'+name+'/submap', RobotSubmap, messages[name].append)
                   for name in ('ugv', 'uav')]
    if args.mode in ('io_failure', 'shutdown'):
        rospy.wait_for_service('/stage2/ugv/flush', timeout=30)
        cloud = rospy.Publisher('/lio_sam/mapping/cloud_registered_raw', PointCloud2, queue_size=1)
        odom = rospy.Publisher('/lio_sam/mapping/odometry', Odometry, queue_size=1)
        clock = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)
        wait(lambda: cloud.get_num_connections() and odom.get_num_connections())
        stamp = rospy.Time.from_sec(100)
        clock.publish(Clock(clock=stamp))
        time.sleep(.1)
        header = Header(stamp=stamp, frame_id='odom')
        o = Odometry(header=header, child_frame_id='odom_mapping')
        o.pose.pose.orientation.w = 1
        cloud.publish(point_cloud2.create_cloud_xyz32(header, [(0, 0, 0), (1, 1, 1)]))
        odom.publish(o)
        wait(lambda: summary().get('robots', {}).get('ugv', {}).get('matched_frames') == 1)
    if args.mode == 'shutdown':
        uav_cloud = rospy.Publisher('/uav/lio_sam/mapping/cloud_registered_raw', PointCloud2, queue_size=1)
        uav_odom = rospy.Publisher('/uav/lio_sam/mapping/odometry', Odometry, queue_size=1)
        wait(lambda: uav_cloud.get_num_connections() and uav_odom.get_num_connections())
        header = Header(stamp=stamp, frame_id='uav_lio_odom')
        o = Odometry(header=header, child_frame_id='uav_odom_mapping')
        o.pose.pose.orientation.w = 1
        uav_cloud.publish(point_cloud2.create_cloud_xyz32(header, [(0, 0, 0), (1, 1, 1)]))
        uav_odom.publish(o)
        wait(lambda: summary().get('robots', {}).get('uav', {}).get('matched_frames') == 1)
        if Path(rospy.get_param('/stage2_submap_builder/output_root')).resolve() != args.output_root.resolve():
            raise RuntimeError('Refusing to stop a bridge outside the owned test output root')
        uri = rosgraph.Master('/stage2_failure_checks').lookupNode('/stage2_submap_builder')
        code, _, pid = xmlrpc.client.ServerProxy(uri).getPid('/stage2_failure_checks')
        if code != 1:
            raise RuntimeError('Cannot identify the owned test bridge')
        os.kill(pid, signal.SIGINT)
        data = wait(lambda: summary() if summary().get('status') == 'stopped' else None)
        wait(lambda: all(messages.values()))
        for name in ('ugv', 'uav'):
            if len(messages[name]) != 1 or not messages[name][0].partial:
                raise RuntimeError('Shutdown did not publish exactly one partial tail per robot')
            if data['robots'][name]['exports'][0]['reason'] != 'shutdown':
                raise RuntimeError('Shutdown reason missing from export metadata')
        report = {'status': 'passed', 'mode': args.mode, 'observed_builder_status': 'stopped',
                  'shutdown_publications': {name: len(m) for name, m in messages.items()}}
        (directory/'shutdown_check.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        print('RESULT_DIR='+str(directory), flush=True)
        return
    if args.mode == 'io_failure':
        # A directory where the temporary PCD should be written forces open() failure.
        # All other run files remain writable, allowing a truthful failed summary.
        (directory/'ugv_submap_000.pcd.tmp').mkdir()
        try:
            response = rospy.ServiceProxy('/stage2/ugv/flush', Trigger)()
            if response.success:
                raise RuntimeError('Write failure incorrectly reported success')
        except rospy.ServiceException:
            pass
    data = wait(lambda: summary() if summary().get('status') == 'failed' else None)
    if args.mode == 'loop_guard' and 'loopClosureEnableFlag=false' not in data.get('error', ''):
        raise RuntimeError('Wrong failure reason for enabled LIO loop closure')
    if args.mode == 'io_failure' and 'ugv_submap_000.pcd' not in data.get('error', ''):
        raise RuntimeError('Wrong failure reason for PCD write failure')
    time.sleep(.2)
    if any(messages.values()) or list(directory.glob('*.pcd')) or list(directory.glob('*_submap_*.yaml')):
        raise RuntimeError('Failed export left a completed file or success publication')
    report = {'status': 'passed', 'mode': args.mode, 'observed_builder_status': 'failed',
              'failure_reason': data['error'], 'success_messages': sum(map(len, messages.values())),
              'scope': 'expected negative test, not a successful data-collection run'}
    (directory/'failure_check.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    print('RESULT_DIR='+str(directory), flush=True)


if __name__ == '__main__':
    main()
