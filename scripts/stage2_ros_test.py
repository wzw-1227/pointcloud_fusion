#!/usr/bin/env python3
"""Synthetic interface regression or read-only live-cloud check plus explicit flush."""
import argparse
import json
import math
import time
from pathlib import Path
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock
from ehpr_bridge.msg import RobotSubmap
from tf.transformations import quaternion_matrix, quaternion_from_euler
import yaml
from scipy.spatial import cKDTree


def xyz(message, sample=None):
    rows = np.array(list(point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=float)
    if sample and len(rows) > sample:
        rows = rows[np.linspace(0, len(rows)-1, sample, dtype=int)]
    return rows


def pose_matrix(pose):
    q = pose.orientation
    matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
    p = pose.position
    matrix[:3, 3] = [p.x, p.y, p.z]
    return matrix


class Checks:
    def __init__(self, mode):
        self.mode = mode
        self.config = yaml.safe_load(Path(rospy.get_param('/stage2_submap_builder/config')).read_text())
        self.submaps = {'ugv': [], 'uav': []}
        self.displays = {}
        self.raw = {'ugv': [], 'uav': []}
        self.subs = []
        for name in ('ugv', 'uav'):
            self.subs += [rospy.Subscriber('/stage2/'+name+'/submap', RobotSubmap, self.submap, callback_args=name),
                          rospy.Subscriber('/stage2/'+name+'/submap_cloud', PointCloud2, self.display, callback_args=name),
                          rospy.Subscriber(self.config['robots'][name]['cloud_topic'], PointCloud2, self.scan, callback_args=name, queue_size=1)]

    def submap(self, m, name):
        if not any(s.submap_id == m.submap_id and s.session_id == m.session_id for s in self.submaps[name]):
            self.submaps[name].append(m)

    def display(self, m, name):
        self.displays[name] = m

    def scan(self, m, name):
        self.raw[name].append(xyz(m, 128))
        self.raw[name] = self.raw[name][-200:]

    def wait(self, condition, timeout=30):
        deadline = time.monotonic()+timeout
        while not condition() and time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(.05)
        if not condition():
            raise RuntimeError('timed out waiting for Stage 2 condition')

    def flush(self):
        responses = {}
        for name in ('ugv', 'uav'):
            service = '/stage2/'+name+'/flush'
            rospy.wait_for_service(service, timeout=30)
            response = rospy.ServiceProxy(service, Trigger)()
            if not response.success:
                raise RuntimeError(name+': flush failed: '+response.message)
            responses[name] = response.message
        self.wait(lambda: all(self.submaps[n] and self.submaps[n][-1].pcd_path == responses[n]
                             and n in self.displays
                             and self.displays[n].header.seq == self.submaps[n][-1].header.seq
                             for n in ('ugv', 'uav')))
        return responses

    def inspect(self):
        report = {}
        for name in ('ugv', 'uav'):
            m = self.submaps[name][-1]
            self.wait(lambda: self.displays[name].header.seq == m.header.seq)
            display = self.displays[name]
            if display.header.stamp != rospy.Time(0):
                raise RuntimeError(name+': visualization must select latest TF')
            local = xyz(m.cloud_local)
            reference = xyz(display)
            matrix = pose_matrix(m.pose_in_lio_map)
            rebuilt = local @ matrix[:3, :3].T + matrix[:3, 3]
            if len(reference) != len(rebuilt) or not len(reference):
                raise RuntimeError(name+': local/display cloud sizes differ or empty')
            error = float(np.max(np.linalg.norm(reference-rebuilt, axis=1)))
            if error > 1e-4:
                raise RuntimeError(name+': local/display reconstruction mismatch')
            r = self.config['robots'][name]
            if m.robot_id != r['robot_id'] or m.header.frame_id != r['reference_frame'] or display.header.frame_id != r['reference_frame']:
                raise RuntimeError(name+': identity/frame isolation failed')
            result = {'robot_id': m.robot_id, 'reference_frame': m.header.frame_id,
                      'points': len(reference), 'pcd': m.pcd_path, 'partial': m.partial,
                      'display_reconstruction_max_error_m': error,
                      'automatic_submaps_observed': sum(not x.partial for x in self.submaps[name])}
            if self.mode == 'live':
                observed = np.concatenate(self.raw[name])
                distances = cKDTree(reference).query(observed)[0]
                # Official map spacing/cap can replace a point by another in the same voxel.
                bound = math.sqrt(3)*self.config['voxel_map']['voxel_size']+1e-3
                result['registered_sample_count'] = len(observed)
                result['registered_sample_max_nearest_distance_m'] = float(np.max(distances))
                if np.max(distances) > bound:
                    raise RuntimeError(name+': registered cloud is not covered by submap voxels')
            report[name] = result
        return report

    def synthetic(self):
        publishers = {}
        clock_pub = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)
        for name in ('ugv', 'uav'):
            r = self.config['robots'][name]
            publishers[name] = (rospy.Publisher(r['cloud_topic'], PointCloud2, queue_size=100),
                                rospy.Publisher(r['odometry_topic'], Odometry, queue_size=100))
        self.wait(lambda: all(p.get_num_connections() for pair in publishers.values() for p in pair))
        stamp = 100.
        def step(new_stamp, positions, duplicate=False, invalid=False, bad_cloud=None, yaw=None):
            clock_pub.publish(Clock(clock=rospy.Time.from_sec(new_stamp)))
            time.sleep(.03)
            for name in ('ugv', 'uav'):
                x = positions[name]
                r = self.config['robots'][name]
                height = 1 if name == 'ugv' else 2.5
                y = 0 if name == 'ugv' else 10
                points = [(x+dx*.25, y+dy*.25, height+dz*.25) for dx in range(-8, 9) for dy in range(-4, 5) for dz in (0, 4)]
                h = Header(stamp=rospy.Time.from_sec(new_stamp), frame_id=r['reference_frame'])
                c = point_cloud2.create_cloud_xyz32(h, points)
                if bad_cloud == 'frame':
                    c.header.frame_id = 'incorrect_reference'
                elif bad_cloud == 'empty':
                    c = point_cloud2.create_cloud_xyz32(h, [])
                elif bad_cloud == 'nonfinite':
                    c = point_cloud2.create_cloud_xyz32(h, [(float('nan'), 0, 0)])
                o = Odometry(header=h, child_frame_id=r['mapping_child'])
                o.pose.pose.position.x = x
                o.pose.pose.position.y = y
                o.pose.pose.position.z = height
                q = quaternion_from_euler(.04, -.03, .2 if yaw is None else yaw[name])
                o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = q
                if invalid:
                    o.pose.pose.orientation.w = 0
                publishers[name][0].publish(c)
                publishers[name][1].publish(o)
                if duplicate:
                    publishers[name][0].publish(c); publishers[name][1].publish(o)
            time.sleep(.03)
        for i in range(170):
            stamp = 100+i*.1
            step(stamp, {'ugv': i*.25, 'uav': i*.25}, duplicate=i == 20)
        self.wait(lambda: all(sum(not s.partial for s in self.submaps[n]) >= 2 for n in ('ugv', 'uav')))
        paths = self.flush()
        first_report = self.inspect()
        # Invalid quaternion must not generate a synchronized accepted frame.
        step(stamp+.1, {'ugv': 42.25, 'uav': 42.25}, invalid=True)
        step(stamp+.11, {'ugv': 42.25, 'uav': 42.25}, bad_cloud='frame')
        step(stamp+.12, {'ugv': 42.25, 'uav': 42.25}, bad_cloud='empty')
        step(stamp+.13, {'ugv': 42.25, 'uav': 42.25}, bad_cloud='nonfinite')
        step(stamp+.2, {'ugv': 47, 'uav': 47})  # jump -> new epoch
        step(stamp+.3, {'ugv': 47, 'uav': 47}, yaw={'ugv': 1.2, 'uav': .2})  # UGV-only rotation reset
        step(stamp+1.5, {'ugv': 47.1, 'uav': 47.1}, yaw={'ugv': 1.2, 'uav': .2})  # gap -> new epoch
        time.sleep(1.2)  # Count expired unmatched messages before resetting clock.
        step(10, {'ugv': 0, 'uav': 0})  # clock rollback -> new epoch
        time.sleep(1.2)
        self.flush()
        second_report = self.inspect()
        for name in ('ugv', 'uav'):
            if self.submaps[name][-1].epoch != (4 if name == 'ugv' else 3):
                raise RuntimeError(name+': translation/rotation/gap/clock reset or robot isolation failed')
        directory = Path(paths['ugv']).parent
        summary = yaml.safe_load((directory/'summary.yaml').read_text())
        for name in ('ugv', 'uav'):
            data = summary['robots'][name]
            if data['invalid_drops'] < 4 or not data['duplicate_or_late_drops'] or not data['unpaired_drops']:
                raise RuntimeError(name+': invalid/duplicate/unpaired inputs were not counted')
        counts = {name: {key: summary['robots'][name][key] for key in
                        ('matched_frames', 'invalid_drops', 'duplicate_or_late_drops', 'unpaired_drops', 'resets')}
                  for name in ('ugv', 'uav')}
        return directory, {'automatic_generation': first_report, 'reset_tail': second_report, 'input_counters': counts}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['synthetic', 'live'], required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('stage2_ros_checks', anonymous=True, disable_signals=True)
    checks = Checks(args.mode)
    if args.mode == 'synthetic':
        directory, details = checks.synthetic()
    else:
        checks.wait(lambda: all(len(checks.raw[n]) >= 20 for n in ('ugv', 'uav')), timeout=90)
        paths = checks.flush()
        directory = Path(paths['ugv']).parent
        details = checks.inspect()
    report = {'status': 'passed', 'mode': args.mode, 'details': details,
              'scope': 'both interface/geometry check; no manual route or inter-robot registration is inferred'}
    (directory/('ros_check_'+args.mode+'.json')).write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2))
    print('RESULT_DIR='+str(directory), flush=True)


if __name__ == '__main__':
    main()
