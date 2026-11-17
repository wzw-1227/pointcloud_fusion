#!/usr/bin/env python3
"""Ideal-pose frontend for fusion-only testing; explicitly not a LIO estimator."""
from collections import Counter, deque
from datetime import datetime, timezone, timedelta
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix
import tf2_ros
from stage1_tools.cloud import cloud_array
from stage1_tools.contracts import load_contract
from stage1_tools.geometry import pose_matrix
from stage1_tools.ideal_frontend import TruthBuffer, initial_reference, register_scan


class Frontend:
    def __init__(self):
        if (not rospy.get_param('/algorithm_test/enabled', False) or
                rospy.get_param('/stage1/environment', '') != 'sim' or
                not rospy.get_param('/use_sim_time', False)):
            raise RuntimeError('Ideal frontend requires the explicit algorithm-only Gazebo entry')
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.stopping = False
        self.generation = 0
        self.clock = -1.
        self.queue_size = int(rospy.get_param('~queue_size', 20))
        self.maximum_age = float(rospy.get_param('~maximum_age', 1.0))
        self.truth_source = rospy.get_param('/stage1/ground_truth_source', 'receipt')
        if self.queue_size < 1 or self.maximum_age <= 0:
            raise ValueError('invalid frontend queue/age configuration')
        self.start_wall = time.monotonic()
        self.start_sim = None
        self.contract = load_contract(rospy.get_param('/stage1/contract_file'))
        self.minimum = float(rospy.get_param('~min_range', .5))
        self.maximum = float(rospy.get_param('~max_range', 30.))
        if not 0 < self.minimum < self.maximum:
            raise ValueError('invalid sensor range filter')
        root = Path(rospy.get_param('~output_root'))
        name = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')+'_'+str(os.getpid())
        self.directory = root/name
        self.directory.mkdir(parents=True, exist_ok=False)
        rospy.set_param('/algorithm_test/run_directory', str(self.directory))
        self.metadata = {'mode': 'gazebo_ideal_pose_algorithm_only', 'lio_acceptance': False,
                         'pose_source': 'Gazebo lidar truth, interpolated at original scan stamp',
                         'matcher_hints': 'none; EHPR coarse pose is the only GICP initial guess',
                         'visibility': 'Gazebo ray sensor returns; no world mesh sampling',
                         'point_time': 'instantaneous simulated scans only; deskew not tested',
                         'min_range_m': self.minimum, 'max_range_m': self.maximum,
                         'truth_max_interpolation_interval_s': .05,
                         'truth_source': self.truth_source, 'queue_size': self.queue_size,
                         'receive_buffer_bytes': 32*1024*1024, 'maximum_age_sim_s': self.maximum_age,
                         'robots': {}}
        self.truth = {r: TruthBuffer() for r in ('ugv', 'uav')}
        self.pending = {r: deque() for r in self.truth}
        self.references, self.last_scan = {}, {}
        self.stats = {r: Counter() for r in self.truth}
        self.latencies = {r: deque(maxlen=5000) for r in self.truth}
        self.durations = {r: deque(maxlen=5000) for r in self.truth}
        self.static = tf2_ros.StaticTransformBroadcaster()
        self.transforms = {}
        self.status = rospy.Publisher('/algorithm_test/status', String, queue_size=1, latch=True)
        self.clouds, self.odoms, self.subscribers = {}, {}, []
        topics = ['/clock', '/tf', '/tf_static', '/algorithm_test/status', '/stage3/status', '/stage3/test_phase',
                  '/stage3/match', '/fused_submap', '/stage3/match_cloud_ugv', '/stage3/match_cloud_uav',
                  '/rosout_agg', '/stage1/ugv/lidar_input_stats', '/stage1/uav/lidar_input_stats']
        for robot, contract in self.contract['robots'].items():
            base = '/algorithm_test/'+robot
            self.clouds[robot] = rospy.Publisher(base+'/cloud_registered', PointCloud2, queue_size=self.queue_size)
            self.odoms[robot] = rospy.Publisher(base+'/odometry', Odometry, queue_size=self.queue_size)
            self.subscribers.extend([
                rospy.Subscriber(contract['ground_truth'], Odometry, self.receive_truth,
                                 callback_args=robot, queue_size=500, tcp_nodelay=True),
                rospy.Subscriber(contract['lio_input_cloud'], PointCloud2, self.receive_scan,
                                 callback_args=robot, queue_size=self.queue_size, buff_size=32*1024*1024,
                                 tcp_nodelay=True)])
            topics.extend([contract['raw_cloud'], contract['lio_input_cloud'], contract['ground_truth'],
                           base+'/cloud_registered', base+'/odometry', '/stage2/'+robot+'/submap'])
        self.recorder = self.record_log = None
        if rospy.get_param('~record', True):
            self.record_log = open(self.directory/'recorder.log', 'w')
            self.recorder = subprocess.Popen(['rosbag', 'record', '--lz4', '--split', '--size=512',
                '--buffsize=256', '-O', str(self.directory/'sensors'), *topics],
                stdout=self.record_log, stderr=subprocess.STDOUT, start_new_session=True)
        rospy.on_shutdown(self.shutdown)
        self.workers = [threading.Thread(target=self.work, args=(r,), name='frontend_'+r, daemon=True)
                        for r in self.truth]
        for worker in self.workers:
            worker.start()
        self.status_worker = threading.Thread(target=self.report, name='frontend_report', daemon=True)
        self.status_worker.start()
        rospy.logwarn('ALGORITHM-ONLY TEST: Gazebo truth replaces LIO. Production localization is NOT being validated.')

    def receive_truth(self, message, robot):
        try:
            if message.header.frame_id != 'world':
                raise ValueError('truth frame must be world')
            with self.lock:
                self.check_clock()
                if message.header.stamp.to_sec() > rospy.get_time()+.05:
                    self.stats[robot]['future_truth_drops'] += 1
                    return
                self.truth[robot].add(message.header.stamp.to_sec(), pose_matrix(message.pose.pose))
                self.condition.notify_all()
        except ValueError as exc:
            rospy.logerr_throttle(5, str(exc))

    def receive_scan(self, message, robot):
        with self.lock:
            self.check_clock()
            self.stats[robot]['scans_received'] += 1
            if message.header.stamp.to_sec() > rospy.get_time()+.05:
                self.stats[robot]['future_scan_drops'] += 1
                return
            if len(self.pending[robot]) >= self.queue_size:
                self.pending[robot].popleft(); self.stats[robot]['queue_drops'] += 1
            self.pending[robot].append((message, time.monotonic()))
            self.condition.notify_all()

    def check_clock(self):
        now = rospy.get_time()
        if self.clock >= 0 and now < self.clock:
            self.generation += 1
            for r in self.truth:
                self.truth[r] = TruthBuffer()
                self.pending[r].clear()
                self.stats[r]['clock_resets'] += 1
            self.last_scan.clear()
        self.clock = now

    def work(self, robot):
        while not rospy.is_shutdown():
            with self.condition:
                if self.stopping:
                    return
                self.check_clock()
                if not self.pending[robot]:
                    self.condition.wait(.02)
                    continue
                message, received = self.pending[robot][0]
                stamp = message.header.stamp.to_sec()
                age = rospy.get_time()-stamp
                if age > self.maximum_age:
                    self.pending[robot].popleft(); self.stats[robot]['expired_drops'] += 1
                    continue
                if stamp <= self.last_scan.get(robot, -1):
                    self.pending[robot].popleft()
                    kind = 'duplicate_drops' if stamp == self.last_scan[robot] else 'out_of_order_drops'
                    self.stats[robot][kind] += 1
                    continue
                truth = self.truth[robot]
                if not truth.times or stamp > truth.times[-1]:
                    self.condition.wait(.01)
                    continue
                self.pending[robot].popleft()
                try:
                    world_pose = truth.at(stamp)
                except ValueError as exc:
                    self.stats[robot]['interpolation_drops'] += 1
                    self.stats[robot]['invalid_or_unsynchronized_drops'] += 1
                    rospy.logwarn_throttle(5, robot+': '+str(exc))
                    continue
                generation = self.generation
            started = time.monotonic()
            try:
                self.process(message, robot, world_pose, generation)
                with self.lock:
                    self.durations[robot].append(time.monotonic()-started)
                    self.stats[robot]['max_queue_wait_wall_s'] = max(
                        self.stats[robot]['max_queue_wait_wall_s'], started-received)
            except ValueError as exc:
                with self.lock:
                    self.stats[robot]['invalid_drops'] += 1
                    self.stats[robot]['invalid_or_unsynchronized_drops'] += 1
                rospy.logwarn_throttle(5, robot+': '+str(exc))
            except Exception as exc:
                rospy.logfatal(str(exc)); rospy.signal_shutdown(str(exc)); return

    def process(self, message, robot, world_pose, generation):
        stamp = message.header.stamp.to_sec()
        if stamp <= self.last_scan.get(robot, -1):
            with self.lock:
                kind = 'duplicate_drops' if stamp == self.last_scan[robot] else 'out_of_order_drops'
                self.stats[robot][kind] += 1
            return
        expected = self.contract['robots'][robot]['raw_frame']
        if message.header.frame_id != expected:
            raise ValueError('unexpected raw frame '+message.header.frame_id)
        array = cloud_array(message)
        if not len(array) or not {'x', 'y', 'z', 'time'}.issubset(array.dtype.names):
            raise ValueError('empty scan or missing XYZ/time')
        if not np.all(np.isfinite(array['time'])) or np.any(np.abs(array['time']) > 1e-8):
            raise ValueError('this diagnostic frontend only supports instantaneous simulated scans')
        if robot not in self.references:
            with self.lock:
                self.references[robot] = initial_reference(world_pose)
                self.metadata['robots'][robot] = {'T_world_reference': self.references[robot].tolist(),
                                                 'initial_scan_stamp': stamp}
            self.publish_reference(robot)
        points = np.column_stack([array[k] for k in ('x', 'y', 'z')])
        registered, pose, removed = register_scan(points, world_pose, self.references[robot], self.minimum, self.maximum)
        if not len(registered):
            raise ValueError('range-filtered scan is empty')
        frame = 'algorithm_'+robot+'_odom'
        cloud = PointCloud2()
        cloud.header.stamp = message.header.stamp; cloud.header.frame_id = frame
        cloud.height = 1; cloud.width = len(registered); cloud.is_dense = True
        cloud.fields = [PointField(name=k, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, k in enumerate(('x', 'y', 'z'))]
        cloud.point_step = 12; cloud.row_step = 12*cloud.width
        cloud.data = registered.astype('<f4').tobytes()
        odom = Odometry(); odom.header = cloud.header
        odom.child_frame_id = 'algorithm_'+robot+'_lidar'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = pose[:3, 3]
        q = quaternion_from_matrix(pose)
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
        with self.lock:
            if generation != self.generation:
                self.stats[robot]['generation_drops'] += 1; return
            if rospy.get_time()-stamp > self.maximum_age:
                self.stats[robot]['expired_drops'] += 1; return
        self.odoms[robot].publish(odom); self.clouds[robot].publish(cloud)
        with self.lock:
            self.last_scan[robot] = stamp
            self.stats[robot].update(published=1, points_removed=removed)
            self.stats[robot]['last_points'] = len(registered)
            self.stats[robot]['last_scan_stamp'] = stamp
            self.latencies[robot].append(max(0., rospy.get_time()-stamp))

    def publish_reference(self, robot):
        matrix = self.references[robot]
        tf = TransformStamped(); tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = 'algorithm_test_world'; tf.child_frame_id = 'algorithm_'+robot+'_odom'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = matrix[:3, 3]
        q = quaternion_from_matrix(matrix)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        with self.lock:
            self.transforms[robot] = tf
            transforms = list(self.transforms.values())
        self.static.sendTransform(transforms)

    def snapshot(self):
        with self.lock:
            counters = {r: dict(s) for r, s in self.stats.items()}
            for r in counters:
                counters[r]['queue_depth'] = len(self.pending[r])
                for name, data in [('processing_wall_s', self.durations[r]), ('output_latency_sim_s', self.latencies[r])]:
                    if data:
                        counters[r][name+'_p95'] = float(np.quantile(data, .95))
                        counters[r][name+'_max'] = float(max(data))
            status = dict(self.metadata, robots=dict(self.metadata['robots']), counters=counters,
                          directory=str(self.directory), stamp=rospy.get_time())
        if self.start_sim is None and any(c.get('published', 0) for c in counters.values()):
            self.start_sim = rospy.get_time(); self.start_wall = time.monotonic()
        elapsed = time.monotonic()-self.start_wall
        status['real_time_factor'] = ((rospy.get_time()-self.start_sim)/elapsed
                                      if self.start_sim is not None and elapsed > 0 else None)
        self.status.publish(String(data=json.dumps(status)))
        tmp = self.directory/'frontend.json.tmp'
        tmp.write_text(json.dumps(status, indent=2)+'\n'); tmp.replace(self.directory/'frontend.json')

    def report(self):
        try:
            while not self.stopping and not rospy.is_shutdown():
                if self.recorder and self.recorder.poll() is not None:
                    raise RuntimeError('algorithm test recorder exited unexpectedly')
                if (self.truth_source == 'stamped' and time.monotonic()-self.start_wall > 40 and
                        not all(self.truth[r].times for r in self.truth)):
                    raise RuntimeError('Missing source-stamped truth; no receipt-time fallback')
                self.snapshot()
                time.sleep(1)
        except Exception as exc:
            rospy.logfatal(str(exc)); rospy.signal_shutdown(str(exc))

    def shutdown(self):
        with self.condition:
            self.stopping = True; self.condition.notify_all()
        for worker in self.workers+[self.status_worker]:
            if worker is not threading.current_thread(): worker.join(timeout=5)
        try: self.snapshot()
        except Exception as exc: rospy.logerr('Final frontend snapshot: %s', exc)
        if self.recorder and self.recorder.poll() is None:
            os.killpg(self.recorder.pid, signal.SIGINT)
            try:
                self.recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self.recorder.pid, signal.SIGTERM); self.recorder.wait(timeout=3)
        if self.record_log:
            self.record_log.close()


if __name__ == '__main__':
    rospy.init_node('stage3_gazebo_frontend'); node = Frontend(); rospy.spin()
