#!/usr/bin/env python3
"""Exercise the real worker loop without a ROS master or moving a robot."""
import importlib.util
from pathlib import Path
from collections import Counter, deque
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from stage1_tools.ideal_frontend import TruthBuffer

spec=importlib.util.spec_from_file_location('frontend',Path(__file__).parents[1]/'scripts/stage3_gazebo_frontend.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class WorkerTests(unittest.TestCase):
    def setUp(self):
        logger=patch.object(rospy,'logwarn_throttle');logger.start();self.addCleanup(logger.stop)
    def node(self):
        n=module.Frontend.__new__(module.Frontend)
        n.lock=threading.RLock();n.condition=threading.Condition(n.lock)
        n.stopping=False;n.generation=0;n.clock=-1;n.maximum_age=1;n.queue_size=20
        n.truth={'ugv':TruthBuffer(),'uav':TruthBuffer()}
        for b in n.truth.values():
            for s in np.arange(1.,2.01,.01):b.add(float(s),np.eye(4))
        n.pending={r:deque() for r in n.truth};n.stats={r:Counter() for r in n.truth}
        n.durations={r:deque(maxlen=5000) for r in n.truth};n.last_scan={}
        return n
    def cloud(self,stamp):
        m=PointCloud2();m.header.stamp=rospy.Time.from_sec(stamp);return m
    def test_processing_does_not_hold_shared_lock_or_block_other_robot(self):
        n=self.node();started=threading.Event();release=threading.Event();done=threading.Event()
        def process(m,r,p,g):
            if r=='uav':started.set();release.wait(2)
            else:done.set()
            n.last_scan[r]=m.header.stamp.to_sec()
        n.process=process
        with patch.object(rospy,'get_time',return_value=1.5),patch.object(rospy,'is_shutdown',return_value=False):
            threads=[threading.Thread(target=n.work,args=(r,),daemon=True) for r in n.truth]
            for t in threads:t.start()
            n.receive_scan(self.cloud(1.1),'uav');self.assertTrue(started.wait(1))
            n.receive_scan(self.cloud(1.2),'ugv');self.assertTrue(done.wait(1))
            self.assertTrue(n.lock.acquire(timeout=.1));n.lock.release()
            release.set();n.stopping=True
            with n.condition:n.condition.notify_all()
            for t in threads:t.join(1)
    def test_burst_overflow_expiry_and_clock_reset(self):
        n=self.node()
        with patch.object(rospy,'get_time',return_value=1.5):
            for i in range(25):n.receive_scan(self.cloud(1+i*.01),'uav')
        self.assertEqual(len(n.pending['uav']),20);self.assertEqual(n.stats['uav']['queue_drops'],5)
        with patch.object(rospy,'get_time',return_value=.5):
            with n.lock:n.check_clock()
        self.assertEqual(n.generation,1);self.assertFalse(n.pending['uav']);self.assertFalse(n.truth['uav'].times)
    def test_half_second_backlog_recovers_without_dropping_valid_frames(self):
        n=self.node();seen=[];n.process=lambda m,r,p,g:seen.append(m.header.stamp.to_sec())
        with patch.object(rospy,'get_time',return_value=1.6),patch.object(rospy,'is_shutdown',return_value=False):
            for s in [1.1,1.2,1.3,1.4,1.5]:n.receive_scan(self.cloud(s),'uav')
            t=threading.Thread(target=n.work,args=('uav',),daemon=True);t.start()
            deadline=time.monotonic()+1
            while len(seen)<5 and time.monotonic()<deadline:time.sleep(.01)
            n.stopping=True
            with n.condition:n.condition.notify_all()
            t.join(1)
        np.testing.assert_allclose(seen,[1.1,1.2,1.3,1.4,1.5],atol=1e-8)
        self.assertEqual(n.stats['uav']['expired_drops'],0)
    def test_over_limit_age_and_missing_truth_are_rejected(self):
        n=self.node();n.process=lambda *a:self.fail('invalid scan reached processing')
        with patch.object(rospy,'get_time',return_value=1.6),patch.object(rospy,'is_shutdown',return_value=False):
            n.receive_scan(self.cloud(.4),'uav')
            n.truth['uav']=TruthBuffer();n.truth['uav'].add(1.1,np.eye(4));n.truth['uav'].add(1.3,np.eye(4))
            n.receive_scan(self.cloud(1.2),'uav');t=threading.Thread(target=n.work,args=('uav',),daemon=True);t.start()
            deadline=time.monotonic()+1
            while n.stats['uav']['interpolation_drops']<1 and time.monotonic()<deadline:time.sleep(.01)
            n.stopping=True
            with n.condition:n.condition.notify_all()
            t.join(1)
        self.assertEqual(n.stats['uav']['expired_drops'],1);self.assertEqual(n.stats['uav']['interpolation_drops'],1)
    def test_duplicate_and_out_of_order_have_separate_counters(self):
        n=self.node();n.last_scan['uav']=1.3
        n.process=lambda *a:self.fail('duplicate/old scan reached processing')
        with patch.object(rospy,'get_time',return_value=1.6),patch.object(rospy,'is_shutdown',return_value=False):
            n.receive_scan(self.cloud(1.3),'uav');n.receive_scan(self.cloud(1.2),'uav')
            t=threading.Thread(target=n.work,args=('uav',),daemon=True);t.start()
            deadline=time.monotonic()+1
            while n.pending['uav'] and time.monotonic()<deadline:time.sleep(.01)
            n.stopping=True
            with n.condition:n.condition.notify_all()
            t.join(1)
        self.assertEqual(n.stats['uav']['duplicate_drops'],1)
        self.assertEqual(n.stats['uav']['out_of_order_drops'],1)

    def test_previous_epoch_future_data_is_rejected_after_clock_reset(self):
        n=self.node()
        with patch.object(rospy,'get_time',return_value=1.6):n.receive_scan(self.cloud(1.2),'uav')
        with patch.object(rospy,'get_time',return_value=.5):n.receive_scan(self.cloud(1.4),'uav')
        self.assertEqual(n.generation,1);self.assertFalse(n.pending['uav'])
        self.assertEqual(n.stats['uav']['future_scan_drops'],1)

if __name__=='__main__':unittest.main()
