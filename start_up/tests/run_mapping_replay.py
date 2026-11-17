#!/usr/bin/env python3
"""Isolate scan-to-map using recorded IMU predictions, without simulator/control.

This is a diagnostic, not end-to-end LIO acceptance: predictions remain frozen
from the original run. Only raw lidar, raw IMU, and those predictions are fed in.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import rosbag
import rosgraph
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--observer', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--end', type=float, default=258.1)
    parser.add_argument('--start', type=float, default=0, help='Skip earlier input for an initialization-only comparison')
    parser.add_argument('--rate', type=float, default=.7)
    parser.add_argument('--imu-rpy-weight', type=float, choices=[.01,.1], help='Diagnostic parameter override, not a production choice')
    parser.add_argument('--features', type=Path, help='Reuse cached feature input instead of extraction')
    parser.add_argument('--closed-loop',action='store_true',help='Recompute IMU preintegration from raw data; no frozen predictions')
    args = parser.parse_args()
    if args.closed_loop and args.features:parser.error('closed-loop needs raw lidar and IMU, not cached predictions')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    probe = json.loads((args.run/'probe.json').read_text())
    recording = Path(probe['recording_directory'])
    params = yaml.safe_load((recording/'parameters_start.yaml').read_text())['uav']['lio_sam']
    if args.imu_rpy_weight is not None:params['imuRPYWeight']=args.imu_rpy_weight
    if args.closed_loop:params['publish_tf']=False  # The actual ParamServer key; only Fusion TF is disabled.
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:'+str(port)
    processes, logs = [], []

    def start(command, name):
        log = (output/(name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    saved = None
    try:
        start(['roscore', '-p', str(port)], 'master')
        deadline = time.monotonic()+20
        while not rosgraph.is_master_online() and time.monotonic()<deadline:
            time.sleep(.1)
        import rospy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, PointCloud2
        from nav_msgs.msg import Odometry
        from lio_sam.msg import cloud_info, Scan2MapDebug
        rospy.init_node('mapping_replay_diagnostic', anonymous=True, disable_signals=True)
        rospy.set_param('/use_sim_time', True)
        rospy.set_param('/uav/lio_sam', params)
        rospy.set_param('/diagnostic/output_directory', str(output))
        clock = rospy.Publisher('/clock', Clock, queue_size=100, latch=True)
        clock.publish(Clock(clock=rospy.Time.from_sec(177)))
        if not args.features:
            start(['rosrun','lio_sam','lio_sam_imageProjection','__ns:=uav','__name:=replay_projection'], 'projection')
            start(['rosrun','lio_sam','lio_sam_featureExtraction','__ns:=uav','__name:=replay_features'], 'features')
        if args.closed_loop:
            start(['rosrun','lio_sam','lio_sam_imuPreintegration','__ns:=uav','__name:=replay_preintegration'], 'preintegration')
        mapper = start([str(args.observer.resolve()),'__ns:=uav','__name:=replay_mapping'], 'mapping')
        pubs = ({'/uav/lio_sam/feature/cloud_info':rospy.Publisher('/uav/lio_sam/feature/cloud_info', cloud_info, queue_size=5)}
            if args.features else {
                '/stage1/uav/points':rospy.Publisher('/stage1/uav/points',PointCloud2,queue_size=5),
                '/iris_0/imu_gazebo':rospy.Publisher('/iris_0/imu_gazebo',Imu,queue_size=2000),
                '/uav/odometry/imu_incremental':rospy.Publisher('/uav/odometry/imu_incremental',Odometry,queue_size=2000)})
        if args.closed_loop:del pubs['/uav/odometry/imu_incremental']
        poses, diagnostics = [], []
        def pose_callback(m):
            p=m.pose.pose.position; q=m.pose.pose.orientation
            poses.append([m.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        def debug_callback(m):
            diagnostics.append([m.header.stamp.to_sec(),int(m.optimization_performed),int(m.is_degenerate),m.lambda_min])
        subscribers = [rospy.Subscriber('/uav/lio_sam/mapping/odometry',Odometry,pose_callback,queue_size=1000),
            rospy.Subscriber('/uav/lio_sam/debug/scan2map',Scan2MapDebug,debug_callback,queue_size=1000)]
        features_count = [0]
        if not args.features:
            saved = rosbag.Bag(str(output/'frozen_features.bag'), 'w', compression='lz4')
            def feature_callback(m):
                # Remove only unused display payload; feature coordinates and
                # prediction fields are preserved for fixed-input mapping replay.
                stored = copy.deepcopy(m)
                stored.cloud_deskewed = PointCloud2()
                stored.startRingIndex=[]; stored.endRingIndex=[]
                stored.pointColInd=[]; stored.pointRange=[]
                saved.write('/uav/lio_sam/feature/cloud_info', stored, m.header.stamp)
                features_count[0] += 1
            subscribers.append(rospy.Subscriber('/uav/lio_sam/feature/cloud_info',cloud_info,feature_callback,queue_size=100))
        deadline=time.monotonic()+25
        while any(p.get_num_connections()==0 for p in pubs.values()) and time.monotonic()<deadline:
            if mapper.poll() is not None:
                raise RuntimeError('mapping observer exited; see mapping.log')
            time.sleep(.1)
        if any(p.get_num_connections()==0 for p in pubs.values()):
            raise RuntimeError('missing replay input subscribers')
        time.sleep(1)
        paths = ([args.features] if args.features else sorted(recording.glob('data*.bag'),key=lambda p:int(p.stem.split('_')[-1])))
        first=None; wall_start=None; sent={topic:0 for topic in pubs}
        last_mapping_stamp=-1.0
        last_progress=time.monotonic(); done=False; truth_rows=[]
        evaluation_truth='/stage1/uav/ground_truth/lidar_odom'
        for path in paths:
            with rosbag.Bag(str(path)) as bag:
                for topic,m,t in bag.read_messages(topics=list(pubs)+([evaluation_truth] if args.closed_loop else [])):
                    if topic==evaluation_truth:
                        p=m.pose.pose.position;q=m.pose.pose.orientation
                        truth_rows.append([m.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                        continue  # Never publish evaluation truth to the replay.
                    if t.to_sec()<args.start:
                        continue
                    if t.to_sec()>args.end:
                        done=True; break
                    if first is None:
                        first=t.to_sec(); wall_start=time.monotonic()
                    target=wall_start+(t.to_sec()-first)/args.rate
                    delay=target-time.monotonic()
                    if delay>0 and not args.features:time.sleep(delay)
                    if mapper.poll() is not None:
                        raise RuntimeError('mapping observer stopped during replay')
                    clock.publish(Clock(clock=t)); pubs[topic].publish(m); sent[topic]+=1
                    if args.features:
                        # Wait for each processed scan, so fixed-feature trials
                        # cannot differ because the mapping callback queue drops.
                        stamp=m.header.stamp.to_sec()
                        if stamp-last_mapping_stamp >= params['mappingProcessInterval']:
                            deadline=time.monotonic()+10
                            while (not poses or poses[-1][0]<stamp-1e-8) and time.monotonic()<deadline:
                                time.sleep(.001)
                            if not poses or poses[-1][0]<stamp-1e-8:
                                raise RuntimeError('mapping did not acknowledge feature stamp '+str(stamp))
                            last_mapping_stamp=stamp
                        else:
                            time.sleep(.002)
                    if time.monotonic()-last_progress>15:
                        print(json.dumps({'stamp':t.to_sec(),'poses':len(poses),'sent':sent}),flush=True)
                        last_progress=time.monotonic()
            if done:break
        time.sleep(3)
        for subscriber in subscribers:subscriber.unregister()
        if saved is not None:saved.close(); saved=None
        import numpy as np
        np.savez_compressed(output/'traces.npz',poses=np.asarray(poses),debug=np.asarray(diagnostics),truth=np.asarray(truth_rows))
        scope=('closed-loop raw lidar/IMU replay; preintegration recomputed; no controller, simulator or truth input; offline diagnostic only'
            if args.closed_loop else 'scan-to-map with frozen original IMU predictions; no controller, simulator, truth input or full LIO acceptance')
        summary={'status':'completed','scope':scope,
            'recording_directory':str(recording),'master_uri':os.environ['ROS_MASTER_URI'],'sent':sent,'poses':len(poses),
            'features_cached':features_count[0],'start_requested':args.start,'end_requested':args.end,'rate':args.rate,'parameters':params}
        (output/'replay.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps({k:v for k,v in summary.items() if k!='parameters'}),flush=True)
    finally:
        if saved is not None:saved.close()
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.terminate();process.wait(timeout=5)
        for log in logs:log.close()


if __name__=='__main__':
    main()
