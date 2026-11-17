#!/usr/bin/env python3
"""Identical recorded sensor inputs, fresh LIO instances, independent gravity-weight comparison."""
import argparse
from datetime import datetime
import json,os,signal,socket,subprocess,time
from pathlib import Path
import numpy as np
import rosbag,rosgraph,yaml
from stage1_tools.geometry import pose_matrix,initial_alignment
from stage1_tools.evaluation import trajectory_metrics,matched_poses

def port():
    with socket.socket() as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--weight',type=float,required=True);a=p.parse_args()
    source=Path(__file__).resolve().parents[1]
    root=source.parents[1]/'results/stability'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_rpy_'+str(a.weight)+'_replay');root.mkdir(exist_ok=False)
    data=[];truth=[]
    for path in sorted(a.directory.glob('data*.bag')):
        with rosbag.Bag(str(path)) as b:
            for topic,msg,t in b.read_messages(topics=['/stage1/uav/points','/iris_0/imu_gazebo','/stage1/uav/ground_truth/lidar_odom']):
                if 'ground_truth' in topic:truth.append((msg.header.stamp.to_sec(),pose_matrix(msg.pose.pose)))
                else:data.append((t.to_sec(),topic,msg))
    data.sort(key=lambda r:r[0]);config=yaml.safe_load((source/'config/stage1_uav_stable.yaml').read_text());config['lio_sam']['imuRPYWeight']=a.weight
    cfg=root/'config.yaml';cfg.write_text(yaml.safe_dump(config,sort_keys=False))
    env=os.environ.copy();env['ROS_MASTER_URI']='http://127.0.0.1:'+str(port());os.environ['ROS_MASTER_URI']=env['ROS_MASTER_URI']
    processes=[];logs=[];result={'status':'running','weight':a.weight,'source_directory':str(a.directory),'sensor_messages':len(data),'controls':'none; offline replay only'}
    def start(args,name):
        log=open(root/(name+'.log'),'w');logs.append(log);proc=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT);processes.append(proc);return proc
    try:
        start(['roscore','-p',env['ROS_MASTER_URI'].rsplit(':',1)[1]],'master');deadline=time.monotonic()+30
        while not rosgraph.is_master_online() and time.monotonic()<deadline:time.sleep(.1)
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu,PointCloud2
        from rosgraph_msgs.msg import Clock
        from lio_sam.msg import Scan2MapDebug
        rospy.init_node('rpy_replay_check',anonymous=True,disable_signals=True)
        rospy.set_param('/use_sim_time',True);rospy.set_param('/stage1/contract_file',str(source/'config/stage1_stable_contract.yaml'));rospy.set_param('/stage1/environment','sim');rospy.set_param('/stage1/robot','uav')
        clock=rospy.Publisher('/clock',Clock,queue_size=10,latch=True);clock.publish(Clock(clock=rospy.Time.from_sec(data[0][0])))
        start(['rosrun','start_up','stage1_sensor_tf.py','_robot:=uav'],'sensor_tf')
        start(['rosrun','tf2_ros','static_transform_publisher','0','0','.08','0','0','0','uav/base_link','uav/laser_3d'],'lidar_tf')
        launch=start(['roslaunch','start_up','stage1_lio.launch','namespace:=uav','robot:=uav','loop_closure:=false','config_file:='+str(cfg)],'lio')
        publishers={'/stage1/uav/points':rospy.Publisher('/stage1/uav/points',PointCloud2,queue_size=10),'/iris_0/imu_gazebo':rospy.Publisher('/iris_0/imu_gazebo',Imu,queue_size=1000)}
        poses=[];debug=[]
        subscribers=[rospy.Subscriber('/uav/lio_sam/mapping/odometry',Odometry,lambda m:poses.append((m.header.stamp.to_sec(),pose_matrix(m.pose.pose))),queue_size=1000),
            rospy.Subscriber('/uav/lio_sam/debug/scan2map',Scan2MapDebug,lambda m:debug.append((m.header.stamp.to_sec(),m.optimization_performed,m.is_degenerate)),queue_size=1000)]
        deadline=time.monotonic()+30
        while any(pub.get_num_connections()==0 for pub in publishers.values()) and time.monotonic()<deadline:time.sleep(.1)
        if any(pub.get_num_connections()==0 for pub in publishers.values()):raise RuntimeError('LIO sensor subscribers missing')
        time.sleep(1)
        previous=data[0][0]
        for stamp,topic,msg in data:
            delay=max(0,stamp-previous)*2
            if delay:time.sleep(min(delay,.2))
            clock.publish(Clock(clock=rospy.Time.from_sec(stamp)));publishers[topic].publish(msg);previous=stamp
            if launch.poll() is not None:raise RuntimeError('LIO exited during replay')
        time.sleep(3)
        pairs=matched_poses(poses,truth,.05)
        if not pairs:raise RuntimeError('no output associated with independent truth')
        alignment=initial_alignment(pairs[0][1],pairs[0][2]);cutoff=data[0][0]+15
        metric=trajectory_metrics([(t,m) for t,m in poses if t>=cutoff],truth,.05,alignment)
        ds=[v for v in debug if v[0]>=cutoff]
        result.update(status='replay_complete',trajectory=metric,degenerate_fraction=sum(v[2] for v in ds)/len(ds) if ds else None,
            unoptimized_frames=sum(not v[1] for v in ds),alignment=alignment.tolist(),motion_acceptance='not performed; do not promote the candidate from this ground replay alone')
    except Exception as exc:result.update(status='failed',error=str(exc))
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=20)
                except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
        for log in logs:log.close()
        (root/'checks.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'directory':str(root),'status':result['status'],'error':result.get('error'),'trajectory':result.get('trajectory')},ensure_ascii=False));return 1 if result['status']=='failed' else 0
if __name__=='__main__':raise SystemExit(main())
