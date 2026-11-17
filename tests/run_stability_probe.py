#!/usr/bin/env python3
"""Passive isolated Gazebo A/B probe. Never arms or commands a robot."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rosgraph
import yaml

def free_port():
    import socket
    with socket.socket() as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--physics',choices=['stable','legacy'],default='stable')
    p.add_argument('--sensor',choices=['stable','legacy','wide'],default='stable');p.add_argument('--duration',type=float,default=45)
    p.add_argument('--uav-config',type=Path);p.add_argument('--near-rich',action='store_true');p.add_argument('--config',type=Path);p.add_argument('--near',action='store_true');p.add_argument('--snapshot',action='store_true');a=p.parse_args()
    root=Path(__file__).resolve().parents[3]/'results/stability';directory=root/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+a.physics+'_'+a.sensor+'_passive')
    directory.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy();env['ROS_MASTER_URI']='http://127.0.0.1:'+str(free_port());env['GAZEBO_MASTER_URI']='http://127.0.0.1:'+str(free_port())
    os.environ['ROS_MASTER_URI']=env['ROS_MASTER_URI'];env['DISABLE_ROS1_EOL_WARNINGS']='1'
    command=['roslaunch','-p',env['ROS_MASTER_URI'].rsplit(':',1)[1],'start_up','stage3_sim.launch',
             'sim_profile:='+a.physics,'uav_sensor_profile:='+a.sensor,'gui:=false','rviz:=false','manual_control:=false',
             'record_output_root:='+str(directory/'recordings'),'output_root:='+str(directory/'matches'),'submap_output_root:='+str(directory/'submaps')]
    if a.uav_config:command+=['uav_config:='+str(a.uav_config.resolve())]
    if a.config:command+=['config:='+str(a.config.resolve())]
    if a.near and a.near_rich:p.error('choose one near preset')
    if a.near:command+=['ugv_x:=-1','ugv_y:=0','uav_x:=1','uav_y:=0']
    if a.near_rich:command+=['ugv_x:=-7','ugv_y:=-8.5','uav_x:=-5','uav_y:=-8.5']
    log=open(directory/'launch.log','w');process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT)
    result={'status':'running','physics':a.physics,'sensor':a.sensor,'near':a.near,'near_rich':a.near_rich,'launch_command':command,'controls':'none; on ground; not hover or motion acceptance'}
    try:
        deadline=time.monotonic()+40
        while not rosgraph.is_master_online() and time.monotonic()<deadline and process.poll() is None:time.sleep(.1)
        if not rosgraph.is_master_online():raise RuntimeError('owned ROS master did not start')
        import rospy
        from sensor_msgs.msg import Imu,PointCloud2
        from nav_msgs.msg import Odometry
        from lio_sam.msg import Scan2MapDebug
        from std_msgs.msg import String
        from ehpr_bridge.msg import InterRobotMatch,RobotSubmap
        from stage1_tools.geometry import pose_matrix,initial_alignment
        from stage1_tools.evaluation import trajectory_metrics,matched_poses
        rospy.init_node('stability_passive_probe',anonymous=True,disable_signals=True)
        samples={r:{'imu':[],'raw':[],'odom':[],'truth':[],'debug':[],'submaps':0} for r in ('ugv','uav')};status=[];matches=[];fused=[];subscribers=[]
        initial_poses={r:{k:[] for k in ('odom','truth')} for r in samples}
        contract=yaml.safe_load(Path(rospy.get_param('/stage1/contract_file')).read_text())
        start=[None]
        def capture(m,r,k):
            stamp=m.header.stamp.to_sec()
            if start[0] is None:start[0]=stamp
            if k in ('odom','truth'):initial_poses[r][k].append((stamp,pose_matrix(m.pose.pose)))
            if stamp<start[0]+15:return
            if k=='imu':
                q=m.orientation;acc=m.linear_acceleration;gyro=m.angular_velocity
                from tf.transformations import quaternion_matrix
                rot=quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3];xyz=np.array([acc.x,acc.y,acc.z]);world=rot@xyz-np.array([0,0,9.8])
                samples[r][k].append((stamp,xyz.tolist(),world.tolist(),[gyro.x,gyro.y,gyro.z]))
            elif k in ('odom','truth'):samples[r][k].append((stamp,pose_matrix(m.pose.pose)))
            elif k=='raw':samples[r][k].append((stamp,m.width*m.height))
            elif k=='debug':samples[r][k].append((stamp,m.optimization_performed,m.weak_direction_count,m.lambda_min))
        for r in samples:
            conf=contract['robots'][r]
            for k,key,typ in [('imu','imu',Imu),('raw','raw_cloud',PointCloud2),('odom','odometry',Odometry),('truth','ground_truth',Odometry),('debug','debug',Scan2MapDebug)]:
                subscribers.append(rospy.Subscriber(conf[key],typ,lambda m,r=r,k=k:capture(m,r,k),queue_size=1000 if k=='imu' else 100))
            subscribers.append(rospy.Subscriber('/stage2/'+r+'/submap',RobotSubmap,lambda m,r=r:samples[r].__setitem__('submaps',samples[r]['submaps']+1),queue_size=20))
        subscribers += [rospy.Subscriber('/stage3/status',String,lambda m:status.append(yaml.safe_load(m.data)),queue_size=10),
                        rospy.Subscriber('/stage3/match',InterRobotMatch,lambda m:matches.append(m),queue_size=10),
                        rospy.Subscriber('/fused_submap',PointCloud2,lambda m:fused.append(m.width*m.height),queue_size=10)]
        wall_start=time.monotonic();wall_deadline=wall_start+max(240,a.duration*8)
        while process.poll() is None and time.monotonic()<wall_deadline:
            if start[0] is not None and rospy.get_time()-start[0]>=a.duration:break
            time.sleep(.25)
        if process.poll() is not None:raise RuntimeError('launch exited before probe ended')
        if start[0] is None or rospy.get_time()-start[0]<a.duration:raise RuntimeError('clock/sensor timeout')
        result['robots']={}
        for r,s in samples.items():
            if not all(s[k] for k in ('imu','raw','odom','truth','debug')):raise RuntimeError(r+' missing probe stream')
            imu=np.array([row[2] for row in s['imu']]);dt=np.diff([row[0] for row in s['imu']]);rawdt=np.diff([row[0] for row in s['raw']]);ds=s['debug']
            pairs=matched_poses(initial_poses[r]['odom'],initial_poses[r]['truth'],.05)
            if not pairs:raise RuntimeError(r+' missing run-wide initial alignment')
            alignment=initial_alignment(pairs[0][1],pairs[0][2])
            result['robots'][r]={'imu_world_acc_mean':imu.mean(0).tolist(),'imu_world_acc_std':imu.std(0).tolist(),
                'imu_rate_hz':(len(dt))/(s['imu'][-1][0]-s['imu'][0][0]),'imu_median_dt':float(np.median(dt)),
                'imu_max_dt':float(max(dt)),'raw_rate_hz':len(rawdt)/(s['raw'][-1][0]-s['raw'][0][0]),
                'raw_median_points':float(np.median([n for _,n in s['raw']])),
                'raw_max_dt':float(max(rawdt)), 'trajectory':trajectory_metrics(s['odom'],s['truth'],.05,alignment),
                'alignment_origin':'first associated pose from the entire run, including warmup',
                'unoptimized_frames':sum(not row[1] for row in ds),'degenerate_fraction':sum(bool(row[2]) for row in ds)/len(ds),
                'submaps':s['submaps']}
        result.update(status='probe_complete',observed_sim_seconds=rospy.get_time()-start[0],
            wall_seconds=time.monotonic()-wall_start,matcher=status[-1] if status else None,
            matches=len(matches),fused_nonempty_messages=sum(n>0 for n in fused))
        if (a.near or a.near_rich) and matches:
            import sys
            sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ehpr_bridge/scripts'))
            from stage3_truth_check import TruthCheck
            checker=TruthCheck.__new__(TruthCheck)
            import threading
            from tf.transformations import quaternion_from_matrix
            checker.lock=threading.Lock();checker.samples={r:[(t,m[:3,3].tolist(),quaternion_from_matrix(m).tolist()) for t,m in samples[r]['truth']] for r in samples}
            result['match_truth']=[checker.evaluate(m) for m in matches if m.header.stamp.to_sec()>start[0]+15 and m.reference_stamp.to_sec()>start[0]+15]
        if a.snapshot:
            rviz_log=open(directory/'rviz.log','w')
            rviz=subprocess.Popen(['rosrun','rviz','rviz','-d',str(Path(__file__).resolve().parents[1]/'rviz/stage3_online.rviz'),'__name:=stability_rviz'],env=env,stdout=rviz_log,stderr=subprocess.STDOUT)
            try:
                window=None;deadline=time.monotonic()+40
                while time.monotonic()<deadline and rviz.poll() is None:
                    tree=subprocess.run(['xwininfo','-root','-tree'],env=env,capture_output=True,text=True).stdout
                    for line in tree.splitlines():
                        if 'stage3_online.rviz' not in line:continue
                        candidate=line.strip().split()[0]
                        prop=subprocess.run(['xprop','-id',candidate,'_NET_WM_PID'],env=env,capture_output=True,text=True).stdout
                        if str(rviz.pid)==prop.rsplit('=',1)[-1].strip():window=candidate;break
                    if window:break
                    time.sleep(.2)
                if not window:raise RuntimeError('owned RViz window did not appear')
                time.sleep(5)
                image=directory/'rviz_passive.png'
                capture_code='from PyQt5.QtWidgets import QApplication; import sys; app=QApplication([]); ok=app.primaryScreen().grabWindow(int(sys.argv[1],16)).save(sys.argv[2]); sys.exit(0 if ok else 1)'
                subprocess.run(['python3','-c',capture_code,window,str(image)],env=env,check=True,timeout=20)
                result['screenshot']=str(image)
            finally:
                if rviz.poll() is None:
                    rviz.send_signal(signal.SIGINT)
                    try:rviz.wait(timeout=15)
                    except subprocess.TimeoutExpired:rviz.terminate();rviz.wait(timeout=5)
                rviz_log.close()
    except Exception as exc:result.update(status='failed',error=str(exc))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=40)
            except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=10)
        log.close();(directory/'probe.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'directory':str(directory),'status':result['status'],'error':result.get('error')},ensure_ascii=False),flush=True)
    return 1 if result['status']=='failed' else 0
if __name__=='__main__':raise SystemExit(main())
