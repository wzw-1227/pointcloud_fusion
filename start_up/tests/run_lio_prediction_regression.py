#!/usr/bin/env python3
"""Actual mapping-node regression: repeated feature loss must retain valid increments."""
import json
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import time
import socket
import rosgraph

def port():
    with socket.socket() as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]
def main():
    root=Path(__file__).resolve().parents[1]
    directory=root.parents[1]/'results/stability'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_lio_prediction_checks')
    directory.mkdir(exist_ok=False)
    env=os.environ.copy();env['ROS_MASTER_URI']='http://127.0.0.1:'+str(port());os.environ['ROS_MASTER_URI']=env['ROS_MASTER_URI']
    processes=[];logs=[];result={'status':'running','input':'synthetic cloud_info; actual compiled LIO map node; no vehicle controls'}
    def start(args,name):
        log=open(directory/(name+'.log'),'w');logs.append(log);proc=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT);processes.append(proc);return proc
    try:
        start(['roscore','-p',env['ROS_MASTER_URI'].rsplit(':',1)[1]],'master')
        deadline=time.monotonic()+30
        while not rosgraph.is_master_online() and time.monotonic()<deadline:time.sleep(.1)
        import rospy
        import yaml
        from lio_sam.msg import cloud_info,Scan2MapDebug
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointField
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header
        rospy.init_node('lio_prediction_regression',anonymous=True,disable_signals=True)
        params=yaml.safe_load((root/'config/stage1_uav_stable.yaml').read_text())['lio_sam']
        params.update(loopClosureEnableFlag=False,mappingProcessInterval=.01,numberOfCores=1)
        rospy.set_param('/uav/lio_sam',params)
        node=start(['rosrun','lio_sam','lio_sam_mapOptmization','__ns:=/uav'],'mapping')
        pub=rospy.Publisher('/uav/lio_sam/feature/cloud_info',cloud_info,queue_size=2)
        incremental=[];global_=[];debug=[]
        subscribers=[rospy.Subscriber('/uav/lio_sam/mapping/odometry_incremental',Odometry,lambda m:incremental.append(m),queue_size=100),
            rospy.Subscriber('/uav/lio_sam/mapping/odometry',Odometry,lambda m:global_.append(m),queue_size=100),
            rospy.Subscriber('/uav/lio_sam/debug/scan2map',Scan2MapDebug,lambda m:debug.append(m),queue_size=100)]
        deadline=time.monotonic()+20
        while pub.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.1)
        if not pub.get_num_connections():raise RuntimeError('mapping subscriber did not start')
        fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z','intensity'))]
        corners=[];surfaces=[]
        for axis in range(3):
            for side in (-3.,3.):
                for j in range(41):
                    pt=[side,side,side];pt[axis]=-2.+j*.1;corners.append(tuple(pt)+(0.,))
        for i in range(31):
            for j in range(31):
                x=-4.5+i*.3;y=-4.5+j*.3
                surfaces.extend([(x,y,-.7,0.),(-3.,x,y,0.),(x,3.,y,0.)])
        for i in range(22):
            t=rospy.Time.now();header=Header(stamp=t,frame_id='uav/laser_3d')
            msg=cloud_info(header=header,imuAvailable=True,odomAvailable=True,initialGuessX=.1*i)
            msg.cloud_corner=point_cloud2.create_cloud(header,fields,corners if i==0 else [])
            msg.cloud_surface=point_cloud2.create_cloud(header,fields,surfaces if i==0 else [])
            msg.cloud_deskewed=point_cloud2.create_cloud(header,fields,surfaces)
            before=len(incremental);pub.publish(msg)
            deadline=time.monotonic()+3
            while len(incremental)<=before and time.monotonic()<deadline and node.poll() is None:time.sleep(.02)
            if len(incremental)<=before:raise RuntimeError('missing incremental output after scan '+str(i))
            time.sleep(.03)
            if i==10:
                msg.header.stamp=rospy.Time.now();msg.initialGuessX=float('nan')
                count=len(incremental);pub.publish(msg);time.sleep(.2)
                if node.poll() is not None or len(incremental)!=count:raise RuntimeError('invalid prediction was published or crashed the node')
        import numpy as np
        x=np.array([m.pose.pose.position.x for m in incremental]);gx=np.array([m.pose.pose.position.x for m in global_])
        if not np.isfinite(x).all() or np.max(np.abs(x-gx))>1e-4:raise RuntimeError('incremental/global pose diverged during feature loss')
        if abs(x[-1]-2.0)>1e-4:raise RuntimeError('prediction-only increments did not advance correctly')
        if any(m.pose.covariance[0]!=1 for m in incremental[1:]):raise RuntimeError('prediction-only scan treated as strong correction')
        if any(m.optimization_performed for m in debug):raise RuntimeError('feature loss falsely marked optimized')
        # Restore enough input features: the node must continue safely rather
        # than crash on stale state; map support can still be geometrically weak.
        msg.header.stamp=rospy.Time.now();msg.initialGuessX=2.1
        msg.cloud_corner=point_cloud2.create_cloud(msg.header,fields,[(x-2.,y,z,a) for x,y,z,a in corners])
        msg.cloud_surface=point_cloud2.create_cloud(msg.header,fields,[(x-2.,y,z,a) for x,y,z,a in surfaces])
        before=len(incremental);pub.publish(msg);deadline=time.monotonic()+5
        while len(incremental)<=before and time.monotonic()<deadline and node.poll() is None:time.sleep(.02)
        if len(incremental)<=before or node.poll() is not None:raise RuntimeError('mapping did not survive feature recovery')
        p=incremental[-1].pose.pose.position
        if not np.isfinite([p.x,p.y,p.z]).all():raise RuntimeError('recovery produced nonfinite pose')
        deadline=time.monotonic()+2
        while (not debug or debug[-1].header.stamp!=msg.header.stamp) and time.monotonic()<deadline:time.sleep(.02)
        if not debug[-1].optimization_performed:raise RuntimeError('rich scene did not restore scan optimization')
        result.update(status='passed',prediction_only_frames=21,invalid_prediction_rejected=True,final_prediction_x=float(x[-1]),max_incremental_global_error=float(np.max(np.abs(x-gx))),recovery_optimization_performed=debug[-1].optimization_performed)
    except Exception as exc:result.update(status='failed',error=str(exc))
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
        for log in logs:log.close()
        (directory/'checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));return 0 if result['status']=='passed' else 1
if __name__=='__main__':raise SystemExit(main())
