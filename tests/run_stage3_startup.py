#!/usr/bin/env python3
"""Passive both-mode SITL check of initial observations and unfused startup.

Starts an owned isolated simulation. Does not arm, drive, fly or call flush.
"""
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rosgraph
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_ros_checks import unused_port
from run_stage3_ros import wait_for


def main():
    package=Path(__file__).resolve().parents[1]
    directory=package.parents[1]/'results/stage3'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_startup_checks')
    directory.mkdir(parents=True)
    env=os.environ.copy();env['ROS_MASTER_URI']='http://127.0.0.1:'+str(unused_port())
    env['GAZEBO_MASTER_URI']='http://127.0.0.1:'+str(unused_port())
    env['DISABLE_ROS1_EOL_WARNINGS']='1';os.environ['ROS_MASTER_URI']=env['ROS_MASTER_URI']
    processes=[];logs=[];result={'status':'running','inputs':'actual Gazebo and both LIO nodes',
        'controls':'none; no arming, driving, flight or flush','checks':'startup only; not flight acceptance'}
    def start(command,name):
        log=open(directory/(name+'.log'),'w');logs.append(log)
        p=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT);processes.append(p);return p
    try:
        launch=start(['roslaunch','-p',env['ROS_MASTER_URI'].rsplit(':',1)[1],'start_up','stage3_sim.launch',
            'gui:=false','rviz:=false','manual_control:=false','output_root:='+str(directory/'matches'),
            'submap_output_root:='+str(directory/'submaps')],'launch')
        wait_for(rosgraph.is_master_online,40)
        import rospy
        import tf2_ros
        from sensor_msgs.msg import PointCloud2
        from ehpr_bridge.msg import RobotSubmap
        rospy.init_node('stage3_startup_checks',anonymous=True,disable_signals=True)
        observed={'ugv':[],'uav':[]};submaps={'ugv':[],'uav':[]};fused=[]
        subscribers=[]
        for r,prefix in (('ugv',''),('uav','/uav')):
            subscribers.append(rospy.Subscriber(prefix+'/lio_sam/mapping/cloud_registered_raw',PointCloud2,
                lambda m,r=r:observed[r].append((m.header.frame_id,m.width*m.height)),queue_size=2))
            subscribers.append(rospy.Subscriber('/stage2/'+r+'/submap',RobotSubmap,
                lambda m,r=r:submaps[r].append((m.partial,m.header.stamp.to_sec())),queue_size=10))
        subscribers.append(rospy.Subscriber('/fused_submap',PointCloud2,lambda m:fused.append(m.width*m.height),queue_size=10))
        buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
        wait_for(lambda:submaps['ugv'] and all(observed.values()),180)
        assert not fused
        wait_for(lambda:submaps['uav'],30)
        for r,frame in (('ugv','odom'),('uav','uav_lio_odom')):
            assert observed[r][-1][0]==frame and observed[r][-1][1]>0
            wait_for(lambda frame=frame:buffer.can_transform('viz_world',frame,rospy.Time(0)),15)
        result['startup']={'observed_counts':{r:observed[r][-1][1] for r in observed},
            'ugv_preview_count':len(submaps['ugv']),'uav_preview_count':len(submaps['uav']),'fusion_messages':0,
            'tf_to_viz_world':'both available; visualization only'}
        rviz=start(['rosrun','rviz','rviz','-d',str(package.parent/'start_up/rviz/stage3_online.rviz')],'rviz')
        deadline=time.monotonic()+30;window=None
        while time.monotonic()<deadline and rviz.poll() is None:
            tree=subprocess.run(['xwininfo','-root','-tree'],env=env,capture_output=True,text=True).stdout
            for line in tree.splitlines():
                if 'stage3_online.rviz' not in line:continue
                candidate=line.strip().split()[0]
                prop=subprocess.run(['xprop','-id',candidate,'_NET_WM_PID'],env=env,capture_output=True,text=True).stdout
                if str(rviz.pid)==prop.rsplit('=',1)[-1].strip():window=candidate;break
            if window:break
            time.sleep(.2)
        if not window:raise RuntimeError('Owned RViz window did not appear')
        time.sleep(5)
        image=directory/'rviz_initial_observations.png'
        capture='from PyQt5.QtWidgets import QApplication; import sys; app=QApplication([]); ok=app.primaryScreen().grabWindow(int(sys.argv[1],16)).save(sys.argv[2]); sys.exit(0 if ok else 1)'
        subprocess.run(['python3','-c',capture,window,str(image)],env=env,check=True,timeout=15)
        assert launch.poll() is None and not fused and submaps['uav']
        result['screenshot']=str(image);result['status']='passed'
    except Exception as exc:
        result['status']='failed';result['error']=str(exc);print('FAIL:',repr(exc),flush=True)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=30)
                except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=10)
        for log in logs:log.close()
        (directory/'checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print('CHECKS_DIR='+str(directory),flush=True)
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
