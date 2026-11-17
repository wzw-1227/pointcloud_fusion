#!/usr/bin/env python3
"""Owned both-mode SITL validation, passive sensors only; saves independent truth."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading
import rosgraph
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from stage3_truth_check import TruthCheck
from stage3_verify_results import verify
from run_ros_checks import unused_port


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ugv-x', default='-1.0'); parser.add_argument('--ugv-y', default='0.0')
    parser.add_argument('--uav-x', default='1.0'); parser.add_argument('--uav-y', default='0.0')
    parser.add_argument('--rviz-snapshot', action='store_true')
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    directory = package.parents[1]/'results/stage3'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_both_live_checks')
    directory.mkdir(parents=True)
    env = os.environ.copy(); env['ROS_MASTER_URI'] = 'http://127.0.0.1:'+str(unused_port())
    env['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:'+str(unused_port())
    os.environ['ROS_MASTER_URI'] = env['ROS_MASTER_URI']
    result = {'status': 'running', 'robot': 'both', 'master_uri': env['ROS_MASTER_URI'],
              'spawn': vars(args), 'controls': 'none; stationary actual two LIO inputs'}
    processes, logs = [], []
    truth = None
    def start(command, file):
        log = open(directory/file, 'w'); logs.append(log)
        p = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT); processes.append(p); return p
    try:
        launch = start(['roslaunch', '-p', env['ROS_MASTER_URI'].rsplit(':', 1)[1], 'start_up', 'stage3_sim.launch',
                        'gui:=false', 'rviz:=false', 'manual_control:=false', 'ugv_x:='+args.ugv_x,
                        'ugv_y:='+args.ugv_y, 'uav_x:='+args.uav_x, 'uav_y:='+args.uav_y,
                        'output_root:='+str(directory/'matches'), 'submap_output_root:='+str(directory/'submaps')], 'launch.log')
        deadline = time.monotonic()+40
        while not rosgraph.is_master_online() and time.monotonic()<deadline and launch.poll() is None:
            time.sleep(.1)
        if not rosgraph.is_master_online(): raise RuntimeError('ROS master did not start')
        import rospy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from sensor_msgs.msg import PointCloud2
        from ehpr_bridge.msg import InterRobotMatch
        rospy.init_node('stage3_live_checks', anonymous=True, disable_signals=True)
        truth = TruthCheck(directory)
        matches, statuses = [], []
        subscribers = [rospy.Subscriber('/stage3/match', InterRobotMatch, lambda m: matches.append(m), queue_size=20),
                       rospy.Subscriber('/stage3/status', String, lambda m: statuses.append(yaml.safe_load(m.data)), queue_size=20)]
        deadline = time.monotonic()+180
        last = None
        while time.monotonic()<deadline and launch.poll() is None:
            try: health = json.loads(rospy.wait_for_message('/stage1/health', String, timeout=2).data)
            except rospy.ROSException: continue
            if health['status'] != last:
                print('LIO health:', health['status'], health['errors'], flush=True); last=health['status']
            if health['status']=='ready': break
            if health['status']=='error': raise RuntimeError('LIO health failed: '+str(health['errors']))
        else: raise RuntimeError('Actual two LIO instances did not become ready')
        time.sleep(6)
        for robot in ('ugv', 'uav'):
            rospy.wait_for_service('/stage2/'+robot+'/flush', timeout=10)
            reply = rospy.ServiceProxy('/stage2/'+robot+'/flush', Trigger)()
            if not reply.success: raise RuntimeError(robot+' flush failed: '+reply.message)
            print(robot+' flush: '+reply.message, flush=True)
            time.sleep(2)
        deadline=time.monotonic()+60; next_flush=time.monotonic()+15
        while not matches and time.monotonic()<deadline and launch.poll() is None:
            if time.monotonic()>=next_flush:
                for robot in ('ugv','uav'):
                    reply=rospy.ServiceProxy('/stage2/'+robot+'/flush', Trigger)()
                    print(robot+' retry flush: '+str(reply),flush=True)
                next_flush=time.monotonic()+15
            time.sleep(.2)
        (directory/'matcher_status_history.yaml').write_text(yaml.safe_dump(statuses))
        if not matches:
            raise RuntimeError('No accepted cross-robot match; last status: '+str(statuses[-1] if statuses else 'none'))
        match=matches[-1]; result['match_file']=match.result_path
        result['export_directory']=str(Path(match.result_path).parent.parent)
        result['truth']=truth.evaluate(match)
        if result['truth']['status']!='passed': raise RuntimeError('Independent Gazebo relative pose check failed: '+str(result['truth']))
        result['file_checks']=verify(result['export_directory'])
        for topic in ('match_cloud_ugv','match_cloud_uav','fused_submap'):
            cloud=rospy.wait_for_message('/stage3/'+topic, PointCloud2, timeout=5)
            if cloud.header.frame_id!='odom' or cloud.width*cloud.height<=0 or cloud.header.stamp.to_sec()!=0:
                raise RuntimeError('Invalid matched display cloud '+topic)
            if topic=='fused_submap' and 'rgb' not in [field.name for field in cloud.fields]:
                raise RuntimeError('Fused display cloud has no RGB field')
        if args.rviz_snapshot:
            env['DISABLE_ROS1_EOL_WARNINGS']='1'
            rviz=start(['rosrun','rviz','rviz','-d',str(package.parent/'start_up/rviz/stage3_matches.rviz')], 'rviz.log')
            deadline=time.monotonic()+30; window=None
            while time.monotonic()<deadline and rviz.poll() is None:
                tree=subprocess.run(['xwininfo','-root','-tree'],env=env,capture_output=True,text=True).stdout
                for line in tree.splitlines():
                    if 'stage3_matches.rviz' not in line: continue
                    candidate=line.strip().split()[0]
                    prop=subprocess.run(['xprop','-id',candidate,'_NET_WM_PID'],env=env,capture_output=True,text=True).stdout
                    if str(rviz.pid)==prop.rsplit('=',1)[-1].strip(): window=candidate;break
                if window: break
                time.sleep(.2)
            if not window: raise RuntimeError('Cannot find owned RViz window')
            time.sleep(8)
            screenshot=Path(result['export_directory'])/'rviz_both.png'
            code='from PyQt5.QtWidgets import QApplication; import sys; app=QApplication([]); ok=app.primaryScreen().grabWindow(int(sys.argv[1],16)).save(sys.argv[2]); sys.exit(0 if ok else 1)'
            subprocess.run(['python3','-c',code,window,str(screenshot)],env=env,check=True,timeout=15)
            result['rviz_screenshot']=str(screenshot)
        result['status']='passed'
    except Exception as exc:
        result['status']='failed';result['error']=str(exc);print('FAIL:',exc,flush=True)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=25)
                except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=10)
        for log in logs: log.close()
        if truth:
            for subscriber in truth.subscribers:subscriber.unregister()
            with truth.lock:truth.log.close()
        if result['status']=='passed':result['file_checks']=verify(result['export_directory'])
        (directory/'checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print('CHECKS_DIR='+str(directory),flush=True)
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
