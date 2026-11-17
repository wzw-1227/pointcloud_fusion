#!/usr/bin/env python3
"""Record a manual test, freeze its configuration, then evaluate its bag."""
import argparse
from datetime import datetime,timezone,timedelta
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import rospy
import rospkg
import rosnode
import yaml
from stage1_tools.contracts import load_contract,selected_robots
from stage1_tools.evaluation import Observations,evaluate_bag,make_report,write_report


def main():
    parser=argparse.ArgumentParser(description='Record Stage 1 manual driving; duration uses ROS simulation time')
    parser.add_argument('--robot',choices=['ugv','uav','both'])
    parser.add_argument('--scenario',choices=['static','motion','height','coexist','smoke'],default='static')
    parser.add_argument('--duration',type=float,default=80,help='ROS seconds; 0 records until Ctrl-C')
    parser.add_argument('--warmup',type=float,default=15)
    parser.add_argument('--output-root');parser.add_argument('--wall-timeout',type=float)
    args=parser.parse_args()
    if args.duration<0 or args.warmup<0:parser.error('duration and warmup must be nonnegative')
    rospy.init_node('stage1_record',anonymous=True,disable_signals=True)
    selection=args.robot or rospy.get_param('/stage1/robot')
    if selection != rospy.get_param('/stage1/robot'):
        parser.error('--robot must match the running stage1 launch')
    environment=rospy.get_param('/stage1/environment')
    contract_path=rospy.get_param('/stage1/contract_file')
    contract=load_contract(contract_path,selection,environment)
    package=Path(rospkg.RosPack().get_path('start_up'))
    root=Path(args.output_root) if args.output_root else package.parents[1]/'results'/'stage1'
    label=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')+'_'+selection+'_'+args.scenario
    directory=root/label;directory.mkdir(parents=True,exist_ok=False)
    frozen=directory/'contract.yaml';shutil.copyfile(contract_path,frozen)
    (directory/'parameters.yaml').write_text(yaml.safe_dump(rospy.get_param('/'),sort_keys=False))
    topics=['/tf','/tf_static','/clock','/rosout_agg','/stage1/health']
    for name in selected_robots(selection):
        r=contract['robots'][name]
        topics.extend(r[k] for k in ('raw_cloud','imu','odometry','registered','registered_raw','deskewed','path','debug'))
        if r.get('lio_input_cloud'):topics.append(r['lio_input_cloud'])
        prefix='' if name=='ugv' else '/uav'
        topics.extend([prefix+'/lio_sam/mapping/map_global',prefix+'/lio_sam/mapping/odometry_incremental',prefix+'/odometry/imu_incremental'])
        if environment=='sim':topics.extend([r['ground_truth'],'/stage1/'+name+'/lidar_input_stats'])
    if environment=='sim':topics+=['/gazebo/model_states','/gazebo/link_states','/cmd_vel','/stage1/ugv/cmd_vel_raw','/iris_0/mavros/local_position/pose','/iris_0/mavros/state','/iris_0/mavros/setpoint_position/local']
    bag=directory/'data.bag'
    metadata={'robot':selection,'scenario':args.scenario,'environment':environment,'warmup':args.warmup,
              'requested_duration':args.duration,'topics':sorted(set(topics)),'nodes_before':rosnode.get_node_names(),
              'timezone':'Asia/Shanghai','ground_truth_timestamp':'Gazebo ModelStates receipt in ROS time, capped at 50 Hz; no message header',
              'map_review':'pending','timed_out':False}
    logfile=open(directory/'recorder.log','w')
    process=subprocess.Popen(['rosbag','record','--buffsize=512','-O',str(bag)]+sorted(set(topics)),stdout=logfile,stderr=subprocess.STDOUT)
    wall_start=time.monotonic();start=None
    timeout=args.wall_timeout or (max(300,args.duration*5+120) if args.duration else float('inf'))
    print('Recording -> '+str(directory),flush=True)
    recording_error=None
    try:
        while not rospy.is_shutdown():
            if process.poll() is not None:raise RuntimeError('rosbag recorder exited early; see recorder.log')
            now=rospy.get_time()
            if start is None and now>0:start=now
            if args.duration and start is not None and now-start>=args.duration:break
            if time.monotonic()-wall_start>timeout:
                metadata['timed_out']=True;break
            time.sleep(.1)
    except KeyboardInterrupt:
        print('Stopping recording and generating report...',flush=True)
    except Exception as exc:
        recording_error=str(exc)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.terminate();process.wait(timeout=10)
        logfile.close()
        metadata['recorder_exit_code']=process.returncode
        try:metadata['nodes_after']=rosnode.get_node_names()
        except Exception:
            metadata['nodes_after']=[]
            metadata['master_lost']=True
        metadata['observed_ros_duration']=rospy.get_time()-start if start is not None else 0
        (directory/'session.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+'\n')
        logs=directory/'ros_logs';logs.mkdir()
        log_root=Path(os.environ.get('ROS_LOG_DIR',str(Path.home()/'.ros/log')))
        latest=log_root/'latest'
        if latest.exists():
            for file in latest.iterdir():
                if file.is_file() and (file.name=='rosout.log' or 'lio_sam' in file.name or file.name.startswith('roslaunch-')):
                    shutil.copy2(file,logs/file.name)
    # The rosbag Python wrapper may exit before its C++ child has renamed
    # data.bag.active. Wait for finalization without reading an active bag.
    finalize_deadline=time.monotonic()+30
    while not bag.is_file() and time.monotonic()<finalize_deadline:
        time.sleep(.1)
    if bag.is_file():
        try:
            report=evaluate_bag(bag,contract,selection,args.scenario,args.warmup,environment)
        except Exception as exc:
            recording_error='bag evaluation failed: '+str(exc)
            report=make_report(Observations(contract,selected_robots(selection)),selection,args.scenario,environment)
    else:
        recording_error='No finalized data.bag; inspect recorder.log and data.bag.active'
        report=make_report(Observations(contract,selected_robots(selection)),selection,args.scenario,environment)
    if recording_error or metadata['recorder_exit_code'] != 0:
        report['errors'].append(recording_error or 'rosbag exited with code '+str(metadata['recorder_exit_code']))
        report['numeric_pass']=False;report['status']='failed'
    if metadata['timed_out']:
        report['errors'].append('recorder wall timeout: simulation clock stalled or recording incomplete')
        report['numeric_pass']=False;report['status']='failed'
    expected=[]
    for name in selected_robots(selection):
        prefix='' if name=='ugv' else '/uav'
        expected.extend(prefix+'/lio_sam_'+suffix for suffix in ('imuPreintegration','imageProjection','featureExtraction','mapOptmization'))
    missing=[n for n in expected if n not in metadata['nodes_after']]
    if missing:
        report['errors'].append('LIO nodes missing at end of recording: '+', '.join(missing))
        report['numeric_pass']=False;report['status']='failed'
    write_report(report,directory)
    (directory/'tf_snapshot.json').write_text(json.dumps(report['tf'],indent=2)+'\n')
    print(report['status']+' -> '+str(directory/'report.md'))
    for error in report['errors']:print('FAIL: '+error)
    return 0 if report['numeric_pass'] else 1


if __name__=='__main__':raise SystemExit(main())
