#!/usr/bin/env python3
"""Authorized isolated SITL takeoff/hover/landing; truth is evaluation-only."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time


def prepare_vertical_scan_trial(source, directory, lines):
    """Create an opt-in sensor trial under the run; package defaults are untouched."""
    import shutil
    import yaml
    if lines == 16:
        return None
    trial=directory/'sensor_trial';trial.mkdir()
    models=trial/'models';models.mkdir()
    paths=[]
    for kind in ('lidar','iris'):
        old_name='stage1_'+kind+'_wide';new_name=old_name+str(lines)
        target=models/new_name
        shutil.copytree(source/'models'/old_name,target)
        sdf=target/(old_name+'.sdf');text=sdf.read_text()
        if kind=='lidar':
            assert text.count('<samples>16</samples>')==1
            text=text.replace('<samples>16</samples>','<samples>'+str(lines)+'</samples>')
        else:
            text=text.replace('model://stage1_lidar_wide','model://stage1_lidar_wide'+str(lines))
        sdf.write_text(text);paths.extend([sdf,target/'model.config'])
    config=yaml.safe_load((source/'config/stage1_uav_stable.yaml').read_text())
    config['lio_sam']['N_SCAN']=lines
    config_file=trial/'uav_config.yaml';config_file.write_text(yaml.safe_dump(config,sort_keys=False));paths.append(config_file)
    contract=yaml.safe_load((source/'config/stage1_stable_contract.yaml').read_text())
    contract['robots']['uav']['n_scan']=lines
    contract_file=trial/'contract.yaml';contract_file.write_text(yaml.safe_dump(contract,sort_keys=False));paths.append(contract_file)
    # Copy only launch files that choose the model/contract; retain every other
    # include, single Gazebo, topic, physical parameter and external model.
    for name in ['stage1_px4_wide.launch','stage1_sim.launch','stage2_sim.launch','stage3_sim.launch']:
        text=(source/'launch'/name).read_text()
        text=text.replace('$(find start_up)/models/stage1_iris_wide/stage1_iris_wide.sdf',str(models/('stage1_iris_wide'+str(lines))/'stage1_iris_wide.sdf'))
        text=text.replace("find('start_up') + '/launch/stage1_px4_wide.launch'",repr(str(trial/'stage1_px4_wide.launch')))
        text=text.replace("find('start_up') + '/config/stage1_stable_contract.yaml'",repr(str(contract_file)))
        for include in ['stage1_sim.launch','stage2_sim.launch']:
            text=text.replace('$(find start_up)/launch/'+include,str(trial/include))
        if name=='stage3_sim.launch':
            import html
            text=text.replace('<launch>','<launch>\n  <rosparam param="/diagnostic/source_files">'+html.escape(yaml.safe_dump([str(path) for path in paths],default_flow_style=True).strip())+'</rosparam>',1)
        target=trial/name;target.write_text(text)
    return {'entry':trial/'stage3_sim.launch','config':config_file,'models':models,'source_files':paths}


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));return sock.getsockname()[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensor',choices=['stable','wide'],required=True)
    parser.add_argument('--duration',type=float,default=61)
    parser.add_argument('--uav-z',type=float,default=.3,
                        help='Gazebo spawn height; .055 starts the base collision box on the ground')
    parser.add_argument('--init-in-air',action='store_true',
                        help='diagnostic only: forward UAV scans to LIO after takeoff, then warm up 15 s')
    parser.add_argument('--vertical-lines',type=int,choices=[16,32],default=16,
                        help='wide-only diagnostic scan-density trial; production models remain unchanged')
    parser.add_argument('--hover-warmup',type=float,default=15,
                        help='ROS seconds after takeoff stabilization before the 61 s phase; no LIO reset')
    args=parser.parse_args()
    if args.duration<61:parser.error('at least 61 seconds of hover required')
    if not 0<args.uav_z<=.3:parser.error('diagnostic spawn height must be in (0, 0.3] m')
    if args.vertical_lines!=16 and args.sensor!='wide':parser.error('vertical-line trial requires --sensor wide')
    if args.hover_warmup<0:parser.error('hover warmup must be nonnegative')
    source=Path(__file__).resolve().parents[1]
    directory=source.parents[1]/'results/stability'/(
        datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+args.sensor+'_hover'+
        ('_ground_spawn' if args.uav_z!=.3 else '')+('_air_init' if args.init_in_air else '')+
        ('_lines'+str(args.vertical_lines) if args.vertical_lines!=16 else '')+
        '_warm'+str(args.hover_warmup).replace('.0',''))
    directory.mkdir(exist_ok=False)
    # Both profiles use identical launch defaults, including birth positions.
    env=os.environ.copy();env['ROS_MASTER_URI']='http://127.0.0.1:'+str(free_port())
    env['GAZEBO_MASTER_URI']='http://127.0.0.1:'+str(free_port())
    os.environ['ROS_MASTER_URI']=env['ROS_MASTER_URI']
    command=['roslaunch','-p',env['ROS_MASTER_URI'].rsplit(':',1)[1],'start_up','stage3_sim.launch',
             'sim_profile:=stable','uav_sensor_profile:='+args.sensor,
             'uav_z:='+str(args.uav_z),
             'gui:=false','rviz:=false','manual_control:=false',
             'record_output_root:='+str(directory/'recordings'),
             'output_root:='+str(directory/'matches'),'submap_output_root:='+str(directory/'submaps')]
    trial=prepare_vertical_scan_trial(source,directory,args.vertical_lines)
    if trial:
        command[3:5]=[str(trial['entry'])]
        command.append('uav_config:='+str(trial['config']))
        env['GAZEBO_MODEL_PATH']=str(trial['models'])+':'+env.get('GAZEBO_MODEL_PATH','')
    if args.init_in_air:
        import yaml
        config=yaml.safe_load((source/'config/stage1_uav_stable.yaml').read_text())
        config['lio_sam']['N_SCAN']=args.vertical_lines
        config['lio_sam']['pointCloudTopic']='/stage1/uav/air_init_points'
        config_file=directory/'uav_config.yaml';config_file.write_text(yaml.safe_dump(config,sort_keys=False))
        command=[arg for arg in command if not arg.startswith('uav_config:=')]
        command.append('uav_config:='+str(config_file))
    if subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode==0:
        parser.error('another Gazebo server is active; finish it before this isolated run')
    logfile=open(directory/'launch.log','w')
    process=subprocess.Popen(command,env=env,stdout=logfile,stderr=subprocess.STDOUT)
    result={'status':'running','sensor_profile':args.sensor,'launch_command':command,
            'spawn_height_m':args.uav_z,
            'vertical_lines':args.vertical_lines,
            'hover_warmup_ros_s':args.hover_warmup,
            'init_in_air':args.init_in_air,
            'controls':'explicitly authorized isolated simulation takeoff/hover/landing; no UGV motion',
            'duration_requested_ros_s':args.duration,'truth':'evaluation only; controller uses existing MAVROS position targets'}
    controller=None;rospy=None;recordings=[];poses={r:[] for r in ('ugv','uav')};truth={r:[] for r in poses};debug={r:[] for r in poses}
    matcher=[];matches=[];fused=[];stage='startup';hover_start=None;hover_end=None;wall_start=time.monotonic()
    relay={'enabled':False,'forwarded':0,'first_stamp':None}

    def progress():
        value={'stage':stage,'wall_elapsed_s':time.monotonic()-wall_start,'directory':str(directory)}
        if rospy is not None:
            value['ros_time']=rospy.get_time()
            if hover_start is not None:value['hover_elapsed_ros_s']=max(0,rospy.get_time()-hover_start)
        if controller is not None:
            with controller.lock:
                value.update(armed=controller.state.armed,mode=controller.state.mode)
                if controller.pose:
                    p=controller.pose.pose.position;value['mavros_position']=[p.x,p.y,p.z]
        if recordings:value['recording_directory']=recordings[-1]['directory']
        temporary=directory/'progress.json.tmp';temporary.write_text(json.dumps(value,indent=2)+'\n');temporary.replace(directory/'progress.json')

    def wait_until(predicate,wall_timeout):
        deadline=time.monotonic()+wall_timeout
        while not predicate():
            if process.poll() is not None:raise RuntimeError('isolated launch exited during '+stage)
            if time.monotonic()>deadline:raise RuntimeError(stage+' timeout')
            progress();time.sleep(.25)
        progress()

    try:
        import rosgraph
        wait_until(rosgraph.is_master_online,40)
        import rospy as ros
        rospy=ros
        from std_msgs.msg import String
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from lio_sam.msg import Scan2MapDebug
        from ehpr_bridge.msg import InterRobotMatch
        from stage1_tools.geometry import pose_matrix,initial_alignment
        from stage1_tools.evaluation import matched_poses,trajectory_metrics
        import yaml
        rospy.init_node('authorized_hover_probe',anonymous=True,disable_signals=True)
        wait_until(lambda:rospy.has_param('/stage1/contract_file'),20)
        if rospy.get_param('/stage1/environment')!='sim' or not rospy.get_param('/use_sim_time'):
            raise RuntimeError('this tool requires the owned simulation')
        contract=yaml.safe_load(Path(rospy.get_param('/stage1/contract_file')).read_text())
        subscribers=[]
        if args.init_in_air:
            relay_pub=rospy.Publisher('/stage1/uav/air_init_points',PointCloud2,queue_size=2)
            def forward_scan(message):
                if not relay['enabled']:return
                if relay['first_stamp'] is None:relay['first_stamp']=message.header.stamp.to_sec()
                relay['forwarded']+=1;relay_pub.publish(message)
            subscribers.append(rospy.Subscriber(contract['robots']['uav']['lio_input_cloud'],PointCloud2,forward_scan,queue_size=2))
        for robot in poses:
            config=contract['robots'][robot]
            subscribers.extend([
                rospy.Subscriber(config['odometry'],Odometry,lambda m,r=robot:poses[r].append((m.header.stamp.to_sec(),pose_matrix(m.pose.pose))),queue_size=1000),
                rospy.Subscriber(config['ground_truth'],Odometry,lambda m,r=robot:truth[r].append((m.header.stamp.to_sec(),pose_matrix(m.pose.pose))),queue_size=1000),
                rospy.Subscriber(config['debug'],Scan2MapDebug,lambda m,r=robot:debug[r].append(m),queue_size=1000)])
        subscribers.extend([
            rospy.Subscriber('/stage3/recording',String,lambda m:recordings.append(json.loads(m.data)),queue_size=5),
            rospy.Subscriber('/stage3/status',String,lambda m:matcher.append(yaml.safe_load(m.data)),queue_size=5),
            rospy.Subscriber('/stage3/match',InterRobotMatch,lambda m:matches.append(m),queue_size=100),
            rospy.Subscriber('/fused_submap',PointCloud2,lambda m:fused.append(m.width*m.height),queue_size=2)])
        phase=rospy.Publisher('/stage3/test_phase',String,queue_size=2)
        def mark(name):
            phase.publish(String(data=json.dumps({'phase':name,'stamp':rospy.get_time()})))
        sys.path.insert(0,str(source/'scripts'))
        from stage1_uav_teleop import Teleop
        controller=Teleop(require_lio_ready=not args.init_in_air)
        stage='waiting_for_inputs'
        wait_until(lambda:bool(recordings) and phase.get_num_connections()>=2
                   and (bool(poses['ugv']) if args.init_in_air else all(poses.values()))
                   and (not args.init_in_air or relay_pub.get_num_connections()>0)
                   and all(truth.values()) and controller.pose is not None and controller.state.connected,90)
        with controller.lock:
            initial_z=controller.pose.pose.position.z
        target_z=initial_z+1.5
        stage='takeoff';controller.request_takeoff()
        stable_since=None
        def stabilized():
            nonlocal stable_since
            with controller.lock:
                if not controller.busy and not controller.active:raise RuntimeError('takeoff routine failed; see launch log')
                p=controller.pose.pose.position
                good=(controller.state.armed and controller.state.mode=='OFFBOARD' and controller.takeoff_z is None
                      and abs(p.z-target_z)<.10 and abs(p.x-controller.target[0])<.12 and abs(p.y-controller.target[1])<.12)
            if not good:stable_since=None;return False
            if stable_since is None:stable_since=rospy.get_time()
            return rospy.get_time()-stable_since>=2
        wait_until(stabilized,240)
        def holding_offboard():
            if not controller.state.armed or controller.state.mode!='OFFBOARD':
                raise RuntimeError('OFFBOARD/arming lost during hover warmup')
            return True
        if args.init_in_air:
            stage='lio_air_initialization';relay['enabled']=True;initialization_start=rospy.get_time()
            wait_until(lambda:holding_offboard() and len(poses['uav'])>=20 and rospy.get_time()-initialization_start>=args.hover_warmup,180)
            result['lio_input_start_stamp']=relay['first_stamp']
            result['alignment_scope']='UAV LIO initialized once in air; no ground/takeoff LIO trajectory in this diagnostic run'
        elif args.hover_warmup:
            stage='hover_warmup';warmup_start=rospy.get_time()
            wait_until(lambda:holding_offboard() and rospy.get_time()-warmup_start>=args.hover_warmup,180)
        stage='hover';hover_start=rospy.get_time();mark('hover')
        print(json.dumps({'event':'hover_started','directory':str(directory),'ros_stamp':hover_start,'target_local_z':target_z}),flush=True)
        def hover_done():
            if not controller.state.armed or controller.state.mode!='OFFBOARD':raise RuntimeError('OFFBOARD/arming lost during hover')
            return rospy.get_time()-hover_start>=args.duration
        wait_until(hover_done,max(600,args.duration*12))
        hover_end=rospy.get_time();mark('end');result['hover_interval']=[hover_start,hover_end]
        result['robots']={}
        for robot in poses:
            pairs=matched_poses(poses[robot],truth[robot],.05)
            if not pairs:raise RuntimeError(robot+' missing initial truth association')
            alignment=initial_alignment(pairs[0][1],pairs[0][2])
            window=[(stamp,pose) for stamp,pose in poses[robot] if hover_start<=stamp<=hover_end]
            ds=[m for m in debug[robot] if hover_start<=m.header.stamp.to_sec()<=hover_end]
            result['robots'][robot]={'trajectory':trajectory_metrics(window,truth[robot],.05,alignment),
                'degenerate_fraction':sum(m.is_degenerate or not m.optimization_performed for m in ds)/len(ds) if ds else None,
                'unoptimized_frames':sum(not m.optimization_performed for m in ds),
                'alignment':'first matched pose for the whole available LIO trajectory; no phase realignment'}
        result.update(matches=len(matches),fused_nonempty_messages=sum(p>0 for p in fused),matcher=matcher[-1] if matcher else None)
        stage='landing';controller.request_land()
        wait_until(lambda:not controller.state.armed,180)
        result.update(status='hover_complete',landed_and_disarmed=True)
    except Exception as exc:
        result.update(status='failed',error=str(exc));print(json.dumps({'event':'failed','directory':str(directory),'error':str(exc)}),flush=True)
    finally:
        if controller is not None and controller.state.armed:
            try:
                stage='cleanup_landing';controller.request_land()
                wait_until(lambda:not controller.state.armed,120)
                result['cleanup_landed_and_disarmed']=True
            except Exception as exc:result['cleanup_landing_error']=str(exc)
        stage='shutdown';progress()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=45)
            except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=10)
        logfile.close()
        result['wall_seconds']=time.monotonic()-wall_start
        if args.init_in_air:result['scan_relay']=relay
        if recordings:result['recording_directory']=recordings[-1]['directory']
        (directory/'probe.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'directory':str(directory),'status':result['status'],'error':result.get('error')}),flush=True)
    return 0 if result['status']=='hover_complete' else 1


if __name__=='__main__':raise SystemExit(main())
