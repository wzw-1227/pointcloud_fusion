#!/usr/bin/env python3
"""Validate both guarded profiles in an owned simulation with unchanged spawns.

Controllers only attach to the isolated master started here. Gazebo truth is used
for test positioning and post-registration scoring, never by the matcher.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

WORKSPACE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE/'src/start_up/scripts'))
sys.path.insert(0, str(WORKSPACE/'src/ehpr_bridge/scripts'))
from run_ros_checks import unused_port
from stage3_verify_results import verify, verify_map


def wait(predicate, timeout=120):
    end = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() > end:
            raise RuntimeError('Owned Gazebo acceptance timed out')
        time.sleep(.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--binary-directory', type=Path, required=True)
    parser.add_argument('--motion', choices=['ugv', 'uav'], required=True)
    parser.add_argument('--uav-path', choices=['witnessed','corridor','room'], default='witnessed')
    args = parser.parse_args()
    def interrupted(signum, frame):
        raise KeyboardInterrupt('owned acceptance interrupted')
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    if subprocess.run(['pgrep', '-x', 'gzserver'], capture_output=True).returncode == 0:
        raise RuntimeError('Existing Gazebo detected; no existing simulator will be controlled')
    args.output.mkdir(parents=True, exist_ok=False)
    root = args.output.resolve()
    ros_port, gazebo_port = unused_port(), unused_port()
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{ros_port}',
                      GAZEBO_MASTER_URI=f'http://127.0.0.1:{gazebo_port}', ROS_HOSTNAME='127.0.0.1')
    os.environ['LD_LIBRARY_PATH'] = str(args.binary_directory.resolve())+':'+os.environ.get('LD_LIBRARY_PATH', '')
    # Reuse the real frontend, world, Stage 2 and sensors; omit only the deployed matcher.
    launch_xml = ET.parse(WORKSPACE/'src/start_up/launch/stage3_gazebo_algorithm.launch')
    for node in list(launch_xml.getroot()):
        if node.tag == 'include' and 'stage3_matching.launch' in node.attrib.get('file', ''):
            launch_xml.getroot().remove(node)
    launch_path = root/'frontend.launch'
    launch_xml.write(launch_path, encoding='unicode')
    command = ['roslaunch', '-p', str(ros_port), str(launch_path),
               'world:='+str(WORKSPACE/'src/start_up/worlds/stage3_rendezvous.world'),
               'ugv_x:=4.0', 'ugv_y:=7.0', 'uav_x:=-7.0', 'uav_y:=5.0', 'uav_z:=0.3',
               'max_range:=12.0', 'gui:=false', 'rviz:=false', 'record:=false', 'manual_control:=false',
               'output_root:='+str(root/'frontend_run')]
    processes, logs = [], []
    report = dict(passed=False, motion=args.motion, command=command,
                  scope='owned Gazebo, real ray scans, ideal frontend; not real LIO acceptance',
                  truth_role='test motion and independent scoring only',
                  ros_master=os.environ['ROS_MASTER_URI'], gazebo_master=os.environ['GAZEBO_MASTER_URI'])
    (root/'process.json').write_text(json.dumps(report, indent=2)+'\n')

    def start(cmd, name):
        log=(root/name).open('w');logs.append(log)
        p=subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p);return p

    controller = None
    velocity = None
    try:
        launch = start(command, 'launch.log')
        import rosgraph
        wait(lambda: launch.poll() is not None or rosgraph.is_master_online())
        if launch.poll() is not None:
            raise RuntimeError('Owned launch failed')
        master=rosgraph.Master('/acceptance_startup')
        wait(lambda:master.hasParam('/algorithm_test/enabled') and master.hasParam('/stage1/environment'))
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from sensor_msgs.msg import PointCloud2
        from ehpr_bridge.msg import InterRobotMatch, MapAlignment
        from stage1_uav_teleop import Teleop
        rospy.init_node('alignment_acceptance', anonymous=True, disable_signals=True)
        poses, truth, matches, states, clouds, statuses = {}, {}, {}, {}, {}, {}
        subscribers = []
        for robot in ['ugv', 'uav']:
            subscribers.append(rospy.Subscriber('/algorithm_test/'+robot+'/odometry', Odometry,
                lambda m, r=robot: poses.__setitem__(r,m), queue_size=1))
            subscribers.append(rospy.Subscriber('/stage1/'+robot+'/ground_truth/lidar_odom', Odometry,
                lambda m, r=robot: truth.__setitem__(r,m), queue_size=1))
        for profile in ['partial', 'partial_contour']:
            prefix='/acceptance/'+profile
            matches[profile], states[profile], clouds[profile], statuses[profile] = [], [], [], []
            subscribers.extend([
                rospy.Subscriber(prefix+'/match', InterRobotMatch, lambda m,p=profile:matches[p].append(m),queue_size=100),
                rospy.Subscriber(prefix+'/map_alignment',MapAlignment,lambda m,p=profile:states[p].append((rospy.get_time(),m.valid)),queue_size=100),
                rospy.Subscriber(prefix+'/fused_map',PointCloud2,lambda m,p=profile:clouds[p].append((rospy.get_time(),m.width)),queue_size=100),
                rospy.Subscriber(prefix+'/status',String,lambda m,p=profile:statuses[p].append(yaml.load(m.data,Loader=yaml.CSafeLoader)),queue_size=100)])
            remaps={ '/stage3/match':prefix+'/match', '/stage3/status':prefix+'/status',
                '/stage3/map_alignment':prefix+'/map_alignment','/stage3/fused_map':prefix+'/fused_map',
                '/stage3/match_cloud_ugv':prefix+'/match_cloud_ugv','/stage3/match_cloud_uav':prefix+'/match_cloud_uav',
                '/fused_submap':prefix+'/fused_submap','/stage3/fused_submap':prefix+'/fused_submap_compat',
                '/stage3/retry_history':prefix+'/retry_history'}
            cmd=[str(args.binary_directory.resolve()/'stage3_matching_node'),'__name:=acceptance_'+profile,
                 '_mode:=ideal_frontend','_require_lio_quality:=false',
                 '_config:='+str(WORKSPACE/'src/start_up/config'/('stage3_gazebo_algorithm_matching_'+profile+'.yaml')),
                 '_output_root:='+str(root/profile)]
            cmd.extend(k+':='+v for k,v in remaps.items())
            start(cmd,profile+'.log')
        wait(lambda: len(poses)==2 and len(truth)==2 and bool(list((root/'frontend_run/frontend').glob('*/frontend.json'))))
        frontend_path=next((root/'frontend_run/frontend').glob('*/frontend.json'))
        wait(lambda: len(json.loads(frontend_path.read_text()).get('robots',{}))==2)
        world={r:np.asarray(json.loads(frontend_path.read_text())['robots'][r]['T_world_reference']) for r in ['ugv','uav']}
        def healthy():
            if any(p.poll() is not None for p in processes):raise RuntimeError('An owned test process exited')
        def hold(seconds):
            stamp=rospy.get_time();deadline=time.monotonic()+max(180,seconds*15)
            while rospy.get_time()-stamp<seconds:
                healthy()
                if time.monotonic()>deadline:raise RuntimeError('Simulation clock stalled')
                time.sleep(.1)
        hold(18)
        report['stationary_initial_match_counts']={p:len(m) for p,m in matches.items()}
        if any(matches.values()):raise RuntimeError('Stationary initialization published unconfirmed fusion')
        velocity=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        if args.motion=='uav':
            controller=Teleop(require_lio_ready=False)
            wait(lambda:controller.pose is not None and controller.state.connected)
            with controller.lock:controller.hold_current();controller.active=True
            hold(3)
            if not controller.mode(custom_mode='OFFBOARD').mode_sent:raise RuntimeError('OFFBOARD rejected')
            wait(lambda:controller.state.mode=='OFFBOARD',20)
            if not controller.arm(value=True).success:raise RuntimeError('Arming rejected')
            wait(lambda:controller.state.armed,20)
            # The corridor retains visible floor and separate wall directions at low altitude.
            waypoints=([(-7,5,3.3),(1.48,9.06,3.3),(1.48,9.06,.89),(1.48,8.1,.89),(.25,8.8,.89),(1.48,9.06,.89)]
                       if args.uav_path=='corridor' else
                       [(-7,5,3.3),(4,8,3.3),(4,8,.9),(4,7.3,.9),(7.5,7.3,.9),(4,7.3,.9)])
            if args.uav_path=='witnessed':
                # Near the two actual independent supports from the recorded positive run.
                waypoints=[(-7,5,3.3),(1.37,9.5,3.3),(1.37,9.5,.875),(1.35,8.9,.855),(1.35,9.06,.65)]
            report['waypoints_world']=waypoints
            for waypoint in waypoints:
                goal=(np.linalg.inv(world['uav'])@np.r_[waypoint,1])[:3]
                before=rospy.get_time();previous=before;deadline=time.monotonic()+900
                while True:
                    healthy();now=rospy.get_time();dt=max(0,min(.1,now-previous));previous=now
                    p=poses['uav'].pose.pose.position;actual=np.array([p.x,p.y,p.z])
                    if np.linalg.norm(actual-goal)<.15:break
                    if time.monotonic()>deadline:raise RuntimeError('UAV waypoint timeout '+str(waypoint))
                    with controller.lock:
                        local=controller.pose.pose.position;desired=np.array([local.x,local.y,local.z])+goal-actual
                        delta=desired-np.array(controller.target[:3]);norm=np.linalg.norm(delta)
                        speed=.2 if args.uav_path=='witnessed' and waypoint[2]<2 else .5
                        if norm:delta*=min(1,speed*dt/norm)
                        controller.target[:3]=(np.array(controller.target[:3])+delta).tolist()
                    time.sleep(.05)
                print('WAYPOINT',args.motion,waypoint,'matches',{p:len(v) for p,v in matches.items()},flush=True)
                hold(10)
        else:
            waypoints=[(4,8.5),(1,8.5),(1,2),(-2.5,2),(-2.5,7.5),(-8,7.5),(-8,5.7),(-8,7.5)]
            report['waypoints_world']=waypoints
            for waypoint in waypoints:
                deadline=time.monotonic()+900
                while True:
                    healthy();p=truth['ugv'].pose.pose.position;q=truth['ugv'].pose.pose.orientation
                    error=np.array(waypoint)-[p.x,p.y];distance=np.linalg.norm(error)
                    if distance<.18:break
                    if time.monotonic()>deadline:raise RuntimeError('UGV waypoint timeout '+str(waypoint))
                    yaw=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]
                    angle=math.atan2(math.sin(math.atan2(error[1],error[0])-yaw),math.cos(math.atan2(error[1],error[0])-yaw))
                    command=Twist();command.angular.z=float(np.clip(1.5*angle,-.7,.7))
                    if abs(angle)<.4:command.linear.x=min(.45,.6*distance)
                    velocity.publish(command);time.sleep(.05)
                velocity.publish(Twist())
                print('WAYPOINT',args.motion,waypoint,'matches',{p:len(v) for p,v in matches.items()},flush=True)
                hold(10)
        hold(20)
        report['profiles']={}
        for profile in matches:
            scores=[]
            for message in matches[profile]:
                m=yaml.safe_load(Path(message.result_path).read_text());a=world['ugv']@np.asarray(m['reference']['T_reference_key']);b=world['uav']@np.asarray(m['query']['T_reference_key'])
                expected=np.linalg.inv(b)@a;actual=np.asarray(m['T_query_reference_refined'])
                scores.append(dict(position_error_m=float(np.linalg.norm(actual[:3,3]-expected[:3,3])),
                    rotation_error_deg=float(np.degrees(Rotation.from_matrix(actual[:3,:3].T@expected[:3,:3]).magnitude())),path=message.result_path))
            runs=list((root/profile).glob('*/events.yaml'));durations=[]
            for path in runs:
                events=list(yaml.load_all(path.read_text(),Loader=yaml.CSafeLoader));durations.extend(e.get('diagnostics',{}).get('matching_processing_wall_s',0) for e in events if e.get('diagnostics',{}).get('matching_processing_wall_s',0)>0)
            drops=max((s.get('worker_queue_drops',0) for s in statuses[profile]),default=0)
            report['profiles'][profile]=dict(matches=scores,queue_drops=drops,alignment_states=states[profile],map_sizes=clouds[profile],
                processing_wall_s_p95=float(np.percentile(durations,95)) if durations else 0)
        report['passed']=all(p['matches'] and p['queue_drops']==0 and all(m['position_error_m']<=.2 and m['rotation_error_deg']<=3 for m in p['matches']) for p in report['profiles'].values())
    except (Exception, KeyboardInterrupt) as error:
        report['error']=repr(error)
    finally:
        if velocity is not None:
            from geometry_msgs.msg import Twist
            velocity.publish(Twist())
        if controller is not None and controller.state.armed:
            controller.request_land()
            try:wait(lambda:not controller.state.armed,90)
            except Exception:pass
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
        for log in logs:log.close()
        # File verification is done after shutdown so checkpoints and journals are final.
        if report.get('profiles'):
            for profile in report['profiles']:
                try:
                    run=next((root/profile).iterdir());report['profiles'][profile]['file_checks']=verify(run)
                    report['profiles'][profile]['map_checks']=verify_map(run,run/'recent_inputs')
                    # Include results saved while landing and draining the worker,
                    # beyond the live subscriber's observation interval.
                    saved=[]
                    for path in sorted(run.glob('match_*/match.yaml')):
                        m=yaml.load(path.read_text(),Loader=yaml.CSafeLoader)
                        expected=np.linalg.inv(world['uav']@np.asarray(m['query']['T_reference_key']))@(world['ugv']@np.asarray(m['reference']['T_reference_key']))
                        actual=np.asarray(m['T_query_reference_refined'])
                        saved.append(dict(path=str(path),position_error_m=float(np.linalg.norm(actual[:3,3]-expected[:3,3])),
                            rotation_error_deg=float(np.degrees(Rotation.from_matrix(actual[:3,:3].T@expected[:3,:3]).magnitude()))))
                    report['profiles'][profile]['all_saved_matches']=saved
                    report['passed'] &= bool(saved) and all(m['position_error_m']<=.2 and m['rotation_error_deg']<=3 for m in saved)
                except Exception as error:
                    report['passed']=False;report['profiles'][profile]['verification_error']=repr(error)
        (root/'checks.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
