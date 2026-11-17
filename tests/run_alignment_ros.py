#!/usr/bin/env python3
"""Replay actual ROS inputs on a private master; check fusion lifecycle contracts."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_ros_checks import unused_port
from stage3_verify_results import verify, verify_map


def wait(predicate, seconds=60):
    end=time.monotonic()+seconds
    while not predicate():
        if time.monotonic()>end:raise RuntimeError('Private ROS contract timed out')
        time.sleep(.05)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bags',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--binary-directory',type=Path,required=True)
    ap.add_argument('--stop-after-matches',type=int,default=0,
                    help='Stop this owned bag player once both profiles have enough confirmed outputs; 0 plays all bags')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False);root=args.output.resolve()
    if args.stop_after_matches<0:raise ValueError('stop-after-matches must be nonnegative')
    def interrupted(signum,frame):raise KeyboardInterrupt('private ROS contract interrupted')
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    port=unused_port();os.environ['ROS_MASTER_URI']=f'http://127.0.0.1:{port}';os.environ['ROS_HOSTNAME']='127.0.0.1'
    os.environ['LD_LIBRARY_PATH']=str(args.binary_directory.resolve())+':'+os.environ.get('LD_LIBRARY_PATH','')
    workspace=Path(__file__).resolve().parents[3];processes=[];logs=[];report=dict(passed=False,scope='private ROS playback, not Gazebo motion acceptance')
    def start(cmd,name):
        log=(root/name).open('w');logs.append(log);p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    try:
        import rosgraph
        start(['roscore','-p',str(port)],'master.log');wait(rosgraph.is_master_online)
        master=rosgraph.Master('/alignment_ros_contract')
        for k,v in {'/use_sim_time':True,'/algorithm_test/enabled':True,'/stage1/environment':'sim'}.items():master.setParam(k,v)
        import rospy
        from sensor_msgs.msg import PointCloud2
        from ehpr_bridge.msg import RobotSubmap,MapAlignment,InterRobotMatch
        from std_srvs.srv import Trigger
        rospy.init_node('alignment_contract',anonymous=True,disable_signals=True)
        profiles=['partial','partial_contour'];maps={p:[] for p in profiles};alignments={p:[] for p in profiles};matches={p:[] for p in profiles};pairs={};subscribers=[]
        for robot in ['ugv','uav']:
            subscribers.append(rospy.Subscriber('/stage2/'+robot+'/submap',RobotSubmap,lambda m,r=robot:pairs.__setitem__(r,m),queue_size=1))
        for profile in profiles:
            base='/contract/'+profile
            subscribers.extend([
                rospy.Subscriber(base+'/fused_map',PointCloud2,lambda m,p=profile:maps[p].append(m),queue_size=10),
                rospy.Subscriber(base+'/alignment',MapAlignment,lambda m,p=profile:alignments[p].append(m),queue_size=10),
                rospy.Subscriber(base+'/match',InterRobotMatch,lambda m,p=profile:matches[p].append(m),queue_size=100)])
            remaps={'/stage3/fused_map':base+'/fused_map','/stage3/map_alignment':base+'/alignment','/stage3/match':base+'/match',
                '/stage3/status':base+'/status','/fused_submap':base+'/fused_submap','/stage3/fused_submap':base+'/fused_compat',
                '/stage3/match_cloud_ugv':base+'/ugv','/stage3/match_cloud_uav':base+'/uav','/stage3/retry_history':base+'/retry'}
            cmd=[str(args.binary_directory.resolve()/'stage3_matching_node'),'__name:=contract_'+profile,'_mode:=ideal_frontend','_require_lio_quality:=false',
                '_config:='+str(workspace/'src/start_up/config'/('stage3_gazebo_algorithm_matching_'+profile+'.yaml')),'_output_root:='+str(root/profile)]
            start(cmd+[k+':='+v for k,v in remaps.items()],profile+'.log')
        wait(lambda:all(any(node=='/contract_'+p for _,nodes in master.getSystemState()[1] for node in nodes) for p in profiles))
        bags=sorted(args.bags.glob('sensors_*.bag'),key=lambda p:int(p.stem.split('_')[-1]))
        if not bags:raise RuntimeError('No recorded ROS bags')
        topics=['/clock']+['/algorithm_test/'+r+'/'+name for r in ['ugv','uav'] for name in ['cloud_registered','odometry']]+['/stage2/ugv/submap','/stage2/uav/submap']
        player=start(['rosbag','play','--queue=1000','--rate=1',*[str(p.resolve()) for p in bags],'--topics',*topics],'playback.log')
        deadline=time.monotonic()+1800
        while player.poll() is None:
            if time.monotonic()>deadline:raise RuntimeError('ROS playback deadline exceeded')
            if args.stop_after_matches and all(len(matches[p])>=args.stop_after_matches for p in profiles):
                os.killpg(player.pid,signal.SIGINT);player.wait(timeout=30)
                report['playback_stop_reason']='confirmed outputs available for lifecycle contract'
                report['playback_matches_required']=args.stop_after_matches
                break
            time.sleep(.2)
        wait(lambda:all(matches[p] and maps[p] and maps[p][-1].width>0 and alignments[p][-1].valid for p in profiles))
        report['profiles']={}
        for profile in profiles:
            base='/contract/'+profile
            late=[];subscribers.append(rospy.Subscriber(base+'/fused_map',PointCloud2,lambda m:late.append(m),queue_size=1))
            wait(lambda:bool(late) and late[-1].width>0)
            retry=rospy.ServiceProxy(base+'/retry',Trigger);before=time.monotonic();response=retry();elapsed=time.monotonic()-before
            report['profiles'][profile]=dict(matches=len(matches[profile]),late_subscriber_receives_fusion=True,retry_service_wall_s=elapsed)
            assert response.success and elapsed<.2, ('historical retry service',profile,response.success,elapsed)
        # A new Stage 2 epoch must clear every fused layer immediately and retain the peer map.
        previous_counts={p:len(matches[p]) for p in profiles}
        msg=pairs['ugv'];msg.epoch+=1;msg.submap_id=0
        publisher=rospy.Publisher('/stage2/ugv/submap',RobotSubmap,queue_size=1);wait(lambda:publisher.get_num_connections()>=3);publisher.publish(msg)
        wait(lambda:all(alignments[p] and not alignments[p][-1].valid and maps[p][-1].width==0 for p in profiles))
        cleared={p:{} for p in profiles}
        for profile in profiles:
            for suffix in ['fused_submap','fused_compat','ugv','uav']:
                subscribers.append(rospy.Subscriber('/contract/'+profile+'/'+suffix,PointCloud2,lambda m,p=profile,k=suffix:cleared[p].__setitem__(k,m.width),queue_size=1))
        wait(lambda:all(len(cleared[p])>=4 for p in profiles))
        assert all(all(size==0 for size in cleared[p].values()) for p in profiles), ('latched fusion was not cleared',cleared)
        for profile in profiles:report['profiles'][profile]['epoch_clears_all_latched_fusion']=True
        report['passed']=True
    except (Exception,KeyboardInterrupt) as error:report['error']=repr(error)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=30)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
        for f in logs:f.close()
        for profile in ['partial','partial_contour']:
            runs=list((root/profile).glob('*/summary.yaml'))
            if not runs:continue
            summary=yaml.load(runs[0].read_text(),Loader=yaml.CSafeLoader);report.setdefault('profiles',{}).setdefault(profile,{})['summary']=dict(accepted_matches=summary.get('accepted_matches'),alignment_valid=summary.get('alignment_valid'))
            try:
                report['profiles'][profile]['file_checks']=verify(runs[0].parent)
                report['profiles'][profile]['map_checks']=verify_map(runs[0].parent)
            except Exception as error:report['passed']=False;report['profiles'][profile]['file_error']=repr(error)
        (root/'checks.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
