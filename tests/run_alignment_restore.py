#!/usr/bin/env python3
"""Prove a warm cache cannot restore trust or bypass a subsequent clock reset."""
import argparse,json,os,signal,subprocess,time,sys
from pathlib import Path
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_ros_checks import unused_port
from stage2_verify_exports import read_pcd
from run_alignment_ros import wait
from scipy.spatial.transform import Rotation
import numpy as np


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--inputs',type=Path,required=True)
    ap.add_argument('--binary-directory',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False);root=a.output.resolve();workspace=Path(__file__).resolve().parents[3]
    os.environ['ROS_MASTER_URI']=f'http://127.0.0.1:{unused_port()}';os.environ['ROS_HOSTNAME']='127.0.0.1'
    os.environ['LD_LIBRARY_PATH']=str(a.binary_directory.resolve())+':'+os.environ.get('LD_LIBRARY_PATH','')
    port=os.environ['ROS_MASTER_URI'].rsplit(':',1)[1];processes=[];logs=[];report=dict(passed=False)
    def start(cmd,name):
        f=(root/name).open('w');logs.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    try:
        import rosgraph
        start(['roscore','-p',port],'master.log');wait(rosgraph.is_master_online)
        master=rosgraph.Master('/restore_contract')
        for k,v in {'/use_sim_time':True,'/algorithm_test/enabled':True,'/stage1/environment':'sim'}.items():master.setParam(k,v)
        import rospy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String,Header
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from ehpr_bridge.msg import RobotSubmap,MapAlignment
        from std_srvs.srv import Trigger
        rospy.init_node('restore_contract',anonymous=True,disable_signals=True)
        clock=rospy.Publisher('/clock',Clock,queue_size=1,latch=True);clock.publish(Clock(rospy.Time.from_sec(1000)))
        alignments={};statuses={};maps={};subs=[]
        for profile in ['partial','partial_contour']:
            base='/restore/'+profile;alignments[profile]=[];statuses[profile]=[];maps[profile]=[]
            subs.extend([rospy.Subscriber(base+'/alignment',MapAlignment,lambda m,p=profile:alignments[p].append(m.valid),queue_size=10),
                rospy.Subscriber(base+'/map',PointCloud2,lambda m,p=profile:maps[p].append(m.width),queue_size=10),
                rospy.Subscriber(base+'/status',String,lambda m,p=profile:statuses[p].append(yaml.load(m.data,Loader=yaml.CSafeLoader)),queue_size=10)])
            remaps={'/stage3/map_alignment':base+'/alignment','/stage3/fused_map':base+'/map','/stage3/status':base+'/status','/stage3/retry_history':base+'/retry',
                '/stage3/match':base+'/match','/fused_submap':base+'/pair','/stage3/fused_submap':base+'/pair_compat',
                '/stage3/match_cloud_ugv':base+'/ugv','/stage3/match_cloud_uav':base+'/uav'}
            start([str(a.binary_directory.resolve()/'stage3_matching_node'),'__name:=restore_'+profile,'_mode:=ideal_frontend','_require_lio_quality:=false',
                '_config:='+str(workspace/'src/start_up/config'/('stage3_gazebo_algorithm_matching_'+profile+'.yaml')),
                '_output_root:='+str(root/profile),'_warm_start_inputs:='+str(a.inputs.resolve())]+[k+':='+v for k,v in remaps.items()],profile+'.log')
        wait(lambda:all(alignments[p] and maps[p] for p in alignments),180)
        assert all(not any(alignments[p]) and not any(maps[p]) for p in alignments)
        for p in alignments:
            retry=rospy.ServiceProxy('/restore/'+p+'/retry',Trigger);assert retry().success
        time.sleep(1);assert all(not any(alignments[p]) for p in alignments)
        report['warm_cache_and_retry_do_not_restore_trust']=True
        # Ensure the worker has observed the initial clock before rolling it back.
        time.sleep(.5);clock.publish(Clock(rospy.Time.from_sec(10)));time.sleep(.5)
        latest={}
        for path in a.inputs.glob('*.yaml'):
            m=yaml.load(path.read_text(),Loader=yaml.CSafeLoader);role=m['robot_id'];sequence=m.get('ingestion_sequence',0)
            if role not in latest or sequence>latest[role][0]:latest[role]=(sequence,path,m)
        publishers=[]
        for role,label in enumerate(['ugv','uav']):
            _,path,meta=latest[role];pose=np.asarray(meta['T_reference_key']);msg=RobotSubmap()
            msg.robot_id=role;msg.session_id=str(meta['session_id']);msg.epoch=meta['epoch'];msg.submap_id=meta['submap_id']+1;msg.partial=True
            msg.header=Header(stamp=rospy.Time.from_sec(10),frame_id=meta['reference_frame']);msg.gravity_alignment.w=1
            msg.cloud_local=point_cloud2.create_cloud_xyz32(Header(stamp=msg.header.stamp,frame_id=meta['local_frame']+'_clock_probe'),read_pcd(path.with_suffix('.pcd')))
            p=msg.pose_in_lio_map.position;p.x,p.y,p.z=pose[:3,3];q=msg.pose_in_lio_map.orientation;q.x,q.y,q.z,q.w=Rotation.from_matrix(pose[:3,:3]).as_quat()
            pub=rospy.Publisher('/stage2/'+label+'/submap',RobotSubmap,queue_size=1);publishers.append(pub);wait(lambda:pub.get_num_connections()>=2);pub.publish(msg)
        reason='clock or odometry reset requires a new Stage 2 epoch'
        wait(lambda:all(statuses[p] and statuses[p][-1].get('last_reason')==reason for p in statuses))
        for p in statuses:
            d=statuses[p][-1].get('last_diagnostics',{});assert d.get('map_ugv_voxels')==0 and d.get('map_uav_voxels')==0
            assert not alignments[p][-1] and maps[p][-1]==0
        report['clock_rollback_clears_maps_and_blocks_cached_epochs']=True;report['passed']=True
    except Exception as error:report['error']=repr(error)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=30)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
        for f in logs:f.close()
        (root/'checks.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
