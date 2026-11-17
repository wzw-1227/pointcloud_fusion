#!/usr/bin/env python3
"""Own an isolated ROS master: warm restore, historical retry and map lifecycle.

This exercises ROS contracts without Gazebo and is not flight acceptance.
"""
import argparse,json,os,signal,subprocess,time,sys
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stage2_verify_exports import read_pcd
from run_ros_checks import unused_port


def wait(predicate,seconds=40):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        if predicate():return
        time.sleep(.05)
    raise RuntimeError('ROS contour contract test timed out')


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--warm-inputs',type=Path,required=True);ap.add_argument('--config',type=Path,required=True)
    ap.add_argument('--binary',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    port=unused_port();os.environ['ROS_MASTER_URI']=f'http://127.0.0.1:{port}';os.environ['ROS_HOSTNAME']='127.0.0.1'
    config=yaml.safe_load(a.config.read_text());config['use_recent_matching_cloud']=False
    path=a.output/'contract_config.yaml';path.write_text(yaml.safe_dump(config))
    processes=[];logs=[];report=dict(passed=False,scope='isolated ROS interface test, not Gazebo or flight acceptance')
    def start(cmd,name):
        log=(a.output/name).open('w');logs.append(log);p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    try:
        import rosgraph
        start(['roscore','-p',str(port)],'master.log');wait(rosgraph.is_master_online)
        master=rosgraph.Master('/contour_contract');master.setParam('/use_sim_time',True)
        master.setParam('/algorithm_test/enabled',True);master.setParam('/stage1/environment','sim')
        import rospy
        from rosgraph_msgs.msg import Clock
        from ehpr_bridge.msg import RobotSubmap,MapAlignment,InterRobotMatch
        from std_srvs.srv import Trigger
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header,String
        rospy.init_node('contour_contract',anonymous=True,disable_signals=True)
        clock=rospy.Publisher('/clock',Clock,queue_size=1,latch=True)
        current=[100.];timer=rospy.Timer(rospy.Duration(.1),lambda _:None) if False else None
        def tick():clock.publish(Clock(rospy.Time.from_sec(current[0])))
        tick()
        alignments=[];maps=[];matches=[];status=[]
        subscribers=[rospy.Subscriber('/stage3/map_alignment',MapAlignment,lambda m:alignments.append(m),queue_size=10),
          rospy.Subscriber('/stage3/fused_map',PointCloud2,lambda m:maps.append((time.monotonic(),m)),queue_size=10),
          rospy.Subscriber('/stage3/match',InterRobotMatch,lambda m:matches.append(m),queue_size=10),
          rospy.Subscriber('/stage3/status',String,lambda m:status.append(yaml.load(m.data,Loader=yaml.CSafeLoader)),queue_size=10)]
        node=start([str(a.binary.resolve()),'_config:='+str(path.resolve()),'_output_root:='+str((a.output/'run').resolve()),
          '_mode:=ideal_frontend','_require_lio_quality:=false','_warm_start_inputs:='+str(a.warm_inputs.resolve())], 'matcher.log')
        initial_pubs=[]
        for robot,label in enumerate(['ugv','uav']):
            source=next(p for p in a.warm_inputs.glob(label+'*.yaml') if yaml.safe_load(p.read_text())['submap_id']==2)
            meta=yaml.safe_load(source.read_text());points=read_pcd(source.with_suffix('.pcd'));matrix=np.array(meta['T_reference_key'])
            msg=RobotSubmap();msg.robot_id=robot;msg.session_id=meta['session_id'];msg.epoch=meta['epoch'];msg.submap_id=3;msg.partial=True
            msg.header=Header(stamp=rospy.Time.from_sec(current[0]),frame_id=meta['reference_frame'])
            msg.cloud_local=point_cloud2.create_cloud_xyz32(Header(stamp=msg.header.stamp,frame_id=label+'_current_key'),points)
            p=msg.pose_in_lio_map.position;p.x,p.y,p.z=matrix[:3,3];q=msg.pose_in_lio_map.orientation
            q.x,q.y,q.z,q.w=Rotation.from_matrix(matrix[:3,:3]).as_quat();msg.gravity_alignment.w=1
            publisher=rospy.Publisher('/stage2/'+label+'/submap',RobotSubmap,queue_size=1);initial_pubs.append(publisher)
            wait(lambda:publisher.get_num_connections()>0);publisher.publish(msg)
        wait(lambda: bool(alignments) and alignments[-1].valid and bool(maps) and maps[-1][1].width>0)
        assert matches and matches[0].reference_submap_id==1 and matches[0].query_submap_id==1
        report['historical_source_pair']=[matches[0].reference_submap_id,matches[0].query_submap_id]
        report['warm_relation_revalidated']=True
        late=[];latched=rospy.Subscriber('/stage3/fused_map',PointCloud2,lambda m:late.append(m),queue_size=1)
        wait(lambda:bool(late));assert late[-1].width==maps[-1][1].width
        retry=rospy.ServiceProxy('/stage3/retry_history',Trigger);rospy.wait_for_service('/stage3/retry_history',timeout=5)
        started=time.monotonic();response=retry();report['retry_service_wall_s']=time.monotonic()-started;assert response.success and report['retry_service_wall_s']<.2
        source=next(p for p in a.warm_inputs.glob('ugv*.yaml') if yaml.safe_load(p.read_text())['submap_id']==2)
        meta=yaml.safe_load(source.read_text());points=read_pcd(source.with_suffix('.pcd'));points=np.vstack((points,[[7.13,12.37,1.83]]))
        msg=RobotSubmap();msg.robot_id=0;msg.session_id=meta['session_id'];msg.epoch=meta['epoch'];msg.submap_id=4;msg.partial=True
        msg.header=Header(stamp=rospy.Time.from_sec(meta['keyframe_timestamp']),frame_id=meta['reference_frame'])
        msg.cloud_local=point_cloud2.create_cloud_xyz32(Header(stamp=msg.header.stamp,frame_id='new_ugv_key'),points)
        matrix=np.array(meta['T_reference_key']);p=msg.pose_in_lio_map.position;p.x,p.y,p.z=matrix[:3,3]
        q=msg.pose_in_lio_map.orientation;q.x,q.y,q.z,q.w=Rotation.from_matrix(matrix[:3,:3]).as_quat();msg.gravity_alignment.w=1
        pub=rospy.Publisher('/stage2/ugv/submap',RobotSubmap,queue_size=1);wait(lambda:pub.get_num_connections()>0)
        before=maps[-1][1].width;pub.publish(msg);wait(lambda:maps[-1][1].width>before)
        report['map_continues_after_alignment']=True;count=maps[-1][1].width
        pub.publish(msg);time.sleep(1.3);assert maps[-1][1].width==count
        report['duplicate_preserves_map']=True
        msg.epoch=1;msg.submap_id=0;pub.publish(msg)
        wait(lambda:not alignments[-1].valid and maps[-1][1].width==0)
        report['epoch_reset_clears_latched_map']=True
        current[0]=10.;tick();time.sleep(.5)
        assert not alignments[-1].valid and maps[-1][1].width==0
        report['clock_rollback_invalidates']=True
        previous=len(status);msg.submap_id=1;pub.publish(msg)
        wait(lambda:len(status)>previous and status[-1].get('last_reason')=='clock or odometry reset requires a new Stage 2 epoch')
        assert not alignments[-1].valid and maps[-1][1].width==0
        assert (status[-1].get('last_diagnostics') or {}).get('map_ugv_voxels',0)==0
        report['old_epoch_input_after_clock_rollback_blocked']=True
        valid_updates=[t for t,m in maps if m.width>0]
        assert all(b-a>=.95 for a,b in zip(valid_updates,valid_updates[1:]))
        report['map_wall_rate_hz_max']=1/min((b-a for a,b in zip(valid_updates,valid_updates[1:])),default=1)
        report['worker_queue_drops']=max((s.get('worker_queue_drops',0) for s in status),default=0);assert report['worker_queue_drops']==0
        report['passed']=True
    except Exception as error:report['error']=repr(error)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for log in logs:log.close()
        (a.output/'checks.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
