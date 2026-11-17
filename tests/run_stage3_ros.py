#!/usr/bin/env python3
"""Isolated ROS serialization, pending/retry, reset, disk-failure and loop guard checks."""
from datetime import datetime
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import rosgraph
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from stage2_verify_exports import read_pcd
from stage3_verify_results import verify
from run_ros_checks import unused_port


def wait_for(predicate, seconds=15):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.1)
    raise RuntimeError('Check timed out')


def main():
    package=Path(__file__).resolve().parents[1]
    directory=package.parents[1]/'results/stage3'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_ros_checks')
    directory.mkdir(parents=True)
    env=os.environ.copy();env['ROS_MASTER_URI']='http://127.0.0.1:'+str(unused_port())
    os.environ['ROS_MASTER_URI']=env['ROS_MASTER_URI']
    processes=[];logs=[];checks={}
    def start(command, name):
        log=open(directory/(name+'.log'),'w');logs.append(log)
        p=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT);processes.append(p);return p
    def stop(p):
        if p.poll() is None:p.send_signal(signal.SIGINT);p.wait(timeout=20)
    try:
        start(['roscore','-p',env['ROS_MASTER_URI'].rsplit(':',1)[1]],'master')
        wait_for(rosgraph.is_master_online)
        import rospy
        from ehpr_bridge.msg import RobotSubmap, InterRobotMatch
        from sensor_msgs import point_cloud2
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Header
        from tf.transformations import quaternion_from_matrix
        rospy.init_node('stage3_ros_checks',anonymous=True,disable_signals=True)
        rospy.set_param('/use_sim_time',False)
        config=yaml.safe_load((package/'config/stage3.yaml').read_text());config['database_capacity']=2
        config_path=directory/'test_config.yaml';config_path.write_text(yaml.safe_dump(config))
        publishers=[rospy.Publisher('/stage2/'+r+'/submap',RobotSubmap,queue_size=10) for r in ('ugv','uav')]
        fixtures=[]
        fixture_root=package.parents[1]/'results/stage3/synthetic_known_pose_inputs'
        for robot in ('ugv','uav'):
            y=yaml.safe_load((fixture_root/(robot+'_submap_000.yaml')).read_text())
            cloud=read_pcd(fixture_root/(robot+'_submap_000.pcd'))
            m=RobotSubmap();m.robot_id=y['robot_id'];m.session_id=y['session_id'];m.epoch=0;m.submap_id=0;m.partial=True
            m.header=Header(stamp=rospy.Time.from_sec(y['keyframe_timestamp']),frame_id=y['reference_frame'])
            m.cloud_local=point_cloud2.create_cloud_xyz32(Header(stamp=m.header.stamp,frame_id=y['local_frame']),cloud)
            pose=np.asarray(y['T_reference_key']);q=quaternion_from_matrix(pose)
            m.pose_in_lio_map.position.x,m.pose_in_lio_map.position.y,m.pose_in_lio_map.position.z=pose[:3,3]
            m.pose_in_lio_map.orientation.x,m.pose_in_lio_map.orientation.y,m.pose_in_lio_map.orientation.z,m.pose_in_lio_map.orientation.w=q
            g=quaternion_from_matrix(np.asarray(y['T_gravity_key']))
            m.gravity_alignment.x,m.gravity_alignment.y,m.gravity_alignment.z,m.gravity_alignment.w=g
            fixtures.append(m)
        def send(robot,id=0,epoch=0,invalid=False):
            m=copy.deepcopy(fixtures[robot]);m.submap_id=id;m.epoch=epoch
            if invalid:m.cloud_local.header.stamp=rospy.Time(0)
            publishers[robot].publish(m)
        for mode in ('pipeline','io_failure','fused_io_failure','loop_guard','quality_guard'):
            for prefix in ('/lio_sam','/uav/lio_sam'):rospy.set_param(prefix+'/loopClosureEnableFlag',mode=='loop_guard')
            root=directory/mode;matches=[];empty=[];fused=[];root_fused=[]
            subs=[rospy.Subscriber('/stage3/match',InterRobotMatch,lambda m:matches.append(m),queue_size=20),
                  rospy.Subscriber('/stage3/match_cloud_ugv',PointCloud2,lambda m:empty.append(m) if m.width*m.height==0 else None,queue_size=20),
                  rospy.Subscriber('/stage3/fused_submap',PointCloud2,lambda m:fused.append(m),queue_size=20),
                  rospy.Subscriber('/fused_submap',PointCloud2,lambda m:root_fused.append(m),queue_size=20)]
            if mode=='quality_guard':
                from rosgraph_msgs.msg import Clock
                rospy.set_param('/use_sim_time',True)
                clock=rospy.Publisher('/clock',Clock,queue_size=10,latch=True)
                clock.publish(Clock(clock=rospy.Time.from_sec(100)))
            node=start(['rosrun','ehpr_bridge','stage3_matching_node','_config:='+str(config_path),
                        '_output_root:='+str(root),'_require_lio_quality:='+('true' if mode=='quality_guard' else 'false')],mode)
            wait_for(lambda:root.exists() and bool(list(root.glob('*/summary.yaml'))))
            run=next(root.iterdir())
            if mode=='loop_guard':
                wait_for(lambda:node.poll() is not None)
                summary=yaml.safe_load((run/'summary.yaml').read_text())
                assert node.returncode!=0 and summary['status']=='failed' and not matches
            else:
                wait_for(lambda:all(p.get_num_connections()>0 for p in publishers))
                time.sleep(.3);assert not fused and not root_fused
                if mode=='quality_guard':
                    from lio_sam.msg import Scan2MapDebug
                    from rosgraph_msgs.msg import Clock
                    diagnostics=[rospy.Publisher(prefix+'/debug/scan2map',Scan2MapDebug,queue_size=10) for prefix in ('/lio_sam','/uav/lio_sam')]
                    wait_for(lambda:all(p.get_num_connections() for p in diagnostics))
                    def health(robot,stamp,good):
                        m=Scan2MapDebug();m.header.stamp=rospy.Time.from_sec(stamp)
                        m.header.frame_id='odom' if robot==0 else 'uav_lio_odom'
                        m.optimization_performed=True;m.is_degenerate=not good;m.weak_direction_count=0 if good else 1;m.lambda_min=200 if good else 20
                        diagnostics[robot].publish(m)
                    for r in (0,1):health(r,fixtures[r].header.stamp.to_sec(),True);health(r,100,True)
                    time.sleep(.3);send(0);send(1);wait_for(lambda:len(matches)==1 and root_fused)
                    clock.publish(Clock(clock=rospy.Time.from_sec(101)));health(0,101,False);time.sleep(.2)
                    clock.publish(Clock(clock=rospy.Time.from_sec(103)));health(0,103,False);health(1,103,True)
                    wait_for(lambda:any(m.width*m.height==0 for m in root_fused))
                    send(0,id=1);send(1,id=1);time.sleep(.5)
                    assert len(matches)==1
                    stop(node);summary=yaml.safe_load((run/'summary.yaml').read_text())
                    assert summary['counters']['lio_quality_invalidations']==1 and summary['database_size']==0
                    assert any('quality quarantine' in e['reason'] for e in summary['events'])
                    rospy.set_param('/use_sim_time',False)
                elif mode in ('io_failure','fused_io_failure'):

                    (run/('match_000.tmp/fused_submap.pcd' if mode=='fused_io_failure' else 'match_000.tmp/reference.pcd')).mkdir(parents=True)
                    send(0);send(1)
                    wait_for(lambda:node.poll() is not None,30)
                    summary=yaml.safe_load((run/'summary.yaml').read_text())
                    assert node.returncode!=0 and summary['status']=='failed' and not matches and not fused and not root_fused
                    assert not list(run.glob('match_*/match.yaml'))
                else:
                    send(1);time.sleep(.7);send(0)
                    wait_for(lambda:len(matches)==1,30)
                    wait_for(lambda:any(m.width*m.height>0 for m in fused))
                    wait_for(lambda:any(m.width*m.height>0 for m in root_fused))
                    cloud=next(m for m in reversed(fused) if m.width*m.height>0)
                    assert cloud.header.frame_id=='odom' and cloud.header.stamp.to_sec()==0
                    assert 'rgb' in [field.name for field in cloud.fields]
                    saved=read_pcd(Path(matches[0].result_path).parent/'fused_submap.pcd')
                    xyz=np.array(list(point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=False)))
                    assert xyz.shape==saved.shape and np.allclose(xyz,saved,atol=1e-6)
                    late=rospy.wait_for_message('/stage3/fused_submap',PointCloud2,timeout=3)
                    assert late.width*late.height==len(saved)
                    send(0);send(1);time.sleep(.5);assert len(matches)==1
                    send(0,1);send(0,2);send(1,9,invalid=True);time.sleep(.8)
                    send(0,3,epoch=1);time.sleep(.5);send(1,3,epoch=1)
                    wait_for(lambda:len(matches)==2,30)
                    assert empty and matches[-1].reference_epoch==1 and matches[-1].query_epoch==1
                    assert any(m.width*m.height==0 for m in fused)
                    assert all('rgb' in [field.name for field in m.fields] for m in fused)
                    assert any(m.width*m.height==0 for m in root_fused)
                    stop(node);summary=yaml.safe_load((run/'summary.yaml').read_text());verify(run)
                    assert summary['counters']['duplicates']==2 and summary['counters']['database_rebuilds']==1
                    assert any(e['status']=='rejected' for e in summary['events'])
            checks[mode]={'status':'passed','run_directory':str(run),'accepted_messages':len(matches)}
            for s in subs:s.unregister()
            stop(node)
    except Exception as exc:
        checks['failure']={'status':'failed','error':str(exc)};print('FAIL:',exc,flush=True)
    finally:
        for p in reversed(processes):stop(p)
        for log in logs:log.close()
        (directory/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    print('CHECKS_DIR='+str(directory),flush=True)
    return 1 if 'failure' in checks else 0


if __name__=='__main__':raise SystemExit(main())
