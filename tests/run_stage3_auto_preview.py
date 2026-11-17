#!/usr/bin/env python3
"""Synthetic sensor-to-fusion test: stationary database and query, automatic previews.

Uses an isolated ROS master and simulated clock. No Gazebo, control, flush service,
or ground-truth hint is supplied to the algorithms.
"""
from datetime import datetime
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
from run_stage3_ros import wait_for


def main():
    package=Path(__file__).resolve().parents[1]
    directory=package.parents[1]/'results/stage3'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_auto_preview_checks')
    directory.mkdir(parents=True)
    env=os.environ.copy();env['ROS_MASTER_URI']='http://127.0.0.1:'+str(unused_port())
    os.environ['ROS_MASTER_URI']=env['ROS_MASTER_URI']
    processes=[];logs=[];result={'status':'running','inputs':'synthetic complementary scene',
        'controls':'none','flush_calls':0,'quality_thresholds':'unchanged stage3.yaml',
        'timing':'shortened test preview intervals and warmup; production config unchanged'}
    def start(command,name):
        log=open(directory/(name+'.log'),'w');logs.append(log)
        p=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT);processes.append(p);return p
    try:
        start(['roscore','-p',env['ROS_MASTER_URI'].rsplit(':',1)[1]],'master')
        wait_for(rosgraph.is_master_online)
        import rospy
        from rosgraph_msgs.msg import Clock
        from nav_msgs.msg import Odometry
        from sensor_msgs import point_cloud2
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Header
        from ehpr_bridge.msg import RobotSubmap,InterRobotMatch
        rospy.init_node('stage3_auto_preview_checks',anonymous=True,disable_signals=True)
        rospy.set_param('/use_sim_time',True)
        for prefix,frame in (('/lio_sam','odom'),('/uav/lio_sam','uav_lio_odom')):
            rospy.set_param(prefix+'/loopClosureEnableFlag',False)
            rospy.set_param(prefix+'/odometryFrame',frame)
            rospy.set_param(prefix+'/odomChildFrame','odom_mapping' if prefix=='/lio_sam' else 'uav_odom_mapping')
        clock=rospy.Publisher('/clock',Clock,queue_size=1,latch=True)
        stamp=100.0
        clock.publish(Clock(clock=rospy.Time.from_sec(stamp)))
        config=yaml.safe_load((package/'config/stage3_live_submaps.yaml').read_text())
        for robot in config['robots'].values():
            robot['preview_warmup_seconds']=.3
            robot['preview_interval_seconds']=.3
            robot['preview_min_frames']=3
        config_path=directory/'test_submaps.yaml';config_path.write_text(yaml.safe_dump(config))
        builder=start(['rosrun','ehpr_bridge','stage2_submap_node','_robot:=both',
            '_config:='+str(config_path),'_output_root:='+str(directory/'submaps')],'builder')
        matcher=start(['rosrun','ehpr_bridge','stage3_matching_node',
            '_config:='+str(package/'config/stage3.yaml'),'_output_root:='+str(directory/'matches')],'matcher')
        submaps={'ugv':[],'uav':[]};fused=[];matches=[]
        subscribers=[rospy.Subscriber('/stage2/'+r+'/submap',RobotSubmap,
            lambda m,r=r:submaps[r].append(m),queue_size=100) for r in submaps]
        subscribers += [rospy.Subscriber('/fused_submap',PointCloud2,lambda m:fused.append(m),queue_size=10),
            rospy.Subscriber('/stage3/match',InterRobotMatch,lambda m:matches.append(m),queue_size=10)]
        pubs={r:(rospy.Publisher(v['cloud_topic'],PointCloud2,queue_size=5),
                 rospy.Publisher(v['odometry_topic'],Odometry,queue_size=5))
              for r,v in config['robots'].items()}
        wait_for(lambda:all(p.get_num_connections()>0 for pair in pubs.values() for p in pair))
        fixture=package.parents[1]/'results/stage3/synthetic_complementary_inputs'
        points={r:read_pcd(fixture/(r+'_submap_000.pcd')) for r in submaps}
        # Query has an extra right-hand building outside the UGV's field of view.
        far=points['uav'][points['uav'][:,0]>12.5]
        assert len(far)>20
        templates={r:point_cloud2.create_cloud_xyz32(Header(frame_id=v['reference_frame']),points[r])
                   for r,v in config['robots'].items()}
        far_cloud=point_cloud2.create_cloud_xyz32(Header(frame_id=config['robots']['uav']['reference_frame']),far)
        def send(z=0.0,overlap=False):
            nonlocal stamp
            stamp=round(stamp+.1,6);t=rospy.Time.from_sec(stamp)
            clock.publish(Clock(clock=t))
            for r,v in config['robots'].items():
                cloud=far_cloud if r=='uav' and not overlap else templates[r]
                cloud.header.stamp=t
                odom=Odometry(header=Header(stamp=t,frame_id=v['reference_frame']),child_frame_id=v['mapping_child'])
                odom.pose.pose.orientation.w=1
                if r=='uav':odom.pose.pose.position.z=z
                pubs[r][1].publish(odom);pubs[r][0].publish(cloud)
            time.sleep(.25)
            assert builder.poll() is None and matcher.poll() is None
        for _ in range(20):send()
        assert submaps['ugv'] and submaps['uav'] and not fused and not matches
        result['stationary_start']={'ugv_previews':len(submaps['ugv']),'uav_previews':len(submaps['uav']),'fusion_messages':0}
        for _ in range(16):send()
        assert submaps['uav'] and not matches and not fused
        result['stationary_without_overlap']={'uav_previews':len(submaps['uav']),'fusion_messages':0}
        for _ in range(120):
            send(overlap=True)
            if matches and any(m.width*m.height>0 for m in fused):break
        assert matches and fused, 'No automatically matched preview with overlapping observations'
        message=next(m for m in reversed(fused) if m.width*m.height>0)
        assert message.header.frame_id=='odom' and 'rgb' in [f.name for f in message.fields]
        assert matches[-1].reference_partial and matches[-1].query_partial
        result['automatic_overlap_fusion']={'match_file':matches[-1].result_path,
            'points':message.width*message.height,'frame':message.header.frame_id,
            'ugv_previews':len(submaps['ugv']),'uav_previews':len(submaps['uav'])}
        run=Path(matches[-1].result_path).parent.parent
        result['file_checks']=verify(run)
        submap_run=next((directory/'submaps').iterdir())
        meta=[yaml.safe_load(p.read_text()) for p in submap_run.glob('*_submap_*.yaml')]
        assert meta and all(m['reason']=='online_preview' and m['partial'] for m in meta)
        assert max(m['segment_travel_distance_m'] for m in meta)<1
        result['maximum_travel_m']=max(m['segment_travel_distance_m'] for m in meta)
        result['status']='passed'
    except Exception as exc:
        result['status']='failed';result['error']=str(exc);print('FAIL:',repr(exc),flush=True)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=20)
                except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=10)
        for log in logs:log.close()
        (directory/'checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print('CHECKS_DIR='+str(directory),flush=True)
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
