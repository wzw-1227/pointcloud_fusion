#!/usr/bin/env python3
"""Replay saved inputs and inspect the new fused output, without Gazebo/control."""
import argparse
import copy
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rosgraph
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stage3_verify_results import verify
from run_ros_checks import unused_port


def main():
    parser=argparse.ArgumentParser();parser.add_argument('input_directory',type=Path)
    parser.add_argument('--snapshot',action='store_true');parser.add_argument('--distance',type=float,default=30)
    parser.add_argument('--config',type=Path,help='Explicit candidate config; defaults to production stage3.yaml')
    args=parser.parse_args();package=Path(__file__).resolve().parents[1]
    root=package.parents[1]/'results/stage3'/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_fusion_replay_checks')
    root.mkdir(parents=True);env=os.environ.copy();env['ROS_MASTER_URI']='http://127.0.0.1:'+str(unused_port())
    os.environ['ROS_MASTER_URI']=env['ROS_MASTER_URI'];processes=[];logs=[]
    result={'status':'running','input_directory':str(args.input_directory.resolve()),
            'configuration':str((args.config or package/'config/stage3.yaml').resolve())}
    settings=yaml.safe_load((args.config or package/'config/stage3.yaml').read_text())
    frame=settings.get('ugv_frame','odom')
    def start(command,name):
        log=open(root/(name+'.log'),'w');logs.append(log)
        p=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT);processes.append(p);return p
    def stop(p):
        if p.poll() is None:p.send_signal(signal.SIGINT);p.wait(timeout=20)
    try:
        command=['roslaunch','-p',env['ROS_MASTER_URI'].rsplit(':',1)[1],'ehpr_bridge','stage3_replay.launch',
                 'input_directory:='+str(args.input_directory.resolve()),'rviz:=false','output_root:='+str(root/'matches'),
                 'reference_frame:='+frame]
        if args.config:command.append('config:='+str(args.config.resolve()))
        launch=start(command,'replay')
        deadline=time.monotonic()+30
        while not rosgraph.is_master_online() and time.monotonic()<deadline:time.sleep(.1)
        import rospy
        from sensor_msgs.msg import PointCloud2
        rospy.init_node('fusion_replay_check',anonymous=True,disable_signals=True)
        # Read latched clouds after replay; match success itself is not latched.
        fused=rospy.wait_for_message('/fused_submap',PointCloud2,timeout=35)
        assert fused.width*fused.height>0 and fused.header.frame_id==frame
        assert 'rgb' in [field.name for field in fused.fields]
        run=next((root/'matches').iterdir());result['export_directory']=str(run)
        result['file_checks']=verify(run)
        if args.snapshot:
            env['DISABLE_ROS1_EOL_WARNINGS']='1'
            config=yaml.safe_load((package.parent/'start_up/rviz/stage3_matches.rviz').read_text())
            config['Visualization Manager']['Global Options']['Fixed Frame']=frame
            for display in config['Visualization Manager']['Displays']:
                if display.get('Class')=='rviz/Grid':display['Reference Frame']=frame
            config['Visualization Manager']['Views']['Current']['Distance']=args.distance
            for view in ('ugv_only','fused'):
                current=copy.deepcopy(config)
                for d in current['Visualization Manager']['Displays']:
                    if d.get('Class')!='rviz/PointCloud2':continue
                    enabled=(d['Topic']=='/stage3/match_cloud_ugv' if view=='ugv_only' else d['Topic']=='/fused_submap')
                    d['Enabled']=d['Value']=enabled
                filename=root/('stage3_fusion_'+view+'.rviz');filename.write_text(yaml.safe_dump(current,sort_keys=False))
                rviz=start(['rosrun','rviz','rviz','-d',str(filename)],'rviz_'+view)
                deadline=time.monotonic()+25;window=None
                while time.monotonic()<deadline and rviz.poll() is None:
                    tree=subprocess.run(['xwininfo','-root','-tree'],env=env,capture_output=True,text=True).stdout
                    for line in tree.splitlines():
                        if filename.name not in line:continue
                        candidate=line.strip().split()[0]
                        prop=subprocess.run(['xprop','-id',candidate,'_NET_WM_PID'],env=env,capture_output=True,text=True).stdout
                        if str(rviz.pid)==prop.rsplit('=',1)[-1].strip():window=candidate;break
                    if window:break
                    time.sleep(.2)
                if not window:raise RuntimeError('Owned RViz window did not appear')
                time.sleep(5);image=root/('rviz_'+view+'.png')
                capture='from PyQt5.QtWidgets import QApplication; import sys; app=QApplication([]); ok=app.primaryScreen().grabWindow(int(sys.argv[1],16)).save(sys.argv[2]); sys.exit(0 if ok else 1)'
                subprocess.run(['python3','-c',capture,window,str(image)],env=env,check=True,timeout=15)
                result[view+'_image']=str(image);stop(rviz)
        result['status']='passed'
    except Exception as exc:
        result['status']='failed';result['error']=str(exc);print('FAIL:',exc,flush=True)
    finally:
        for p in reversed(processes):stop(p)
        for log in logs:log.close()
        if result['status']=='passed':result['file_checks']=verify(Path(result['export_directory']))
        (root/'checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print('CHECKS_DIR='+str(root),flush=True)
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
