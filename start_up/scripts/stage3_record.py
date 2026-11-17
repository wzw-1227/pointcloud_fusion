#!/usr/bin/env python3
"""Startup recorder: bounded compressed chunks, immutable run metadata, manual phases."""
from datetime import datetime,timezone,timedelta
import json
import hashlib
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import rospy
import rospkg
from roslib.packages import find_node
import yaml
from std_msgs.msg import String
from stage1_tools.contracts import load_contract,selected_robots

def resource_sample(start_wall, start_ros):
    """Host-level load and clock progress; not algorithm or control input."""
    memory={}
    for line in Path('/proc/meminfo').read_text().splitlines():
        name,value=line.split(':',1)
        if name in ('MemTotal','MemAvailable'):memory[name]=int(value.split()[0])*1024
    elapsed=time.monotonic()-start_wall
    sim_elapsed=rospy.get_time()-start_ros
    return {'wall_elapsed_s':elapsed,'ros_elapsed_s':sim_elapsed,
            'mean_real_time_factor':sim_elapsed/elapsed if elapsed else None,
            'host_load_1_5_15min':list(os.getloadavg()),'host_cpu_count':os.cpu_count(),
            'memory_bytes':memory}

def runtime_sources():
    pack=rospkg.RosPack();records={}
    for package,executable,sources in (
        ('lio_sam','lio_sam_mapOptmization',('src/mapOptmization.cpp','include/scanStateGuard.h')),
        ('ehpr_bridge','stage3_matching_node',('src/matching_node.cpp','patches/ehpr_stage3.patch'))):
        base=Path(pack.get_path(package))
        paths=[base/name for name in sources]+[Path(p) for p in find_node(package,executable) or []]
        for path in paths:
            records[str(path)]={'available':path.is_file()}
            if path.is_file():records[str(path)]['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    base=Path(pack.get_path('start_up'))
    profile=rospy.get_param('/stage1/uav_sensor_profile','')
    assets=[]
    if profile in ('stable','wide'):
        suffix='dense' if profile=='stable' else 'wide'
        for model in ('lidar','iris'):
            asset=base/('models/stage1_'+model+'_'+suffix)
            assets.extend((asset/'model.config',asset/('stage1_'+model+'_'+suffix+'.sdf')))
    world=rospy.get_param('/stage1/world_file','')
    if world:assets.append(Path(world))
    # An isolated sensor trial may load copies outside the package's defaults.
    assets.extend(Path(path) for path in rospy.get_param('/diagnostic/source_files',[]))
    for path in assets:
        records[str(path)]={'available':path.is_file()}
        if path.is_file():records[str(path)]['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    return records

def main():
    rospy.init_node('stage3_record',disable_signals=True)
    package=Path(rospkg.RosPack().get_path('start_up'))
    directory=Path(rospy.get_param('~output_root',str(package.parents[1]/'results/stability')))/(
        datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')+'_'+str(os.getpid()))
    directory.mkdir(parents=True,exist_ok=False)
    frozen=directory/'contract.yaml'
    shutil.copyfile(rospy.get_param('/stage1/contract_file'),frozen)
    selection=rospy.get_param('/stage1/robot')
    contract=load_contract(str(frozen),selection,'sim')
    topics=['/tf','/tf_static','/clock','/rosout_agg','/stage1/health','/stage3/status','/stage3/match',
            '/stage3/test_phase','/fused_submap','/stage3/fused_submap','/stage3/match_cloud_ugv','/stage3/match_cloud_uav',
            '/gazebo/model_states','/cmd_vel','/stage1/ugv/cmd_vel_raw','/iris_0/mavros/local_position/pose',
            '/iris_0/mavros/state','/iris_0/mavros/setpoint_position/local']
    for name in selected_robots(selection):
        r=contract['robots'][name]
        topics += [r[k] for k in ('raw_cloud','lio_input_cloud','imu','odometry','registered','registered_raw','deskewed','path','debug','ground_truth')]
        prefix='' if name=='ugv' else '/uav'
        # Include the actual LIO input if a diagnostic config differs from the
        # contract's normal gate output; preserve both streams and timestamps.
        actual_input=rospy.get_param(prefix+'/lio_sam/pointCloudTopic',r['lio_input_cloud'])
        topics.append(actual_input if actual_input.startswith('/') else prefix+'/'+actual_input)
        topics += [prefix+'/lio_sam/mapping/odometry_incremental',prefix+'/odometry/imu_incremental',
                   '/stage1/'+name+'/lidar_input_stats','/stage2/'+name+'/submap','/stage2/'+name+'/status']
    phases=[];lock=threading.Lock();stop=threading.Event();callback_errors=[]
    def mark(msg):
        try:
            value=json.loads(msg.data)
            if value['phase'] not in ('hover','ugv_motion','uav_motion','height','coexist','end'):raise ValueError('unknown phase')
            if abs(float(value['stamp'])-rospy.get_time())>2:raise ValueError('stale phase marker')
            with lock:
                phases.append(value)
                temporary=directory/'phases.json.tmp'
                temporary.write_text(json.dumps(phases,indent=2)+'\n');temporary.replace(directory/'phases.json')
            rospy.loginfo('Manual phase: %s',value['phase'])
        except (ValueError,KeyError,TypeError) as exc:rospy.logerr('Invalid phase: %s',exc)
        except OSError as exc:
            callback_errors.append('phase persistence failed: '+str(exc));stop.set();rospy.logfatal('%s',callback_errors[-1])
    sub=rospy.Subscriber('/stage3/test_phase',String,mark,queue_size=20)
    status=rospy.Publisher('/stage3/recording',String,queue_size=1,latch=True)
    metadata={'status':'recording','robot':selection,'topics':sorted(set(topics)), 'timezone':'Asia/Shanghai',
              'controls':'manual; recorder sends no vehicle commands','alignment':'one initial rigid alignment per robot for the whole run',
              'phase_review':'pending','ros_run_id':rospy.get_param('/run_id'),'started_ros':rospy.get_time()}
    metadata.update(sim_profile=rospy.get_param('/stage1/sim_profile','unknown'),
                    uav_sensor_profile=rospy.get_param('/stage1/uav_sensor_profile','unknown'))
    (directory/'parameters_start.yaml').write_text(yaml.safe_dump(rospy.get_param('/'),sort_keys=False))
    for name in ('stable_source_record.json','uav_wide_source_record.json'):
        if (package/'config'/name).exists():shutil.copyfile(package/'config'/name,directory/name)
    shutil.copytree(package/'patches',directory/'patches')
    (directory/'runtime_sources.json').write_text(json.dumps(runtime_sources(),indent=2)+'\n')
    (directory/'session.json').write_text(json.dumps(metadata,indent=2)+'\n')
    logfile=open(directory/'recorder.log','w')
    resources=open(directory/'resources.jsonl','w')
    process=subprocess.Popen(['rosbag','record','--lz4','--split','--size=512','--buffsize=512',
        '-O',str(directory/'data')]+metadata['topics'],stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
    start_wall=time.monotonic();start_ros=rospy.get_time()
    rospy.on_shutdown(stop.set)
    error=None
    rospy.loginfo('Recording -> %s ; mark phases with rosrun start_up stage3_phase.py hover',directory)
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            if process.poll() is not None:raise RuntimeError('rosbag recorder exited early')
            if shutil.disk_usage(directory).free<5*1024**3:raise RuntimeError('less than 5 GiB free; recording stopped without deleting results')
            resources.write(json.dumps(resource_sample(start_wall,start_ros),allow_nan=False)+'\n');resources.flush()
            status.publish(String(data=json.dumps({'status':'recording','directory':str(directory),'ros_stamp':rospy.get_time()})))
            stop.wait(1)
    except KeyboardInterrupt:pass
    except Exception as exc:error=str(exc);rospy.logfatal('%s',error)
    finally:
        if callback_errors:error=callback_errors[-1]
        resources.close()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3);error=error or 'recorder finalization timeout'
        logfile.close()
        if process.returncode:error=error or 'rosbag recorder exit code '+str(process.returncode)
        deadline=time.monotonic()+5
        while list(directory.glob('*.active')) and time.monotonic()<deadline:time.sleep(.1)
        active=list(directory.glob('*.active'))
        metadata.update(status='failed' if error or active or not list(directory.glob('*.bag')) else 'recorded_review_pending',
                        error=error,recorder_exit_code=process.returncode,ended_ros=rospy.get_time(),
                        unfinished_bags=[p.name for p in active],wall_elapsed_s=time.monotonic()-start_wall,
                        resource_file='resources.jsonl',resource_scope='host load, memory and ROS/wall clock; includes other host processes')
        with lock:metadata['phases']=phases
        (directory/'session.json').write_text(json.dumps(metadata,indent=2)+'\n')
        try:(directory/'parameters_end.yaml').write_text(yaml.safe_dump(rospy.get_param('/'),sort_keys=False))
        except Exception:pass
        logs=Path(os.environ.get('ROS_LOG_DIR',str(Path.home()/'.ros/log')))/metadata['ros_run_id']
        if logs.exists():shutil.copytree(logs,directory/'ros_logs',ignore=shutil.ignore_patterns('*.bag','*.active'))
        (directory/'report.md').write_text('# Stability recording\n\nStatus: '+metadata['status']+'\n\n'+
            'Evaluate after shutdown:\n\n```bash\nrosrun start_up stage3_evaluate.py '+str(directory)+'\n```\n\n'+
            'Hover/motion, map review and screenshot acceptance are pending until evaluated.\n')
    return 1 if metadata['status']=='failed' else 0
if __name__=='__main__':raise SystemExit(main())
