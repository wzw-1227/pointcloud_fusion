#!/usr/bin/env python3
"""Exercise startup recording, phase persistence and load telemetry without vehicles."""
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid


def main():
    root=Path(__file__).resolve().parents[1]
    directory=root.parents[1]/'results/stability'/(
        datetime.now().strftime('%Y%m%d_%H%M%S')+'_recording_checks')
    directory.mkdir(exist_ok=False)
    with socket.socket() as socket_:
        socket_.bind(('127.0.0.1',0));port=socket_.getsockname()[1]
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(port)
    processes=[];logs=[];result={'status':'running','input':'synthetic recorder test; no Gazebo or vehicle control'}

    def start(args,name):
        log=open(directory/(name+'.log'),'w');logs.append(log)
        process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
        processes.append(process);return process

    try:
        import rosgraph
        start(['roscore','-p',str(port)],'master')
        deadline=time.monotonic()+20
        while not rosgraph.is_master_online() and time.monotonic()<deadline:time.sleep(.1)
        if not rosgraph.is_master_online():raise RuntimeError('isolated master did not start')
        import rospy
        from std_msgs.msg import String
        rospy.init_node('recording_check',anonymous=True,disable_signals=True)
        rospy.set_param('/run_id',str(uuid.uuid4()))
        rospy.set_param('/stage1/robot','both')
        rospy.set_param('/stage1/contract_file',str(root/'config/stage1_stable_contract.yaml'))
        rospy.set_param('/stage1/sim_profile','stable')
        rospy.set_param('/stage1/uav_sensor_profile','wide')
        rospy.set_param('/stage1/world_file',str(root/'worlds/stage1_stable.world'))
        rospy.set_param('/uav/lio_sam/pointCloudTopic','/diagnostic/uav/points')
        diagnostic_source=directory/'isolated_sensor_source.sdf'
        diagnostic_source.write_text((root/'models/stage1_lidar_wide/stage1_lidar_wide.sdf').read_text().replace('<samples>16</samples>','<samples>32</samples>'))
        rospy.set_param('/diagnostic/source_files',[str(diagnostic_source)])
        seen=[]
        sub=rospy.Subscriber('/stage3/recording',String,lambda m:seen.append(json.loads(m.data)),queue_size=5)
        recorder=start(['rosrun','start_up','stage3_record.py','_output_root:='+str(directory/'recordings')],'recorder')
        deadline=time.monotonic()+20
        while not seen and time.monotonic()<deadline and recorder.poll() is None:time.sleep(.1)
        if not seen:raise RuntimeError('no startup recording status')
        run=Path(seen[-1]['directory']);result['recording_directory']=str(run)
        pub=rospy.Publisher('/stage3/test_phase',String,queue_size=1)
        deadline=time.monotonic()+15
        # Recorder and rosbag must both receive the phase message.
        while pub.get_num_connections()<2 and time.monotonic()<deadline:time.sleep(.1)
        if pub.get_num_connections()<2:raise RuntimeError('phase subscribers did not connect')
        pub.publish(String(data=json.dumps({'phase':'hover','stamp':rospy.get_time()})))
        time.sleep(2.5)
        recorder.send_signal(signal.SIGINT);recorder.wait(timeout=20)
        if recorder.returncode:raise RuntimeError('recorder exit code '+str(recorder.returncode))
        session=json.loads((run/'session.json').read_text())
        if session['status']!='recorded_review_pending' or session['unfinished_bags']:raise RuntimeError('recording was not finalized')
        if '/diagnostic/uav/points' not in session['topics']:raise RuntimeError('actual configured LIO input was not selected for recording')
        versions=json.loads((run/'runtime_sources.json').read_text())
        lidar=str(root/'models/stage1_lidar_wide/stage1_lidar_wide.sdf')
        if session['uav_sensor_profile']!='wide' or not versions[lidar].get('sha256'):raise RuntimeError('selected candidate source was not recorded')
        import hashlib
        if versions.get(str(diagnostic_source),{}).get('sha256')!=hashlib.sha256(diagnostic_source.read_bytes()).hexdigest():
            raise RuntimeError('actual isolated sensor source was not recorded')
        samples=[json.loads(line) for line in (run/'resources.jsonl').read_text().splitlines()]
        if len(samples)<2 or samples[-1]['wall_elapsed_s']<=samples[0]['wall_elapsed_s']:raise RuntimeError('missing load/clock telemetry')
        import rosbag
        bags=list(run.glob('data*.bag'));phase_messages=0
        for path in bags:
            with rosbag.Bag(str(path)) as bag:
                phase_messages+=bag.get_message_count('/stage3/test_phase')
                compression=bag.get_compression_info()
                if compression.compression!='lz4' or compression.uncompressed<=0:raise RuntimeError('bag compression is not LZ4')
        if not phase_messages or len(session['phases'])!=1:raise RuntimeError('phase missing from bag or session metadata')
        result.update(status='passed',finalized_bags=len(bags),phase_messages=phase_messages,
                      resource_samples=len(samples),compression='lz4',vehicle_commands_sent=0)
    except Exception as exc:result.update(status='failed',error=str(exc))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5)
        for log in logs:log.close()
        (directory/'checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));return 0 if result['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
