"""Bag and live observations, plus explicit simulation acceptance criteria."""
from collections import defaultdict
from bisect import bisect_left
import json
from pathlib import Path
import numpy as np
from .cloud import inspect_cloud, xyz_samples
from .geometry import pose_matrix, initial_alignment, rotation_error_degrees, interpolate_pose


class Observations:
    def __init__(self, contract, robots, cutoff=0):
        self.contract, self.robots, self.cutoff = contract, robots, cutoff
        self.stamps = defaultdict(list)
        self.frames = defaultdict(set)
        self.errors, self.warnings = set(), set()
        self.poses, self.truth, self.debug = defaultdict(list), defaultdict(list), defaultdict(list)
        self.cloud_sizes = defaultdict(list)
        self.raw_info = {}
        self.optimization_skipped = defaultdict(int)
        self.tf_parents, self.tf_callers = defaultdict(set), defaultdict(set)
        self.registration = defaultdict(dict)
        self.registration_errors = defaultdict(list)
        self.imu_acc = defaultdict(list)
        self.health_failures = set()
        self.end = cutoff
        self.routes = {}
        for name in robots:
            r = contract['robots'][name]
            for kind in ('raw_cloud','imu','odometry','registered','registered_raw','deskewed','debug','ground_truth'):
                self.routes[r[kind]] = (name,kind)

    def observe(self, topic, message, reception_time, caller='unknown'):
        if isinstance(caller, bytes):
            caller = caller.decode('utf-8', errors='replace')
        self.end = max(self.end,reception_time)
        if topic in ('/tf','/tf_static'):
            for t in message.transforms:
                self.tf_parents[t.child_frame_id].add(t.header.frame_id)
                self.tf_callers[t.child_frame_id].add(caller)
                q=t.transform.rotation
                quaternion=np.asarray([q.x,q.y,q.z,q.w])
                if not np.all(np.isfinite(quaternion)) or abs(np.linalg.norm(quaternion)-1)>1e-3:
                    self.errors.add('TF invalid quaternion: '+t.child_frame_id)
                p=t.transform.translation
                if not np.all(np.isfinite([p.x,p.y,p.z])):
                    self.errors.add('TF nonfinite translation: '+t.child_frame_id)
            return
        if reception_time < self.cutoff:
            return
        if topic == '/rosout_agg':
            if 'lio_sam' in message.name:
                text=message.msg
                if any(x in text for x in ('Large velocity','Large bias','Quaternion Not Properly Normalized','Invalid quaternion')):
                    self.errors.add(message.name+': '+text)
            return
        if topic == '/stage1/health':
            health=json.loads(message.data)
            if health.get('status') == 'error':
                self.health_failures.update(health.get('errors',[]))
            return
        if topic not in self.routes:
            return
        name,kind=self.routes[topic];r=self.contract['robots'][name]
        stamp=message.header.stamp.to_sec()
        if stamp < self.cutoff:
            return
        self.stamps[topic].append(stamp)
        self.frames[topic].add(message.header.frame_id)
        expected=r['raw_frame'] if kind=='raw_cloud' else r['imu_message_frame'] if kind=='imu' else r['lidar_frame'] if kind=='deskewed' else 'world' if kind=='ground_truth' else r['reference_frame']
        if message.header.frame_id != expected:
            self.errors.add(topic+': frame '+message.header.frame_id+' != '+expected)
        try:
            if kind in ('raw_cloud','registered','registered_raw','deskewed'):
                problems,warnings,info=inspect_cloud(message,r,kind=='raw_cloud')
                self.errors.update(topic+': '+x for x in problems)
                self.warnings.update(name+': '+x for x in warnings)
                self.cloud_sizes[topic].append(info['points'])
                if kind=='raw_cloud':self.raw_info[name]=info
                if kind in ('registered_raw','deskewed') and info['points']:
                    entry=self.registration[name].setdefault(stamp,{})
                    entry[kind]=xyz_samples(message)
                    entry[kind+'_count']=info['points']
                    self._check_registration(name,stamp)
            elif kind in ('odometry','ground_truth'):
                q=message.pose.pose.orientation
                if abs(np.linalg.norm([q.x,q.y,q.z,q.w])-1)>1e-3:
                    self.errors.add(topic+': invalid pose quaternion')
                t=pose_matrix(message.pose.pose)
                (self.truth if kind=='ground_truth' else self.poses)[name].append((stamp,t))
                if kind=='odometry':
                    if message.child_frame_id != r['mapping_child']:
                        self.errors.add(topic+': unexpected pose child frame '+message.child_frame_id)
                    self.registration[name].setdefault(stamp,{})['pose']=t
                    self._check_registration(name,stamp)
            elif kind=='imu':
                q=message.orientation
                quaternion=np.asarray([q.x,q.y,q.z,q.w])
                if not np.all(np.isfinite(quaternion)) or abs(np.linalg.norm(quaternion)-1)>1e-3:
                    self.errors.add(topic+': invalid IMU orientation')
                a=message.linear_acceleration;g=message.angular_velocity
                values=[a.x,a.y,a.z,g.x,g.y,g.z]
                if not np.all(np.isfinite(values)):
                    self.errors.add(topic+': nonfinite IMU')
                else:
                    self.imu_acc[name].append(values[:3])
            elif kind=='debug':
                performed = bool(message.optimization_performed)
                if performed and not np.isfinite(message.lambda_min):
                    self.errors.add(topic+': nonfinite scan-matching eigenvalue after optimization')
                if not performed:
                    self.optimization_skipped[name] += 1
                self.debug[name].append((stamp, int(message.weak_direction_count) if performed else 6,
                                        float(message.lambda_min) if performed else float('nan')))
        except (ValueError,TypeError,KeyError) as exc:
            self.errors.add(topic+': '+str(exc))
        # Buffer bounded samples rather than accumulating full LiDAR messages.
        for old in [t for t in self.registration[name] if t < stamp-2]:
            del self.registration[name][old]

    def _check_registration(self,name,stamp):
        e=self.registration[name][stamp]
        if {'registered_raw','deskewed','pose'}.issubset(e):
            if e['registered_raw_count'] != e['deskewed_count'] or e['registered_raw'].shape != e['deskewed'].shape:
                self.errors.add(name+': registered raw and deskewed cloud sizes disagree')
            else:
                t=e['pose']
                expected=e['deskewed']@t[:3,:3].T+t[:3,3]
                error=float(np.max(np.linalg.norm(expected-e['registered_raw'],axis=1)))
                self.registration_errors[name].append(error)
                if error>0.01:self.errors.add(name+': registered cloud transform mismatch (>1 cm)')
            del self.registration[name][stamp]


def matched_poses(estimated,truth,max_gap):
    truth=sorted(truth,key=lambda x:x[0]);times=[x[0] for x in truth]
    output=[]
    for stamp,pose in sorted(estimated,key=lambda x:x[0]):
        i=bisect_left(times,stamp)
        if i<len(times) and abs(times[i]-stamp)<1e-8:
            output.append((stamp,pose,truth[i][1]));continue
        if i==0 or i==len(times):continue
        left,right=truth[i-1],truth[i]
        if max(stamp-left[0],right[0]-stamp)>max_gap or right[0]<=left[0]:continue
        output.append((stamp,pose,interpolate_pose(left[1],right[1],(stamp-left[0])/(right[0]-left[0]))))
    return output


def trajectory_metrics(estimated,truth,max_gap,alignment=None):
    pairs=matched_poses(estimated,truth,max_gap)
    if len(pairs)<2:return {'matched_samples':len(pairs)}
    if alignment is None:alignment=initial_alignment(pairs[0][1],pairs[0][2])
    position=[];rotation=[]
    for stamp,e,g in pairs:
        e=alignment@e
        position.append(np.linalg.norm(e[:3,3]-g[:3,3]))
        rotation.append(rotation_error_degrees(e,g))
    # 5 Hz truth sampling avoids integrating high-frequency contact jitter.
    coarse=[]
    for stamp,e,g in pairs:
        if not coarse or stamp-coarse[-1][0]>=.19:coarse.append((stamp,g))
    xyz=np.array([t[:3,3] for _,t in coarse])
    yaw=np.unwrap([np.arctan2(t[1,0],t[0,0]) for _,t in coarse])
    deltas=np.abs(np.diff(yaw));turn=float(np.sum(deltas[deltas>.005]))
    return {'matched_samples':len(pairs),'match_fraction':len(pairs)/max(1,len(estimated)),
            'position_rmse_m':float(np.sqrt(np.mean(np.square(position)))),
            'position_max_m':float(max(position)),
            'rotation_rmse_deg':float(np.sqrt(np.mean(np.square(rotation)))),
            'endpoint_position_error_m':float(position[-1]),
            'truth_distance_m':float(np.sum(np.linalg.norm(np.diff(xyz,axis=0),axis=1))),
            'truth_quarter_turns':int((turn+.1)/(np.pi/2)),
            'truth_height_excursion_m':float(np.ptp(xyz[:,2])),
            'truth_height_return_error_m':float(abs(xyz[-1,2]-xyz[0,2])),
            'duration_seconds':float(pairs[-1][0]-pairs[0][0]),
            'alignment_method':'first matched pose only; rigid SE(3), no scale or trajectory fitting'}


def coexist_metrics(truth):
    if not all(len(truth[n])>2 for n in ('ugv','uav')):return {}
    start=max(truth[n][0][0] for n in truth);end=min(truth[n][-1][0] for n in truth)
    grid=np.arange(start,end,.2)
    if len(grid)<3:return {}
    speeds={}
    for name in ('ugv','uav'):
        values=np.array([t[:3,3] for _,t in truth[name]])
        times=[s for s,_ in truth[name]]
        xyz=np.column_stack([np.interp(grid,times,values[:,k]) for k in range(3)])
        speeds[name]=np.linalg.norm(np.diff(xyz,axis=0),axis=1)/.2
    result={}
    for moving,holding in [('ugv','uav'),('uav','ugv')]:
        good=(speeds[moving]>.05)&(speeds[holding]<.03)
        longest=current=0
        for value in good:
            current=current+.2 if value else 0;longest=max(longest,current)
        result[moving+'_moving_other_holding_seconds']=round(longest,3)
    return result


def make_report(obs,selection,scenario,environment='sim',map_review='pending',alignments=None):
    limits=obs.contract['acceptance'];errors=set(obs.errors)|obs.health_failures;warnings=set(obs.warnings)
    result={'robot':selection,'scenario':scenario,'environment':environment,'map_review':map_review,
            'robots':{},'tf':{},'thresholds':limits}
    for child,parents in obs.tf_parents.items():
        callers=obs.tf_callers[child]
        result['tf'][child]={'parents':sorted(parents),'publishers':sorted(callers)}
        if len(parents)>1 or len(callers)>1:errors.add('TF conflict on '+child)
    for name in obs.robots:
        r=obs.contract['robots'][name];details={'streams':{},'raw_cloud':obs.raw_info.get(name,{})}
        required=['raw_cloud','imu','odometry','registered','registered_raw','deskewed','debug']
        if environment=='sim':required.append('ground_truth')
        for kind in required:
            topic=r[kind];stamps=obs.stamps[topic];delta=np.diff(stamps)
            maxgap=float(max(delta)) if len(delta) else None
            tail=max(0,obs.end-stamps[-1]) if stamps else None
            head=max(0,stamps[0]-obs.cutoff) if stamps else None
            info={'messages':len(stamps),'frames':sorted(obs.frames[topic]),'max_gap_seconds':maxgap,
                  'start_gap_seconds':head,'end_gap_seconds':tail,
                  'rate_hz':(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else 0}
            if obs.cloud_sizes[topic]:info['median_points']=float(np.median(obs.cloud_sizes[topic]))
            details['streams'][kind]=info
            if len(stamps)<2:errors.add(name+': missing/insufficient '+kind)
            if len(delta) and np.any(delta<0):errors.add(name+': backwards timestamps in '+kind)
            if len(delta) and np.any(delta==0):
                if environment=='sim' and kind=='raw_cloud':
                    info['duplicate_stamps']=int(np.sum(delta==0))
                    warnings.add(name+': repeated GPU raw scan timestamps; sim input gate forwards first scan only')
                else:errors.add(name+': duplicate timestamps in '+kind)
            if kind in ('raw_cloud','imu','odometry','registered','registered_raw','ground_truth') and stamps:
                if max([maxgap or 0,head or 0,tail or 0])>limits['max_output_gap']:
                    errors.add(name+': output discontinuity in '+kind)
        if r['base_frame'] not in obs.tf_parents or r['lidar_frame'] not in obs.tf_parents:
            errors.add(name+': missing base/lidar TF')
        metric=trajectory_metrics(obs.poses[name],obs.truth[name],limits['max_sync_gap'],
                                  None if alignments is None else alignments.get(name))
        details['trajectory']=metric
        if environment=='sim':
            if metric.get('matched_samples',0)<2:errors.add(name+': cannot associate LiDAR-origin ground truth')
            else:
                if metric['match_fraction']<.8:errors.add(name+': ground truth match coverage below 80%')
                threshold=limits['max_static_rmse'][name] if scenario=='static' else limits['max_pose_rmse'][name]
                if metric['position_rmse_m']>threshold:errors.add(name+': position RMSE exceeds threshold')
                if metric['rotation_rmse_deg']>limits['max_rotation_rmse_deg']:errors.add(name+': orientation RMSE exceeds threshold')
                if scenario=='static':
                    if metric['duration_seconds']<limits['min_static_seconds']:errors.add(name+': static/hover data shorter than 60 seconds')
                    if name=='ugv' and metric['endpoint_position_error_m']>limits['max_static_rmse']['ugv']:
                        errors.add('ugv: 60-second endpoint displacement error exceeds 0.10 m')
                    xyz=np.array([t[:3,3] for _,t in obs.truth[name]])
                    if np.max(np.linalg.norm(xyz-xyz[0],axis=1))>(.15 if name=='ugv' else .4):
                        errors.add(name+': static/hover scenario contains excessive actual motion')
                if name=='uav' and scenario in ('static','height'):
                    initial_height=obs.truth[name][0][1][2,3]
                    if initial_height<limits.get('min_uav_hover_height_m',1.0):
                        errors.add('uav: hover/height test started on the ground or below 1.0 m; take off and stabilize first')
                if scenario=='motion':
                    if metric['truth_distance_m']<limits['min_motion_distance']:errors.add(name+': motion route shorter than 20 m')
                    if metric['truth_quarter_turns']<limits['min_quarter_turns']:errors.add(name+': insufficient cumulative 90-degree turns')
                if scenario=='height' and name=='uav':
                    if metric['truth_height_excursion_m']<limits['min_height_excursion']-.05:errors.add('uav: height excursion below 0.5 m')
                    if metric['truth_height_return_error_m']>.2:errors.add('uav: height did not return to initial hover')
        ds=obs.debug[name];longest=current=0;previous=None;previous_weak=False
        for stamp,weak,_ in ds:
            current=current+(stamp-previous) if weak and previous_weak and previous is not None else 0
            if not weak:current=0
            longest=max(longest,current);previous=stamp;previous_weak=bool(weak)
        details['degeneracy']={'fraction':sum(weak>0 for _,weak,_ in ds)/len(ds) if ds else None,
                               'longest_seconds':longest,
                               'unoptimized_frames':obs.optimization_skipped[name],
                               'min_eigenvalue':min((v for _,_,v in ds if np.isfinite(v)),default=None)}
        if longest>limits['max_continuous_degeneracy']:errors.add(name+': continuous scan-matching degeneracy exceeds 2 seconds; inspect scene support')
        registration=obs.registration_errors[name]
        details['registration_transform']={'checked_frames':len(registration),'max_error_m':max(registration,default=None)}
        if not registration:errors.add(name+': no paired registered-raw/deskewed/pose transform checks')
        acc=obs.imu_acc[name]
        if acc:
            a=np.asarray(acc);details['imu_acceleration']={'mean':a.mean(0).tolist(),'std':a.std(0).tolist(),'min':a.min(0).tolist(),'max':a.max(0).tolist()}
        result['robots'][name]=details
    if scenario=='coexist':
        if selection!='both':errors.add('coexist requires both robots')
        result['coexist']=coexist_metrics(obs.truth)
        for name in ('ugv','uav'):
            if result['coexist'].get(name+'_moving_other_holding_seconds',0)<limits['min_coexist_phase_seconds']:
                errors.add(name+': coexist moving/holding phase shorter than 10 seconds')
    if scenario=='height' and 'uav' not in obs.robots:errors.add('height requires UAV')
    if map_review=='fail':errors.add('manual map review failed')
    result['errors']=sorted(errors);result['warnings']=sorted(warnings)
    result['numeric_pass']=not errors
    result['status']='failed' if errors else 'smoke_complete' if scenario=='smoke' else 'hardware_data_checked' if environment=='real' else 'passed' if map_review=='pass' else 'numeric_pass_map_review_pending'
    return result


def write_report(report,directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    (directory/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    (directory/'tf_snapshot.json').write_text(json.dumps(report['tf'],indent=2,allow_nan=False)+'\n')
    lines=['# Stage 1 report','',f"Status: **{report['status']}**",'',
           f"Robot: {report['robot']}; scenario: {report['scenario']}; environment: {report['environment']}",
           f"Map review: {report['map_review']}",'',
           'Alignment: first associated LiDAR pose only; no scale/full-trajectory fitting.','',
           '| Robot | Position RMSE (m) | Rotation RMSE (deg) | Matched samples | Degenerate fraction |',
           '|---|---:|---:|---:|---:|']
    for name,d in report['robots'].items():
        t=d['trajectory'];lines.append(f"| {name} | {t.get('position_rmse_m','N/A')} | {t.get('rotation_rmse_deg','N/A')} | {t.get('matched_samples',0)} | {d['degeneracy']['fraction']} |")
    for heading,key in [('Failures','errors'),('Warnings / limitations','warnings')]:
        lines+=['', '## '+heading,'']+[('- '+x) for x in report[key]]
        if not report[key]:lines.append('None.')
    lines+=['','Full stream statistics, TF publishers and thresholds are in report.json.','']
    (directory/'report.md').write_text('\n'.join(lines))


def evaluate_bag(bag_path,contract,selection,scenario,warmup=15.,environment='sim',map_review='pending'):
    import rosbag
    from .contracts import selected_robots
    with rosbag.Bag(str(bag_path)) as bag:
        obs=Observations(contract,selected_robots(selection),bag.get_start_time()+warmup)
        for topic,message,stamp,header in bag.read_messages(return_connection_header=True):
            obs.observe(topic,message,stamp.to_sec(),header.get('callerid','unknown'))
    return make_report(obs,selection,scenario,environment,map_review)
