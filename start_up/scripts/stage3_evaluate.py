#!/usr/bin/env python3
"""Evaluate split bags and manual phases with one run-wide alignment."""
import argparse
import heapq
import json
from pathlib import Path
import numpy as np
import rosbag
import yaml
from stage1_tools.contracts import load_contract
from stage1_tools.evaluation import Observations,make_report,matched_poses,trajectory_metrics,write_report
from stage1_tools.geometry import initial_alignment,pose_matrix,rotation_error_degrees

PHASES={'hover':{'ugv':'static','uav':'static'},'ugv_motion':{'ugv':'motion','uav':'static'},
        'uav_motion':{'ugv':'static','uav':'motion'},'height':{'ugv':'static','uav':'height'},
        'coexist':{'ugv':'smoke','uav':'smoke'}}

def inherit_static_tf(observations,static_observations):
    """Latched transforms received before a phase remain valid in that phase."""
    for child,parents in static_observations.tf_parents.items():
        observations.tf_parents[child].update(parents)
        observations.tf_callers[child].update(static_observations.tf_callers[child])
    observations.errors.update(static_observations.errors)

def fixed_alignment(observations,contract):
    result={}
    for robot in ('ugv','uav'):
        pairs=matched_poses(observations.poses[robot],observations.truth[robot],contract['acceptance']['max_sync_gap'])
        if pairs:result[robot]=initial_alignment(pairs[0][1],pairs[0][2])
    return result

def interpolate_truth(truth,stamp):
    found=matched_poses([(stamp,np.eye(4))],truth,.05)
    if not found:raise ValueError('truth does not bracket keyframe time within 0.05 seconds')
    return found[0][2]

def check_pcd(path):
    with open(path,'rb') as stream:
        header={}
        for _ in range(30):
            line=stream.readline()
            if not line:raise ValueError('truncated PCD header')
            parts=line.decode('ascii').split()
            if parts and not parts[0].startswith('#'):header[parts[0]]=parts[1:]
            if parts and parts[0]=='DATA':break
        if header.get('DATA')!=['binary']:raise ValueError('expected binary PCD')
        points=int(header['POINTS'][0]);step=sum(int(a)*int(b) for a,b in zip(header['SIZE'],header['COUNT']))
        if points<=0 or step<=0 or path.stat().st_size-stream.tell()!=points*step:raise ValueError('empty or malformed PCD payload')
        return points

def evaluate(directory,map_review='pending',screenshots=()):
    contract=load_contract(str(directory/'contract.yaml'),'both','sim')
    files=sorted(directory.glob('data*.bag'))
    if not files:raise ValueError('no finalized split bags')
    all_obs=Observations(contract,['ugv','uav']);static_obs=Observations(contract,['ugv','uav'])
    phases=[];current=None;matches=[];statuses=[];end=0;fused_points=[]
    topics=list(all_obs.routes)+['/tf','/tf_static','/rosout_agg','/stage1/health','/stage3/test_phase','/stage3/match','/stage3/status','/fused_submap']
    bags=[rosbag.Bag(str(f)) for f in files]
    try:
        messages=heapq.merge(*(b.read_messages(topics=topics,return_connection_header=True) for b in bags),key=lambda row:row[2].to_sec())
        for topic,msg,stamp,header in messages:
            time=stamp.to_sec();end=max(end,time);caller=header.get('callerid','unknown')
            if topic=='/tf_static':static_obs.observe(topic,msg,time,caller)
            if topic=='/stage3/test_phase':
                marker=json.loads(msg.data);name=marker['phase']
                if current:current['end']=time
                current=None
                if name in PHASES:
                    current={'phase':name,'start':time,'end':None,'observations':{r:Observations(contract,[r],time) for r in ('ugv','uav')}}
                    for obs in current['observations'].values():inherit_static_tf(obs,static_obs)
                    phases.append(current)
            if topic=='/stage3/match':matches.append(msg)
            elif topic=='/stage3/status':statuses.append(yaml.safe_load(msg.data))
            elif topic=='/fused_submap' and msg.width*msg.height:fused_points.append(msg.width*msg.height)
            all_obs.observe(topic,msg,time,caller)
            if current:
                for obs in current['observations'].values():obs.observe(topic,msg,time,caller)
    finally:
        for bag in bags:bag.close()
    if current:current['end']=end
    alignments=fixed_alignment(all_obs,contract)
    errors=list(all_obs.errors);errors+=['missing run-wide initial alignment: '+r for r in ('ugv','uav') if r not in alignments]
    if list(directory.glob('*.active')):errors.append('recording has unfinished active bags')
    session=json.loads((directory/'session.json').read_text())
    if session.get('status')=='failed':errors.append('recorder failed: '+str(session.get('error')))
    phase_reports=[]
    for index,phase in enumerate(phases):
        entry={k:phase[k] for k in ('phase','start','end')};entry['robots']={}
        for robot,obs in phase['observations'].items():
            obs.end=phase['end']
            # A holding segment need not last 60 s: that requirement is covered
            # by the dedicated hover phase, rather than each movement marker.
            if phase['phase']!='hover':
                import copy
                obs.contract=copy.deepcopy(contract);obs.contract['acceptance']['min_static_seconds']=0
            scenario=PHASES[phase['phase']][robot]
            report=make_report(obs,robot,scenario,'sim',map_review,alignments)
            write_report(report,directory/'phases'/('%02d_%s_%s'%(index,phase['phase'],robot)))
            entry['robots'][robot]=report
        entry['numeric_pass']=all(v['numeric_pass'] for v in entry['robots'].values());phase_reports.append(entry)
    for name in ('hover','ugv_motion','uav_motion','height'):
        found=[p for p in phase_reports if p['phase']==name]
        if not found:errors.append('manual phase missing: '+name)
        elif not found[-1]['numeric_pass']:errors.append('manual phase failed: '+name)
    truth_checks=[]
    for match in matches:
        try:
            a=interpolate_truth(all_obs.truth['ugv'],match.reference_stamp.to_sec())
            b=interpolate_truth(all_obs.truth['uav'],match.header.stamp.to_sec())
            truth=np.linalg.inv(b)@a
            from types import SimpleNamespace
            estimate=pose_matrix(SimpleNamespace(position=match.refined_transform.translation,orientation=match.refined_transform.rotation))
            position=float(np.linalg.norm(estimate[:3,3]-truth[:3,3]));rotation=rotation_error_degrees(estimate,truth)
            result_path=Path(match.result_path);meta=yaml.safe_load(result_path.read_text());points=check_pcd(result_path.parent/'fused_submap.pcd')
            if points!=meta['fusion']['point_count']:raise ValueError('fusion metadata point count mismatch')
            truth_checks.append({'position_error_m':position,'rotation_error_deg':rotation,'passed':position<=.5 and rotation<=5,
                'reference_stamp':match.reference_stamp.to_sec(),'query_stamp':match.header.stamp.to_sec(),
                'result_path':str(result_path),'fused_points':points})
        except (ValueError,KeyError,OSError) as exc:truth_checks.append({'passed':False,'error':str(exc)})
    if not any(t['passed'] for t in truth_checks):errors.append('no saved match passing independent 0.50 m / 5 degree truth check')
    if any(not t['passed'] for t in truth_checks):errors.append('at least one accepted match failed independent verification')
    if not fused_points:errors.append('no nonempty /fused_submap message recorded')
    sensors={}
    for r in ('ugv','uav'):
        sensors[r]={}
        for kind in ('imu','raw_cloud','odometry','registered_raw'):
            stamps=all_obs.stamps[contract['robots'][r][kind]];delta=np.diff(stamps);positive=delta[delta>0]
            sensors[r][kind]={'messages':len(stamps),'rate_hz':(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else 0,
                'median_dt':float(np.median(positive)) if len(positive) else None,'max_dt':float(max(delta)) if len(delta) else None,
                'nonpositive_dt':int(np.sum(delta<=0))}
        acc=np.asarray(all_obs.imu_acc[r])
        if len(acc):sensors[r]['imu_acceleration']={'mean':acc.mean(0).tolist(),'std':acc.std(0).tolist(),'max_norm':float(np.linalg.norm(acc,axis=1).max())}
    screenshot_paths=[str(Path(p).resolve()) for p in screenshots if Path(p).is_file()]
    if map_review=='fail':errors.append('manual public-structure/map review failed')
    numeric_pass=not errors
    report={'status':'failed' if errors else 'passed' if map_review=='pass' and screenshot_paths else 'numeric_pass_visual_review_pending',
        'numeric_pass':numeric_pass,'errors':sorted(set(errors)),'map_review':map_review,'screenshots':screenshot_paths,
        'alignment_method':'first associated lidar pose once per robot for the entire run; no phase refitting',
        'alignments':{r:t.tolist() for r,t in alignments.items()},'phases':phase_reports,'match_truth_checks':truth_checks,
        'sensors':sensors,'last_matcher_status':statuses[-1] if statuses else None,
        'fused_nonempty_messages':len(fused_points),'tf':{c:{'parents':sorted(all_obs.tf_parents[c]),'publishers':sorted(all_obs.tf_callers[c])} for c in all_obs.tf_parents}}
    (directory/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    (directory/'tf_snapshot.json').write_text(json.dumps(report['tf'],indent=2)+'\n')
    lines=['# 双机稳定性与融合验收','', 'Status: **'+report['status']+'**','',
           '整次运行只做一次起始刚体对齐；真值不进入算法。','',
           '阶段：'+', '.join(p['phase']+(' 通过' if p['numeric_pass'] else ' 失败') for p in phase_reports),'',
           '独立真值检查通过匹配数：'+str(sum(t['passed'] for t in truth_checks)),'',
           '## 待处理项','']+['- '+e for e in report['errors']]
    if not report['errors']:lines.append('- 数值检查通过；完整验收还需人工地图检查及截图。' if report['status']!='passed' else '- 全部检查通过。')
    (directory/'report.md').write_text('\n'.join(lines)+'\n')
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    p.add_argument('--map-review',choices=['pending','pass','fail'],default='pending');p.add_argument('--screenshot',action='append',default=[])
    a=p.parse_args();result=evaluate(a.directory,a.map_review,a.screenshot)
    print(result['status']+' -> '+str(a.directory/'report.md'))
    for error in result['errors']:print('FAIL: '+error)
    return 0 if result['numeric_pass'] else 1
if __name__=='__main__':raise SystemExit(main())
