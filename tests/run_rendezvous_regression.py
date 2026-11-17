#!/usr/bin/env python3
"""Replay a recorded ideal-frontend rendezvous, scoring truth only after registration."""
import argparse
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import shutil
import subprocess
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

PACKAGE=Path(__file__).resolve().parents[1]
WORKSPACE=PACKAGE.parents[1]
sys.path.insert(0,str(PACKAGE/'scripts'))
from stage3_verify_results import verify
from stage2_verify_exports import read_pcd


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--submaps',type=Path,required=True)
    parser.add_argument('--frontend',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=WORKSPACE/'src/start_up/config/stage3_gazebo_algorithm_matching.yaml')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--binary',type=Path,default=WORKSPACE/'devel/lib/ehpr_bridge/stage3_match_cli')
    parser.add_argument('--chronological-only',action='store_true',help='All epochs, in export order; do not invent negative pairs from this recording')
    parser.add_argument('--alias-reference-id',type=int,help='Optional known incorrect repeated-structure pair')
    parser.add_argument('--alias-query-id',type=int)
    args=parser.parse_args()
    output=args.output or WORKSPACE/'results/stage3_rendezvous'/datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_regression')
    output.mkdir(parents=True,exist_ok=False)
    frontend=json.loads(args.frontend.read_text());world={r:np.asarray(frontend['robots'][r]['T_world_reference']) for r in ['ugv','uav']}
    config=output/'parameters.yaml';shutil.copy2(args.config,config)
    shutil.copy2(args.frontend,output/'frontend_truth_for_evaluation.json')
    refs=sorted(args.submaps.glob('ugv_submap_*.yaml'));queries=sorted(args.submaps.glob('uav_submap_*.yaml'))
    if not refs or not queries:raise ValueError('Both robots must have persisted input submaps')
    previews=[p for p in refs if yaml.safe_load(p.read_text()).get('reason')=='online_preview']
    cases=[('chronological_rendezvous',refs+queries,True,True),
           ('disjoint_start',refs[:1]+queries[-1:],False,False),
           ('latest_full_history',(previews[-1:] or refs[-1:])+queries[-1:],True,False)]
    if args.chronological_only:cases=cases[:1]
    if args.alias_reference_id is not None and args.alias_query_id is not None:
        cases.append(('repetitive_structure_alias',[
            args.submaps/f'ugv_submap_{args.alias_reference_id:03d}.yaml',
            args.submaps/f'uav_submap_{args.alias_query_id:03d}.yaml'],False,False))
    report={'scope':'actual Gazebo rays, ideal frontend, recorded manual motion; not LIO acceptance',
            'truth_role':'independent scoring only; never CLI initialization or retrieval hint','cases':[]}
    for name,files,positive,chronological in cases:
        folder=output/name;inputs=folder/'inputs';inputs.mkdir(parents=True)
        for file in files:
            for ext in ['.pcd','.yaml']:shutil.copy2(file.with_suffix(ext),inputs/file.with_suffix(ext).name)
            metadata=yaml.safe_load(file.read_text())
            if 'recent_matching_cloud' in metadata:
                relative=Path(metadata['recent_matching_cloud']['pcd'])
                if relative.is_absolute() or '..' in relative.parts:raise ValueError('Invalid recent cloud path')
                destination=inputs/relative;destination.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(file.parent/relative,destination)
        command=[str(args.binary.resolve()),str(inputs),str(config),str(folder/'matches')]
        if chronological:command.append('--chronological')
        with (folder/'cli.log').open('w') as log:
            completed=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=300)
        run=next((folder/'matches').iterdir());summary=yaml.safe_load((run/'summary.yaml').read_text())
        item={'name':name,'chronological':chronological,'accepted_matches':summary['accepted_matches'],
              'returncode':completed.returncode,'counters':summary.get('counters',{}),'matches':[]}
        item['passed']=completed.returncode in (0,2) and (item['accepted_matches']>0 if positive else item['accepted_matches']==0)
        if positive and item['accepted_matches']:
            item['file_checks']=verify(run)
            for file in sorted(run.glob('match_*/match.yaml')):
                m=yaml.safe_load(file.read_text())
                expected=np.linalg.inv(world['uav']@np.asarray(m['query']['T_reference_key']))@world['ugv']@np.asarray(m['reference']['T_reference_key'])
                actual=np.asarray(m['T_query_reference_refined'])
                position=float(np.linalg.norm(actual[:3,3]-expected[:3,3]));rotation=float(np.degrees(Rotation.from_matrix(actual[:3,:3].T@expected[:3,:3]).magnitude()))
                points=read_pcd(file.parent/'reference.pcd');g=np.asarray(m['reference']['T_gravity_key']);gp=points@g[:3,:3].T
                radius=m['matching_scope']['ugv_horizontal_radius_m']
                outside=int(np.count_nonzero(np.linalg.norm(gp[:,:2],axis=1)>radius)) if radius else 0
                item['matches'].append({'reference_id':m['reference']['submap_id'],'query_id':m['query']['submap_id'],
                    'position_error_m':position,'rotation_error_deg':rotation,'residual_rmse_m':m['residual_rmse_m'],
                    'full_reference_points':len(points),'retained_reference_points_outside_matching_radius':outside,
                    'fused_points':m['fusion']['point_count'],'path':str(file)})
                item['passed'] &= position<=.5 and rotation<=5
            if name=='latest_full_history':item['passed'] &= item['matches'][0]['retained_reference_points_outside_matching_radius']>0
        report['cases'].append(item)
        print(json.dumps({k:v for k,v in item.items() if k not in ['file_checks','matches']}),flush=True)
        (output/'checks.json').write_text(json.dumps(report,indent=2)+'\n')
    report['status']='passed' if all(c['passed'] for c in report['cases']) else 'failed'
    (output/'checks.json').write_text(json.dumps(report,indent=2)+'\n')
    print('CHECKS_DIR='+str(output),flush=True)
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
