#!/usr/bin/env python3
"""Check a real accepted record and ensure damaged evidence is rejected."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import tempfile
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from alignment_verify import verify_confirmation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--match',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    config=yaml.safe_load(args.config.read_text());original=yaml.safe_load(args.match.read_text())
    verify_confirmation(args.match,original,config)
    results=dict(original_recomputes=True)
    with tempfile.TemporaryDirectory(prefix='stage3_evidence_') as temporary:
        target=Path(temporary)/'match';shutil.copytree(args.match.parent,target)
        path=target/'match.yaml'
        damaged=yaml.safe_load(args.match.read_text())
        damaged['visibility_validation']['ugv_in_uav']['checked']+=100
        try:verify_confirmation(path,damaged,config)
        except (ValueError,KeyError,FileNotFoundError):results['fabricated_ray_count_rejected']=True
        else:results['fabricated_ray_count_rejected']=False
        damaged=yaml.safe_load(args.match.read_text());supports=damaged['alignment_validation']['supports']
        damaged['alignment_validation']['supports']=[supports[0]]*len(supports)
        try:verify_confirmation(path,damaged,config)
        except (ValueError,KeyError,FileNotFoundError):results['repeated_observation_votes_rejected']=True
        else:results['repeated_observation_votes_rejected']=False
        damaged=yaml.safe_load(args.match.read_text());damaged['T_query_reference_refined'][0][3]+=1
        try:verify_confirmation(path,damaged,config)
        except (ValueError,KeyError,FileNotFoundError):results['wrong_transform_rejected']=True
        else:results['wrong_transform_rejected']=False
        damaged=yaml.safe_load(args.match.read_text());current=target/Path(damaged['alignment_validation']['current_observation']).parent/'ugv.yaml'
        meta=yaml.safe_load(current.read_text());(current.parent/meta['recent_matching_cloud']['latest_scan_pcd']).unlink()
        try:verify_confirmation(path,damaged,config)
        except (ValueError,KeyError,FileNotFoundError):results['missing_single_scan_rejected']=True
        else:results['missing_single_scan_rejected']=False
    results['passed']=all(results.values());args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results,indent=2))
    return 0 if results['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
