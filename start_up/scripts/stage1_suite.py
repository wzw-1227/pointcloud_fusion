#!/usr/bin/env python3
"""Never label smoke records or an incomplete manual test matrix as accepted."""
import argparse
import json
from pathlib import Path

REQUIRED=[('ugv','static'),('ugv','motion'),('uav','static'),('uav','motion'),('uav','height'),('both','static'),('both','coexist')]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('results');args=parser.parse_args()
    root=Path(args.results);reports=[]
    for p in root.glob('*/report.json'):
        data=json.loads(p.read_text());data['path']=str(p);reports.append(data)
    passed={(r['robot'],r['scenario']) for r in reports if r['status']=='passed' and r['environment']=='sim'}
    missing=[list(pair) for pair in REQUIRED if pair not in passed]
    result={'status':'stage1_sim_accepted' if not missing else 'manual_acceptance_pending',
            'missing_passed_scenarios':missing,'records':[{'robot':r['robot'],'scenario':r['scenario'],'status':r['status'],'path':r['path']} for r in reports],
            'hardware_status':'preparation_only; device-specific calibration and validation are pending'}
    release=root/'development_release.json'
    if release.is_file():
        result['development_release']=json.loads(release.read_text())
    (root/'suite.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False));return 0 if not missing else 1

if __name__=='__main__':raise SystemExit(main())
