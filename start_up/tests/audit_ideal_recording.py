#!/usr/bin/env python3
"""Audit a stopped ideal-frontend recording without providing hints to matching."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import rosbag
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'ehpr_bridge/scripts'))
from stage3_verify_results import verify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    frontend = json.loads(next(args.run.glob('frontend/*/frontend.json')).read_text())
    summary = yaml.safe_load(next(args.run.glob('submaps/*/summary.yaml')).read_text())
    folder = next(args.run.glob('matches/*'))
    verify(folder)
    world = {r: np.asarray(frontend['robots'][r]['T_world_reference']) for r in ('ugv','uav')}
    report = {'scope': 'short static diagnostic; not 300s acceptance, LIO or manual rendezvous acceptance',
              'source_run': str(args.run), 'truth_source': frontend['truth_source'],
              'frontend_final': frontend['counters'], 'matches': [],
              'submap_epochs': {r: entry['epoch'] for r, entry in summary['robots'].items()}}
    for path in sorted(folder.glob('match_*/match.yaml')):
        m = yaml.safe_load(path.read_text())
        expected = np.linalg.inv(world['uav'] @ np.asarray(m['query']['T_reference_key'])) @ world['ugv'] @ np.asarray(m['reference']['T_reference_key'])
        actual = np.asarray(m['T_query_reference_refined'])
        position = float(np.linalg.norm(actual[:3,3]-expected[:3,3]))
        angle = float(np.degrees(Rotation.from_matrix(actual[:3,:3].T @ expected[:3,:3]).magnitude()))
        report['matches'].append({'path': str(path), 'position_error_m': position,
                                 'rotation_error_deg': angle, 'passed': position<=.5 and angle<=5})
    stamps = {r: [] for r in ('ugv','uav')}; statuses = []
    topics = ['/algorithm_test/'+r+'/odometry' for r in stamps]+['/algorithm_test/status']
    parts = sorted(args.run.glob('frontend/*/sensors_*.bag'), key=lambda p: int(p.stem.rsplit('_',1)[1]))
    for path in parts:
        with rosbag.Bag(str(path)) as bag:
            for topic, message, _ in bag.read_messages(topics=topics):
                if topic == '/algorithm_test/status':statuses.append(json.loads(message.data))
                else:stamps[topic.split('/')[2]].append(message.header.stamp.to_sec())
    report['output_intervals'] = {}
    for robot, sequence in stamps.items():
        difference = np.diff(sequence)
        report['output_intervals'][robot] = {'samples': len(sequence), 'duration_sim_s': max(sequence)-min(sequence),
            'max_gap_sim_s': float(max(difference)), 'nonincreasing_pairs': int(np.count_nonzero(difference<=0))}
    report['initial_status'] = min(statuses, key=lambda s:s['stamp']) if statuses else None
    valid_statuses = [s for s in statuses if all(s['counters'][r].get('published',0) for r in stamps)]
    baseline = min(valid_statuses, key=lambda s:s['stamp']) if valid_statuses else None
    report['first_valid_output_status'] = baseline
    report['interpolation_drops_during_measurement'] = {
        r: frontend['counters'][r].get('interpolation_drops',0)-
        (baseline or {}).get('counters',{}).get(r,{}).get('interpolation_drops',0) for r in stamps}
    report['passed_short_checks'] = (
        bool(report['matches']) and all(m['passed'] for m in report['matches']) and
        all(epoch==0 for epoch in report['submap_epochs'].values()) and
        all(drops==0 for drops in report['interpolation_drops_during_measurement'].values()) and
        all(v['max_gap_sim_s']<=1 and v['nonincreasing_pairs']==0 for v in report['output_intervals'].values()) and
        all(frontend['counters'][r].get('expired_drops',0)==0 and
            frontend['counters'][r].get('queue_drops',0)==0 and
            frontend['counters'][r]['output_latency_sim_s_p95']<=.5 for r in stamps))
    # Startup interpolation drops remain visible; they are never rewritten as zero.
    report['status'] = 'short_checks_passed' if report['passed_short_checks'] else 'short_checks_failed'
    report['acceptance_complete'] = False
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('status','output_intervals')}, indent=2))
    return 0 if report['passed_short_checks'] else 1


if __name__ == '__main__':raise SystemExit(main())
