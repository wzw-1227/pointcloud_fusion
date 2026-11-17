#!/usr/bin/env python3
"""Score recorded inputs after autonomous matching; truth never enters the CLI."""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from stage3_verify_results import verify


def confirmation_metrics(events):
    attempted=[e for e in events if e.get('candidate_attempts')]
    geometry=[e for e in attempted if any(a.get('reason') in (
        'accepted','geometrically valid alternative',
        'bidirectional single-scan free-space contradiction gate failed')
        and a.get('registration') for a in e['candidate_attempts'])]
    confirmed=[e for e in events if e.get('status')=='matched']
    def stamp(event):
        path=event.get('input_path')
        if not path:return None
        meta=yaml.load(Path(path).read_text(),Loader=yaml.CSafeLoader)
        return meta.get('recent_matching_cloud',{}).get('end_stamp',meta['keyframe_timestamp'])
    first_geometry=next((stamp(e) for e in geometry if e.get('input_path')),None)
    first_confirmed=next((stamp(e) for e in confirmed if e.get('input_path')),None)
    return dict(candidate_search_events=len(attempted),geometry_only_pass_events=len(geometry),
        geometry_only_pass_rate=len(geometry)/max(1,len(attempted)),
        confirmed_event_rate=len(confirmed)/max(1,len(geometry)),
        first_fusion_delay_after_geometry_sim_s=(first_confirmed-first_geometry)
            if first_geometry is not None and first_confirmed is not None else None,
        delay_definition='input scan time at first confirmation minus first geometry-passing candidate; independent support span is reported separately')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--frontend', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--binary', type=Path,
                        default=WORKSPACE / 'devel/lib/ehpr_bridge/stage3_match_cli')
    parser.add_argument('--negative', action='store_true')
    parser.add_argument('--min-queries', type=int, default=1)
    parser.add_argument('--required-query', type=int)
    parser.add_argument('--min-high-queries', type=int, default=0)
    parser.add_argument('--first-match-high', action='store_true')
    parser.add_argument('--prime', action='store_true')
    parser.add_argument('--ingestion-order', action='store_true')
    parser.add_argument('--existing-run', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    command = [str(args.binary.resolve()), str(args.inputs.resolve()),
               str(args.config.resolve()), str((args.output / 'matches').resolve()), '--chronological']
    if args.prime:command.append('--prime')
    if args.ingestion_order:command.append('--ingestion-order')
    started = time.monotonic()
    if args.existing_run:
        run=args.existing_run
        result=subprocess.CompletedProcess(command, 0)
    else:
        with (args.output / 'cli.log').open('w') as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=1800)
        run = next((args.output / 'matches').iterdir())
    loader = getattr(yaml, 'CSafeLoader', yaml.SafeLoader)
    summary = yaml.load((run / 'summary.yaml').read_text(), Loader=loader)
    events = list(yaml.load_all((run / 'events.yaml').read_text(), Loader=loader))
    frontend = json.loads(args.frontend.read_text())
    world = {name: np.asarray(frontend['robots'][name]['T_world_reference'])
             for name in ('ugv', 'uav')}
    matches = []
    for file in sorted(run.glob('match_*/match.yaml')):
        m = yaml.safe_load(file.read_text())
        expected = np.linalg.inv(world['uav'] @ np.asarray(m['query']['T_reference_key'])) @ (
            world['ugv'] @ np.asarray(m['reference']['T_reference_key']))
        actual = np.asarray(m['T_query_reference_refined'])
        matches.append(dict(query_id=m['query']['submap_id'], reference_id=m['reference']['submap_id'],
                            branch=m.get('registration_branch','legacy_3d'),
                            height_m=m.get('contour_validation',{}).get('query_height_above_ground_m',0),
                            position_error_m=float(np.linalg.norm(actual[:3, 3] - expected[:3, 3])),
                            rotation_error_deg=float(np.degrees(Rotation.from_matrix(
                                actual[:3, :3].T @ expected[:3, :3]).magnitude())),
                            ransac_inliers=m['ransac_inliers'], path=str(file)))
    query_ids = sorted({m['query_id'] for m in matches})
    high_ids=sorted({m['query_id'] for m in matches if m['branch']=='contour_xy_ground' and m['height_m']>=2.5})
    accurate = all(m['position_error_m'] <= .20 and m['rotation_error_deg'] <= 3 for m in matches)
    passed = result.returncode in (0, 2) and (not matches if args.negative else
             len(query_ids) >= args.min_queries and accurate and
             (args.required_query is None or args.required_query in query_ids))
    passed = passed and len(high_ids)>=args.min_high_queries and (not args.first_match_high or
        (bool(matches) and matches[0]['branch']=='contour_xy_ground'))
    # A malformed fixture is not evidence that an algorithm rejects bad geometry.
    invalid = [e for e in events if e.get('reason') in (
        'invalid identity/frame/pose/cloud', 'nonfinite cloud', 'invalid recent cloud metadata',
        'nonfinite recent cloud', 'nonfinite latest scan')]
    passed = passed and not invalid
    times = [e.get('diagnostics', {}).get('matching_processing_wall_s', 0) for e in events
             if e.get('status') not in ('database_updated', 'duplicate')]
    report = dict(passed=bool(passed), run=str(run), inputs=str(args.inputs),
                  truth_role='independent post-registration scoring only; not sent to matcher',
                  returncode=result.returncode, accepted_matches=len(matches), query_ids=query_ids,
                  high_query_ids=high_ids,
                  invalid_input_events=len(invalid),
                  wall_seconds=time.monotonic() - started, matches=matches,
                  reason_counts=dict(Counter(e.get('reason', '') for e in events)),
                  processing_wall_s_p95=float(np.percentile(times, 95)) if times else 0,
                  max_candidates=max((e.get('diagnostics', {}).get('candidate_count', 0)
                                      for e in events), default=0))
    report['geometric_candidate_events'] = sum(e.get('diagnostics', {}).get('geometrically_valid_candidates', 0) > 0 for e in events)
    report.update(confirmation_metrics(events))
    report['confirmation_wait_events'] = sum(e.get('status') == 'waiting_confirmation' for e in events)
    report['visibility_wait_events'] = sum(e.get('status') == 'waiting_visibility_evidence' for e in events)
    report['max_independent_observations'] = max((e.get('diagnostics', {}).get('independent_observations', 0) for e in events), default=0)
    report['confirmation_wait_sim_s'] = [yaml.safe_load(Path(m['path']).read_text()).get('alignment_validation', {}).get('confirmation_wait_sim_s', 0) for m in matches]
    if not matches and report['confirmation_wait_events']:
        report['unconfirmed_reason'] = 'Geometry candidates exist, but the recording lacks enough independent confirmation observations'
    if yaml.load(args.config.read_text(), Loader=loader).get('matching_profile') == 'partial':
        max_per_reference = max((max(Counter(a['reference_identity']
            for a in e.get('candidate_attempts', [])).values(), default=0) for e in events), default=0)
        report['max_hypotheses_per_reference'] = max_per_reference
        max_per_pair = max((max(Counter((a['reference_identity'], a.get('registration', {}).get('query', {}).get('submap_id'))
            for a in e.get('candidate_attempts', [])).values(), default=0) for e in events), default=0)
        report['max_hypotheses_per_pair'] = max_per_pair
        report['passed'] &= report['max_candidates'] <= 30 and max_per_pair <= 3
    if yaml.load(args.config.read_text(), Loader=loader).get('matching_profile') == 'partial_contour':
        max_per_pair=max((max(Counter((a['reference_identity'],a.get('registration',{}).get('query',{}).get('submap_id'))
            for a in e.get('candidate_attempts',[])).values(),default=0) for e in events),default=0)
        report['max_hypotheses_per_pair']=max_per_pair
        report['passed'] &= report['max_candidates']<=40 and max_per_pair<=4
    if matches:
        try:
            report['file_checks'] = verify(run)
        except Exception as exc:
            report['passed'] = False
            report['file_error'] = str(exc)
    (args.output / 'checks.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('matches', 'file_checks')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
