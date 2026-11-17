#!/usr/bin/env python3
"""Isolate EHPR/GICP/fusion from LIO with independently known synthetic poses.

Truth creates sensor-local fixtures and is used after registration for scoring.
It is never supplied as a retrieval hint or an initial registration transform.
"""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from stage2_verify_exports import read_pcd, COMMIT
from stage3_verify_results import verify, pcd_colors


def pose(angles=(0, 0, 0), position=(0, 0, 0)):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
    matrix[:3, 3] = position
    return matrix


def apply(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def write_input(folder, robot, world, matrix, noise=0):
    local = apply(world, np.linalg.inv(matrix))
    if noise:
        local += np.random.default_rng(803+robot).normal(0, noise, local.shape)
    local = local.astype('<f4')
    name = 'ugv' if robot == 0 else 'uav'
    path = folder/(name+'_submap_000.pcd')
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              'WIDTH '+str(len(local))+'\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS '+str(len(local))+'\nDATA binary\n')
    path.write_bytes(header.encode()+local.tobytes())
    roll, pitch, _ = Rotation.from_matrix(matrix[:3, :3]).as_euler('xyz')
    gravity = np.eye(4)
    gravity[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, 0]).as_matrix()
    metadata = dict(robot_id=robot, session_id=folder.name, epoch=0, submap_id=0,
                    keyframe_timestamp=100+robot, reference_frame='odom' if robot == 0 else 'uav_lio_odom',
                    local_frame=name+'_key_0', partial=True, point_count=len(local),
                    T_reference_key=matrix.tolist(), T_gravity_key=gravity.tolist(), upstream_commit=COMMIT)
    path.with_suffix('.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    return local.astype(float)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--binary', type=Path, default=WORKSPACE/'devel/lib/ehpr_bridge/stage3_match_cli')
    parser.add_argument('--test-binary', type=Path, default=WORKSPACE/'devel/lib/ehpr_bridge/test_matcher')
    parser.add_argument('--config', type=Path, default=PACKAGE/'config/stage3.yaml',
                        help='Candidate config override; never changes production YAML')
    args = parser.parse_args()
    timestamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    output = (args.output or WORKSPACE/'results/stage3'/(timestamp+'_algorithm_validation')).resolve()
    output.mkdir(parents=True, exist_ok=False)
    binary = args.binary.resolve()
    test_binary = args.test_binary.resolve()
    config = args.config.resolve()
    fixture = output/'base_fixture';fixture.mkdir()
    import os
    env = os.environ.copy();env['STAGE3_FIXTURE_DIR'] = str(fixture)
    with (output/'fixture.log').open('w') as log:
        subprocess.run([str(test_binary), '--gtest_filter=Matching.KnownPoseGravityAndDisplayDirection'],
                       env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=60)
    world = read_pcd(fixture/'ugv_submap_000.pcd')
    identity = pose()
    six = pose((-2.864788976, 4.583662361, 11.45915590), (1.2, -2.1, 1.4))
    tilted_ref = pose((5, -4, 17), (4, 1, .6))
    tilted_query = pose((-4, 6, -13), (2, -2, 1.8))
    cases = [
        ('identity_full', world, world, identity, identity, 0, True),
        ('six_dof_full', world, world, identity, six, 0, True),
        ('both_keys_tilted', world, world, tilted_ref, tilted_query, 0, True),
        ('complementary', world[world[:, 0]<16], world[world[:, 0]>-16], identity, pose(position=(4,-4,0)), 0, True),
        ('complementary_noisy_six_dof', world[world[:, 0]<16], world[world[:, 0]>-16], identity,
         pose((-3,4,14), (2,-2,1.2)), .008, True),
        ('disjoint_regions', world[world[:, 0]<-8], world[world[:, 0]>8], identity, six, 0, False),
        ('flat_height', world, np.column_stack((world[:, :2], np.zeros(len(world)))), identity, identity, 0, False),
        ('mirror_geometry', world, world*np.array([-1.,1.,1.]), identity, identity, 0, False),
    ]
    report = dict(status='running', scope='algorithm-only synthetic regression; no LIO, Gazebo or flight acceptance',
                  configuration=str(config), parameters=yaml.safe_load(config.read_text()),
                  thresholds='actual config recorded; production stage3.yaml is never changed by this script',
                  truth_usage='fixture generation and independent scoring only; not an algorithm hint',
                  source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                                 (binary, config, PACKAGE/'src/matcher.cpp', PACKAGE/'src/submap_fusion.cpp')}, cases=[])
    for name, ref_world, query_world, ref_pose, query_pose, noise, positive in cases:
        folder = output/name;folder.mkdir();inputs = folder/'inputs';inputs.mkdir()
        ref = write_input(inputs, 0, ref_world, ref_pose, noise)
        query = write_input(inputs, 1, query_world, query_pose, noise)
        expected = np.linalg.inv(query_pose) @ ref_pose
        (folder/'evaluation_truth.json').write_text(json.dumps({'T_UAVkey_UGVkey':expected.tolist(),
            'positive':positive, 'noise_std_m':noise, 'role':'evaluation only; not read by CLI'}, indent=2)+'\n')
        with (folder/'cli.log').open('w') as log:
            completed = subprocess.run([str(binary),str(inputs),str(config),str(folder/'matches')],
                                       stdout=log, stderr=subprocess.STDOUT, timeout=90)
        runs = list((folder/'matches').iterdir())
        if len(runs)!=1:raise RuntimeError('missing unique CLI run')
        run = runs[0];summary = yaml.safe_load((run/'summary.yaml').read_text())
        paths = list(run.glob('match_*/match.yaml'))
        item = dict(name=name, expected_match=positive, returncode=completed.returncode, accepted_matches=len(paths),
                    input_points=[len(ref),len(query)], noise_std_m=noise,
                    event_reasons=[event['reason'] for event in summary['events'] if event['reason']])
        item['passed'] = completed.returncode in (0,2) and (len(paths)==1 if positive else len(paths)==0)
        if not paths and summary['events'][-1].get('attempted_match'):
            attempt=summary['events'][-1]['attempted_match']
            actual=np.asarray(attempt['T_query_reference_refined'])
            item['rejected_attempt']=dict(position_error_m=float(np.linalg.norm(actual[:3,3]-expected[:3,3])),
                rotation_error_deg=float(np.degrees(Rotation.from_matrix(actual[:3,:3].T @ expected[:3,:3]).magnitude())),
                residual_rmse_m=attempt['residual_rmse_m'], overlap=attempt['overlap_smaller_cloud'])
        if paths:
            metadata = yaml.safe_load(paths[0].read_text())
            actual = np.asarray(metadata['T_query_reference_refined'])
            item['position_error_m'] = float(np.linalg.norm(actual[:3,3]-expected[:3,3]))
            item['rotation_error_deg'] = float(np.degrees(Rotation.from_matrix(actual[:3,:3].T @ expected[:3,:3]).magnitude()))
            item['ransac_inliers'] = metadata['ransac_inliers']
            item['residual_rmse_m'] = metadata['residual_rmse_m']
            item['overlap'] = metadata['overlap_smaller_cloud']
            try:
                verified = verify(run)
                item['independent_fusion_checks'] = verified['matches'][0]['fusion']
            except Exception as exc:
                item['passed'] = False;item['file_error'] = str(exc)
            item['passed'] &= positive and item['position_error_m']<=.20 and item['rotation_error_deg']<=3
            # Check common geometry against truth after registration, rather
            # than relying only on the matcher's own nearest-neighbor fitness.
            ref_display = apply(ref, ref_pose)
            query_display = apply(query, ref_pose @ np.linalg.inv(actual))
            item['query_world_truth_rmse_m'] = float(np.sqrt(np.mean(np.sum((query_display-apply(query, query_pose))**2,axis=1))))
            if name in ('complementary', 'complementary_noisy_six_dof'):
                fused = read_pcd(paths[0].parent/'fused_submap.pcd')
                colors=pcd_colors(paths[0].parent/'fused_submap.pcd')
                rgb=np.column_stack(((colors>>16)&255,(colors>>8)&255,colors&255))/255
                item['coverage_bounds_x']={'ugv_max':float(ref_display[:,0].max()),'fused_max':float(fused[:,0].max())}
                item['passed'] &= item['coverage_bounds_x']['fused_max']>item['coverage_bounds_x']['ugv_max']+1
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt
                fig, axes = plt.subplots(1,3,figsize=(15,5),sharex=True,sharey=True)
                for ax, cloud, title, color in zip(axes,[ref_display,query_display,fused],
                        ['UGV coverage','UAV aligned coverage','Fused pair'],['green','purple',rgb[::3]]):
                    ax.scatter(cloud[::3,0],cloud[::3,1],s=1,c=color);ax.set_title(title);ax.set_aspect('equal');ax.grid(alpha=.2)
                    ax.set_xlabel('X (m)')
                axes[0].set_ylabel('Y (m)')
                fig.suptitle('Synthetic algorithm test (not flight data)\n'+name,fontsize=12)
                fig.tight_layout(rect=(0,0,1,.90));fig.savefig(output/(name+'_fusion.png'),dpi=160);plt.close(fig)
        report['cases'].append(item)
        print(json.dumps(item),flush=True)
        (output/'checks.json').write_text(json.dumps(report,indent=2)+'\n')
    report['status'] = 'passed' if all(c['passed'] for c in report['cases']) else 'failed'
    report['passed_cases'] = sum(c['passed'] for c in report['cases'])
    (output/'checks.json').write_text(json.dumps(report,indent=2)+'\n')
    lines=['# EHPR/GICP/融合独立验证','','状态：**'+report['status']+'**。输入为合成点云，绕过 LIO 和 Gazebo。',
           '配置：`'+str(config)+'`（完整参数见 checks.json）；正式配置不被修改。',
           '已知位姿只用于生成局部点云和事后评估，不传给算法。','',
           '| 场景 | 接受匹配 | 位置误差 m | 姿态误差 ° | 检查 |','|---|---:|---:|---:|---|']
    for c in report['cases']:
        lines.append('| '+c['name']+' | '+str(c['accepted_matches'])+' | '+
            (f"{c['position_error_m']:.4f}" if 'position_error_m' in c else '—')+' | '+
            (f"{c['rotation_error_deg']:.4f}" if 'rotation_error_deg' in c else '—')+' | '+str(c['passed'])+' |')
    lines += ['', '融合 PCD 由独立 NumPy 实现核对坐标变换、体素联合质心和双方来源颜色。',
              '本结果只覆盖上述输入范围，不能证明所有场景均能匹配或漂移点云仍可正确融合。',
              '完整历史地图优化、飞行稳定性和跨时段鲁棒性不在本次验收范围。']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print('CHECKS_DIR='+str(output),flush=True)
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
