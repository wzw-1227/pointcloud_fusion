#!/usr/bin/env python3
"""Read binary PCD and its coordinate metadata; never infer manual acceptance."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml

COMMIT = 'b4b7137f7e675974e025fc3c8e0ab891d353739a'


def read_pcd(path):
    header = {}
    with Path(path).open('rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('missing PCD DATA line')
            text = line.decode('ascii').strip()
            if not text or text.startswith('#'):
                continue
            key, *values = text.split()
            header[key] = values
            if key == 'DATA':
                break
        if header['DATA'] != ['binary']:
            raise ValueError('expected uncompressed binary PCD')
        fields = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        types = header['TYPE']
        codes = {'F': 'f', 'I': 'i', 'U': 'u'}
        dtype = np.dtype([(name, '<' + codes[kind] + str(size), (count,)) if count > 1
                          else (name, '<' + codes[kind] + str(size))
                          for name, size, kind, count in zip(fields, sizes, types, counts)])
        n = int(header['POINTS'][0])
        payload = stream.read()
    if len(payload) != n * dtype.itemsize:
        raise ValueError('PCD payload size does not match POINTS')
    data = np.frombuffer(payload, dtype=dtype)
    xyz = np.column_stack([data[k] for k in ('x', 'y', 'z')]).astype(float)
    if not len(xyz) or not np.all(np.isfinite(xyz)):
        raise ValueError('PCD contains no points or nonfinite XYZ')
    return xyz


def validate_transform(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError('transform must be a finite 4x4 matrix')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError('invalid homogeneous transform last row')
    r = matrix[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1, atol=1e-6):
        raise ValueError('transform rotation is not proper')
    return matrix


def verify(directory, selection='both'):
    directory = Path(directory)
    errors, records = [], []
    names = ['ugv', 'uav'] if selection == 'both' else [selection]
    summary = yaml.safe_load((directory / 'summary.yaml').read_text())
    if summary.get('status') == 'failed':
        errors.append('builder failed: ' + summary.get('error', 'unknown'))
    parameters = yaml.safe_load((directory / 'parameters.yaml').read_text())
    expected = parameters['builder_config']['robots']
    seen = set()
    for name in names:
        paths = sorted(directory.glob(name + '_submap_*.pcd'))
        if not paths:
            errors.append(name + ': no PCD exports')
        for path in paths:
            try:
                meta = yaml.safe_load(path.with_suffix('.yaml').read_text())
                xyz = read_pcd(path)
                transform = validate_transform(meta['T_reference_key'])
                gravity = validate_transform(meta['T_gravity_key'])
                if meta['robot'] != name or meta['robot_id'] != (0 if name == 'ugv' else 1):
                    raise ValueError('robot identity mismatch')
                if meta['reference_frame'] != expected[name]['reference_frame']:
                    raise ValueError('reference frame mismatch')
                if meta['upstream_commit'] != COMMIT:
                    raise ValueError('unexpected upstream version')
                if meta['point_count'] != len(xyz):
                    raise ValueError('point count mismatch')
                if meta['session_id'] != summary['session_id'] or meta['pcd'] != path.name:
                    raise ValueError('run/file identity mismatch')
                identity = (meta['robot_id'], meta['submap_id'])
                if identity in seen:
                    raise ValueError('duplicate submap ID within robot')
                seen.add(identity)
                if not np.allclose(gravity[:3, 3], 0):
                    raise ValueError('gravity alignment must have no translation')
                if not meta['partial'] and meta['segment_travel_distance_m'] <= parameters['builder_config']['voxel_map']['max_distance']:
                    raise ValueError('automatic submap did not exceed distance threshold')
                reference = xyz @ transform[:3, :3].T + transform[:3, 3]
                records.append({'pcd': path.name, 'robot': name, 'epoch': meta['epoch'],
                                'submap_id': meta['submap_id'], 'points': len(xyz), 'partial': meta['partial'],
                                'reason': meta['reason'], 'travel_distance_m': meta['segment_travel_distance_m'],
                                'reference_bounds_min': reference.min(0).tolist(),
                                'reference_bounds_max': reference.max(0).tolist()})
            except (ValueError, KeyError, TypeError, OSError) as exc:
                errors.append(path.name + ': ' + str(exc))
    report = {'status': 'file_checks_passed' if not errors else 'failed', 'robot': selection,
              'upstream_commit': COMMIT, 'errors': errors, 'exports': records,
              'manual_visual_review': 'pending',
              'limitations': ['Partial exports do not prove a completed 20 m route',
                              'Independent submaps and viz_world do not prove cross-robot alignment']}
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('--robot', choices=['ugv', 'uav', 'both'], default='both')
    args = parser.parse_args()
    directory = Path(args.directory)
    try:
        report = verify(directory, args.robot)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        report = {'status': 'failed', 'errors': [str(exc)], 'exports': [], 'manual_visual_review': 'pending'}
    (directory / 'verification.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    lines = ['# Stage 2 file verification', '', 'Status: **' + report['status'] + '**', '',
             'Manual visual review: pending', '', '| PCD | Points | Partial | Distance (m) |', '|---|---:|---|---:|']
    for r in report['exports']:
        lines.append(f"| {r['pcd']} | {r['points']} | {r['partial']} | {r['travel_distance_m']:.3f} |")
    lines += ['', '## Failures', ''] + ['- ' + e for e in report['errors']]
    if not report['errors']:
        lines.append('None.')
    (directory / 'verification.md').write_text('\n'.join(lines) + '\n')
    print(report['status'] + ' -> ' + str(directory / 'verification.json'))
    for error in report['errors']:
        print('FAIL:', error)
    return 0 if not report['errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
