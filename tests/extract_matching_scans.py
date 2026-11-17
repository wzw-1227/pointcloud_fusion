#!/usr/bin/env python3
"""Attach recorded single scans in each robot's own map; never read truth poses."""
import argparse
from pathlib import Path
import shutil
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--bags', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    shutil.copytree(args.inputs, args.output)
    targets = {}
    for p in args.output.glob('*.yaml'):
        y = yaml.load(p.read_text(), Loader=yaml.CSafeLoader)
        recent = y.get('recent_matching_cloud')
        if recent:
            topic = '/algorithm_test/' + ('ugv' if y['robot_id'] == 0 else 'uav') + '/cloud_registered'
            targets.setdefault((topic, round(recent['end_stamp'] * 1e9)), []).append((p, y))
    import rosbag
    found = set()
    topics = sorted({key[0] for key in targets})
    for bag_path in sorted(args.bags.glob('sensors_*.bag'), key=lambda p: int(p.stem.split('_')[-1])):
        with rosbag.Bag(str(bag_path)) as bag:
            lo, hi = bag.get_start_time(), bag.get_end_time()
            if not any(lo - 1 <= key[1] / 1e9 <= hi + 1 and key not in found for key in targets):
                continue
            for topic, message, _ in bag.read_messages(topics=topics):
                key = topic, message.header.stamp.to_nsec()
                if key not in targets or key in found:
                    continue
                fields = {f.name: f.offset for f in message.fields}
                order = '>' if message.is_bigendian else '<'
                dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=[order+'f4']*3,
                    offsets=[fields[x] for x in ('x', 'y', 'z')], itemsize=message.point_step))
                packed = np.frombuffer(message.data, dtype=dtype)
                xyz = np.column_stack([packed[x] for x in ('x', 'y', 'z')]).astype(float)
                xyz = xyz[np.isfinite(xyz).all(axis=1)]
                for path, meta in targets[key]:
                    if message.header.frame_id != meta['reference_frame']:
                        raise ValueError('Recorded scan is in a different own-map frame')
                    pose = np.asarray(meta['T_reference_key'])
                    local = ((xyz - pose[:3, 3]) @ pose[:3, :3]).astype('<f4')
                    relative = 'recent/' + path.stem + '_scan.pcd'
                    with (args.output / relative).open('wb') as out:
                        out.write(('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                            f'WIDTH {len(local)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(local)}\nDATA binary\n').encode())
                        out.write(local.tobytes())
                    meta['recent_matching_cloud']['latest_scan_pcd'] = relative
                    path.write_text(yaml.safe_dump(meta, sort_keys=False))
                found.add(key)
        print(bag_path.name, 'attached', len(found), '/', len(targets), flush=True)
    report = dict(requested=len(targets), attached=len(found),
                  truth_role='none; only own-map registered clouds and input keyframe poses')
    (args.output.parent / 'scan_extraction.yaml').write_text(yaml.safe_dump(report))
    if len(found) != len(targets):
        raise RuntimeError('Some exact-stamp recorded single scans were not found')


if __name__ == '__main__':
    main()
