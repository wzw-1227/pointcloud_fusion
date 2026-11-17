#!/usr/bin/env python3
"""Measure floor support through recorded scan/feature stages, evaluation only.

Uses independently recorded lidar truth only to classify world-ground returns.
Does not start ROS nodes, publish truth, alter LIO, or claim flight acceptance.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import rosbag
import rospy
import yaml
from scipy.spatial.transform import Rotation, Slerp
from stage1_tools.cloud import cloud_array


def xyz(message):
    a = cloud_array(message)
    return a, np.column_stack([a[k] for k in ('x', 'y', 'z')]).astype(float)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--features', type=Path, required=True)
    p.add_argument('--truth-cache', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stamps', type=float, nargs='+', default=[180, 191, 192, 193, 194, 197])
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    truth = np.load(args.truth_cache)['truth']
    rotations = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))
    recording = Path(json.loads((args.run / 'probe.json').read_text())['recording_directory'])
    params = yaml.safe_load((recording/'parameters_start.yaml').read_text())['uav']['lio_sam']
    lines, horizon = params['N_SCAN'], params['Horizon_SCAN']
    range_min, range_max = params['lidarMinRange'], params['lidarMaxRange']
    chosen = []
    with rosbag.Bag(str(args.features)) as bag:
        for stamp in args.stamps:
            candidates = list(bag.read_messages(start_time=rospy.Time.from_sec(stamp-.12),
                                              end_time=rospy.Time.from_sec(stamp+.12)))
            if not candidates:
                raise RuntimeError('no feature frame near '+str(stamp))
            chosen.append(min(candidates, key=lambda row: abs(row[1].header.stamp.to_sec()-stamp))[1])
    raw_frames = {}
    for path in sorted(recording.glob('data*.bag'), key=lambda f: int(f.stem.split('_')[-1])):
        with rosbag.Bag(str(path)) as bag:
            for m in chosen:
                stamp = m.header.stamp.to_sec()
                if stamp < bag.get_start_time()-.15 or stamp > bag.get_end_time()+.15:
                    continue
                for _, raw, _ in bag.read_messages(topics=['/stage1/uav/points'],
                        start_time=rospy.Time.from_sec(stamp-.1), end_time=rospy.Time.from_sec(stamp+.1)):
                    if abs(raw.header.stamp.to_sec()-stamp) < 1e-7:
                        raw_frames[stamp] = raw
                        break
    results = []
    samples = {}
    for m in chosen:
        stamp = m.header.stamp.to_sec()
        if stamp not in raw_frames:
            raise RuntimeError('no exact raw scan for feature stamp '+str(stamp))
        rotation = rotations(stamp)
        pos = np.array([np.interp(stamp, truth[:, 0], truth[:, i]) for i in (1, 2, 3)])
        def describe(points):
            world = rotation.apply(points)+pos
            floor = abs(world[:, 2]) < .03
            return {'points':len(points), 'floor_points':int(floor.sum()),
                    'range_below_0_3m':int((np.linalg.norm(points, axis=1)<.3).sum()),
                    'range_below_0_6m':int((np.linalg.norm(points, axis=1)<.6).sum()),
                    'world_height_quantiles':np.quantile(world[:, 2], [0,.1,.25,.5,.75,.9,1]).tolist()}, floor
        a, raw = xyz(raw_frames[stamp])
        ranges = np.linalg.norm(raw, axis=1)
        valid = (ranges >= range_min) & (ranges <= range_max) & (a['ring'] < lines)
        # Match imageProjection's C++ round (away from zero) and first return.
        angles = (np.degrees(np.arctan2(raw[:, 0], raw[:, 1]))-90)/(360/horizon)
        rounded = np.sign(angles)*np.floor(abs(angles)+.5)
        columns = (-rounded+horizon//2).astype(int)
        columns[columns>=horizon] -= horizon
        valid &= (columns>=0) & (columns<horizon)
        indexes = np.flatnonzero(valid)
        _, first = np.unique(a['ring'][indexes].astype(int)*horizon+columns[indexes], return_index=True)
        projected = raw[indexes[first]]
        stages = {}
        stages['raw'], raw_floor = describe(raw)
        stages['range_filtered'], _ = describe(raw[valid])
        stages['projected_expected'], _ = describe(projected)
        if m.cloud_deskewed.width:
            _, deskewed = xyz(m.cloud_deskewed)
            stages['deskewed_recorded'], _ = describe(deskewed)
        _, surface = xyz(m.cloud_surface)
        _, corner = xyz(m.cloud_corner)
        stages['surface'], _ = describe(surface)
        stages['corner'], _ = describe(corner)
        rings = []
        for ring in np.unique(a['ring']):
            mask = a['ring'] == ring
            rings.append({'ring':int(ring), 'raw_points':int(mask.sum()),
                          'floor_points':int((mask & raw_floor).sum()),
                          'elevation_median_deg':float(np.median(np.degrees(np.arctan2(raw[mask,2], np.linalg.norm(raw[mask,:2],axis=1))))),
                          'median_range_m':float(np.median(ranges[mask]))})
        row = {'stamp':stamp, 'truth_lidar_position':pos.tolist(), 'stages':stages, 'rings':rings}
        results.append(row)
        samples[str(stamp)+'_raw'] = raw
        samples[str(stamp)+'_surface'] = surface
        print(json.dumps({'stamp':stamp,'stages':{k:{f:v[f] for f in ('points','floor_points')} for k,v in stages.items()}}), flush=True)
    report = {'scope':'evaluation-only floor classification using independent lidar truth; no estimator modification',
              'floor_tolerance_m':.03, 'projection_parameters':{'N_SCAN':lines,'Horizon_SCAN':horizon,'range_min':range_min,'range_max':range_max},
              'run':str(args.run), 'features':str(args.features), 'results':results}
    (args.output/'floor_pipeline.json').write_text(json.dumps(report, indent=2)+'\n')
    np.savez_compressed(args.output/'scan_samples.npz', **samples)


if __name__ == '__main__':
    main()
