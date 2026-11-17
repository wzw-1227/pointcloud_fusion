#!/usr/bin/env python3
"""Deterministic four-region geometry and delayed cross-robot query fixture."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml

COMMIT = 'b4b7137f7e675974e025fc3c8e0ab891d353739a'


def pose(x=0, y=0, z=0, yaw=0):
    out = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    out[:2, :2] = [[c, -s], [s, c]]
    out[:3, 3] = [x, y, z]
    return out


def geometry(center, corridor=False, style=0, straight=False):
    if corridor:
        if straight:
            segments = [((-5, -2), (5, -2)), ((-5, 2), (5, 2))]
        else:
            segments = [((-5,-2),(-1,-2)),((-1,-2),(-1,-3)),((-1,-3),(1,-3)),
                ((1,-3),(1,-2)),((1,-2),(5,-2)),((-5,2),(-2,2)),((-2,2),(-2,4)),
                ((-2,4),(0,4)),((0,4),(0,2)),((0,2),(5,2)),((5,2),(5,-2)),((-5,-2),(-5,2))]
        extent = 7
    else:
        # A room's observed patch has one facade; it cannot independently locate
        # the robot along that facade. Unique room cells test full-map retention.
        angle = style * .31
        r = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        segments = [(r @ np.array([-3,3]), r @ np.array([3,3]))]
        extent = 4
    points = []
    for x in np.arange(-extent, extent+.01, .2):
        for y in np.arange(-extent, extent+.01, .2):
            points.append([x+center[0], y+center[1], 0])
    for a, b in segments:
        a, b = np.asarray(a), np.asarray(b)
        for t in np.linspace(0, 1, max(2, round(np.linalg.norm(b-a)/.15))):
            p = a+t*(b-a)
            for z in np.arange(.1, 2.51, .15):
                points.append([p[0]+center[0], p[1]+center[1], z])
    return np.asarray(points, dtype=float)


def pcd(path, xyz):
    xyz = np.asarray(xyz, dtype='<f4')
    with path.open('wb') as out:
        out.write(('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n').encode())
        out.write(xyz.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--swap-order', action='store_true')
    ap.add_argument('--extra-history', type=int, default=0)
    ap.add_argument('--straight', action='store_true')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'recent').mkdir()
    world = {'ugv': pose(8, 8, .35), 'uav': pose(-8, 8, .4, .25)}
    front = dict(mode='synthetic_geometry_for_independent_scoring',
        matcher_hints='none; world matrices are not supplied to matching',
        robots={k: dict(T_world_reference=v.tolist()) for k,v in world.items()})
    (args.output/'frontend.json').write_text(json.dumps(front, indent=2))
    ids = [0, 0]
    regions = []

    def emit(robot, name, center, stamp, corridor=False, style=0):
        label = 'ugv' if robot == 0 else 'uav'
        lidar = pose(center[0], center[1], .9 if corridor or ids[robot] else (.35 if robot == 0 else .4))
        own = np.linalg.inv(world[label]) @ lidar
        points = geometry(center, corridor, style, args.straight)
        local = (points-lidar[:3,3]) @ lidar[:3,:3]
        stem = label+f'_submap_{ids[robot]:04d}'
        pcd(args.output/(stem+'.pcd'), local)
        pcd(args.output/'recent'/(stem+'.pcd'), local)
        meta = dict(robot_id=robot,session_id='four_rooms',epoch=0,submap_id=ids[robot],
            reference_frame='algorithm_'+label+'_odom',local_frame=stem,
            keyframe_timestamp=float(stamp),segment_end_timestamp=float(stamp),partial=True,
            point_count=len(local),T_reference_key=own.tolist(),T_gravity_key=np.eye(4).tolist(),
            upstream_commit=COMMIT,observation_quality_valid=True,
            recent_matching_cloud=dict(pcd='recent/'+stem+'.pcd',point_count=len(local),
                start_stamp=float(stamp-5),end_stamp=float(stamp),frames=50,center_local=[0,0,0]))
        (args.output/(stem+'.yaml')).write_text(yaml.safe_dump(meta, sort_keys=False))
        if name in 'abcd':
            regions.append(dict(name=name,robot=label,world_center=list(center),identity=f'{robot}:four_rooms:0:{ids[robot]}'))
        ids[robot] += 1

    order = [1, 0] if args.swap_order else [0, 1]
    stamp = 10
    for robot in order:
        emit(robot, 'a' if robot == 0 else 'b', (8,8) if robot == 0 else (-8,8), stamp, style=robot+1);stamp+=10
        emit(robot, 'corridor', (0,0), stamp, corridor=True);stamp+=10
        if robot == order[0]:
            for j in range(args.extra_history):
                emit(robot, 'unrelated', (150+3*j, 80), stamp, style=5);stamp+=10
        emit(robot, 'c' if robot == 0 else 'd', (-100,-100) if robot == 0 else (100,-100), stamp, style=robot+3);stamp+=10
    (args.output/'fixture.json').write_text(json.dumps(dict(regions=regions,
        deferred_search='prime both completed trajectories before any cross-robot query',
        old_regions_evicted_from_input_clouds=True,extra_history=args.extra_history),indent=2))


if __name__ == '__main__':
    main()
