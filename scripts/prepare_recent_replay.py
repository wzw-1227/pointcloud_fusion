#!/usr/bin/env python3
"""Attach recent real registered scans to copied submaps; never use cross-robot truth."""
import argparse
from collections import deque
import json
from pathlib import Path
import shutil

import numpy as np
import rosbag
import rospy
from scipy.spatial.transform import Rotation
import yaml


def windowed_bag(path, begin, end, intervals):
    """Read Noetic V2 chunk headers/indexes only for the requested time interval.

    The normal reader seeks through every chunk even for an empty time interval,
    which is prohibitively slow on large split recordings on a busy disk.
    Message decoding still uses rosbag's reader unchanged.
    """
    reader = rosbag.bag._BagReader200
    original = reader.start_reading

    def start(self):
        b = self.bag
        self.read_file_header_record()
        if not b._index_data_pos:
            raise rosbag.bag.ROSBagUnindexedException()
        b._file.seek(b._index_data_pos)
        b._connection_indexes = {}
        for _ in range(b._connection_count):
            connection = self.read_connection_record(b._file, True)
            b._connections[connection.id] = connection
            b._connection_indexes[connection.id] = []
        chunks = [self.read_chunk_info_record() for _ in range(b._chunk_count)]
        b._chunks = [c for c in chunks if any(c.end_time.to_sec() >= a and c.start_time.to_sec() <= z
                                              for a, z in intervals)]
        b._chunk_headers = {}
        for chunk in b._chunks:
            b._file.seek(chunk.pos)
            b._chunk_headers[chunk.pos] = self.read_chunk_header()

    reader.start_reading = start
    try:
        return rosbag.Bag(str(path), skip_index=True)
    finally:
        reader.start_reading = original


def xyz(message):
    fields = {f.name: f for f in message.fields}
    if message.is_bigendian or any(fields[n].datatype != 7 for n in ('x', 'y', 'z')):
        raise ValueError('Expected little-endian FLOAT32 registered XYZ')
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                      'offsets': [fields[n].offset for n in ('x', 'y', 'z')],
                      'itemsize': message.point_step})
    data = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                      strides=(message.row_step, message.point_step))
    points = np.column_stack([data[n].reshape(-1) for n in ('x', 'y', 'z')])
    return points[np.isfinite(points).all(axis=1)]


def voxel(points, leaf):
    keys = np.floor(points.astype(np.float64) / leaf).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_inverse=True)
    count = np.bincount(index)
    return np.column_stack([np.bincount(index, weights=points[:, k]) / count
                            for k in range(3)]).astype(np.float32)


def write_pcd(path, points):
    points = np.asarray(points, dtype='<f4')
    header = ('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n')
    path.write_bytes(header.encode() + points.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--submaps', type=Path, required=True)
    parser.add_argument('--bags', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--latest', type=int, default=0, help='Last N maps per robot; zero keeps all')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    window, leaf = config['recent_window_seconds'], config['recent_voxel_size']
    minimum, capacity = config['recent_min_frames'], config['recent_max_frames']
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'recent').mkdir()
    targets, indices, buffers, pending, records = {}, {}, {}, {}, []
    topics = {}
    for robot in ('ugv', 'uav'):
        paths = sorted(args.submaps.glob(robot + '_submap_*.yaml'))
        if args.latest:
            paths = paths[-args.latest:]
        targets[robot] = []
        generations = set()
        for path in paths:
            metadata = yaml.safe_load(path.read_text())
            generations.add((str(metadata['session_id']), metadata['epoch']))
            if metadata['reference_frame'] != config[robot + '_frame']:
                raise ValueError('Submap frame disagrees with matching configuration')
            targets[robot].append((float(metadata.get('segment_end_timestamp', metadata['keyframe_timestamp'])),
                                   path, metadata))
        if len(generations) > 1:
            raise ValueError('Reconstruct each robot session/epoch separately; reset timestamps can repeat')
        targets[robot].sort(key=lambda item: (item[0], item[2]['submap_id']))
        indices[robot], buffers[robot], pending[robot] = 0, deque(), {}
        topics[config[robot + '_registered_cloud_topic']] = (robot, 'cloud')
        topics[config[robot + '_registered_odometry_topic']] = (robot, 'odom')
    all_targets = [t[0] for group in targets.values() for t in group]
    if not all_targets:
        raise ValueError('No submaps to reconstruct')
    begin, end = min(all_targets) - window - .5, max(all_targets) + .5
    intervals = [(stamp - window - .5, stamp + .5) for stamp in all_targets]

    def emit(robot):
        stop, path, metadata = targets[robot][indices[robot]]
        frames = [f for f in buffers[robot] if stop - window - 1e-6 <= f[0] <= stop + 1e-6]
        frames = frames[-capacity:]
        if len(frames) < minimum:
            raise ValueError(f'{path.name}: only {len(frames)} recent synchronized frames; need {minimum}')
        key = np.asarray(metadata['T_reference_key'])
        joined = voxel(np.vstack([f[1] for f in frames]), leaf)
        local = (joined - key[:3, 3]) @ key[:3, :3]
        relative = Path('recent') / path.with_suffix('.pcd').name
        write_pcd(args.output / relative, local)
        center = (frames[-1][2][:3, 3] - key[:3, 3]) @ key[:3, :3]
        metadata['recent_matching_cloud'] = dict(pcd=str(relative), point_count=len(local),
                                                start_stamp=frames[0][0], end_stamp=frames[-1][0],
                                                frames=len(frames), center_local=center.tolist())
        shutil.copy2(path.with_suffix('.pcd'), args.output / path.with_suffix('.pcd').name)
        (args.output / path.name).write_text(yaml.safe_dump(metadata, sort_keys=False))
        records.append(dict(file=path.name, recent_points=len(local), frames=len(frames),
                            window_start=frames[0][0], window_end=frames[-1][0]))
        indices[robot] += 1

    def receive(robot, stamp, cloud, odom):
        while indices[robot] < len(targets[robot]) and targets[robot][indices[robot]][0] < stamp - 1e-6:
            emit(robot)
        if buffers[robot] and stamp <= buffers[robot][-1][0]:
            return
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        pose = np.eye(4); pose[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        pose[:3, 3] = [p.x, p.y, p.z]
        if buffers[robot]:
            previous = buffers[robot][-1][2]
            if np.linalg.norm(pose[:3, 3] - previous[:3, 3]) > 1 or Rotation.from_matrix(
                    previous[:3, :3].T @ pose[:3, :3]).magnitude() > np.pi / 4:
                buffers[robot].clear()
        buffers[robot].append((stamp, voxel(xyz(cloud), leaf), pose))
        while buffers[robot] and (stamp - buffers[robot][0][0] > window or len(buffers[robot]) > capacity):
            buffers[robot].popleft()
        while indices[robot] < len(targets[robot]) and targets[robot][indices[robot]][0] <= stamp + 1e-6:
            emit(robot)

    bag_paths = sorted(args.bags.glob('*.bag'), key=lambda p: int(p.stem.rsplit('_', 1)[-1]))
    for bag_path in bag_paths:
        with windowed_bag(bag_path, begin, end, intervals) as bag:
            if not bag._chunks:
                continue
            print('READING', bag_path.name, flush=True)
            for topic, message, _ in bag.read_messages(topics=list(topics),
                    start_time=rospy.Time.from_sec(begin), end_time=rospy.Time.from_sec(end)):
                robot, kind = topics[topic]
                if message.header.frame_id != config[robot + '_frame']:
                    raise ValueError('Registered scan/odometry frame disagrees with submaps')
                stamp = message.header.stamp.to_sec()
                item = pending[robot].setdefault(stamp, {})
                item[kind] = message
                if len(item) == 2:
                    receive(robot, stamp, item['cloud'], item['odom']);del pending[robot][stamp]
                for old in [k for k in pending[robot] if k < stamp - 1]:
                    del pending[robot][old]
    for robot in targets:
        while indices[robot] < len(targets[robot]):
            emit(robot)
    (args.output / 'recent_reconstruction.json').write_text(json.dumps({
        'source': 'recorded per-robot registered scans and odometry; no cross-robot truth hint',
        'window_seconds': window, 'voxel_size_m': leaf,
        'numerics': 'NumPy centroid approximation to PCL VoxelGrid; float32 output',
        'records': records}, indent=2) + '\n')
    print('RECENT_SUBMAPS', len(records), 'OUTPUT', args.output, flush=True)


if __name__ == '__main__':
    main()
