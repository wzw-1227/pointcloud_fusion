"""Diagnostic Gazebo frontend. Truth replaces odometry, never registration hints."""
import bisect
import numpy as np
from .geometry import interpolate_pose, inverse_pose


class TruthBuffer:
    def __init__(self, max_interval=.05, capacity=500):
        self.max_interval = max_interval
        self.capacity = capacity
        self.times, self.poses = [], []

    def add(self, stamp, pose):
        if not np.isfinite(stamp) or not np.all(np.isfinite(pose)):
            raise ValueError('nonfinite truth sample')
        if self.times and stamp <= self.times[-1]:
            return False
        self.times.append(stamp); self.poses.append(pose.copy())
        if len(self.times) > self.capacity:
            del self.times[0]; del self.poses[0]
        return True

    def at(self, stamp):
        index = bisect.bisect_left(self.times, stamp)
        if index < len(self.times) and abs(self.times[index]-stamp) < 1e-9:
            return self.poses[index].copy()
        if index == 0 or index == len(self.times):
            raise ValueError('truth does not bracket scan timestamp')
        a, b = self.times[index-1:index+1]
        if b-a > self.max_interval:
            raise ValueError('truth interpolation gap exceeds limit')
        return interpolate_pose(self.poses[index-1], self.poses[index], (stamp-a)/(b-a))


def initial_reference(world_lidar):
    """Fix one translated, yaw-only reference per run; Z stays gravity-aligned."""
    yaw = np.arctan2(world_lidar[1, 0], world_lidar[0, 0])
    c, s = np.cos(yaw), np.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    result[:3, 3] = world_lidar[:3, 3]
    return result


def register_scan(points, world_lidar, world_reference, min_range, max_range):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('expected Nx3 laser points')
    distances = np.linalg.norm(points, axis=1)
    valid = np.all(np.isfinite(points), axis=1) & (distances >= min_range) & (distances <= max_range)
    pose = inverse_pose(world_reference) @ world_lidar
    return points[valid] @ pose[:3, :3].T + pose[:3, 3], pose, int(np.count_nonzero(~valid))
