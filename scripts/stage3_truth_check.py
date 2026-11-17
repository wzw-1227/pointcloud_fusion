#!/usr/bin/env python3
"""Independent evaluator: Gazebo lidar truth never enters the matcher."""
import bisect
import threading
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_slerp


def pose_matrix(position, rotation):
    matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    matrix[:3, 3] = [position.x, position.y, position.z]
    return matrix


class TruthCheck:
    def __init__(self, directory):
        import rospy
        from nav_msgs.msg import Odometry
        self.lock = threading.Lock()
        self.samples = {'ugv': [], 'uav': []}
        self.log = open(directory/'lidar_truth.jsonl', 'w')
        self.subscribers = [rospy.Subscriber('/stage1/'+robot+'/ground_truth/lidar_odom', Odometry,
                            self.receive, callback_args=robot, queue_size=500) for robot in self.samples]

    def receive(self, message, robot):
        import json
        p, q = message.pose.pose.position, message.pose.pose.orientation
        row = (message.header.stamp.to_sec(), [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        with self.lock:
            if self.samples[robot] and row[0] <= self.samples[robot][-1][0]:
                return
            self.samples[robot].append(row)
            self.log.write(json.dumps({'robot': robot, 'stamp': row[0], 'xyz': row[1], 'xyzw': row[2]})+'\n')
            self.log.flush()

    def interpolate(self, robot, stamp):
        with self.lock:
            samples = list(self.samples[robot])
        times = [row[0] for row in samples]
        index = bisect.bisect_left(times, stamp)
        if index < len(times) and abs(times[index]-stamp) < 1e-9:
            row = samples[index]
            result = quaternion_matrix(row[2]); result[:3, 3] = row[1]
            return result
        if index == 0 or index == len(times):
            raise ValueError(robot+' truth does not bracket keyframe time '+str(stamp))
        a, b = samples[index-1:index+1]
        if b[0]-a[0] > .1:
            raise ValueError('Truth interpolation interval exceeds 0.1 s')
        fraction = (stamp-a[0])/(b[0]-a[0])
        result = quaternion_matrix(quaternion_slerp(a[2], b[2], fraction))
        result[:3, 3] = np.array(a[1])*(1-fraction)+np.array(b[1])*fraction
        return result

    def evaluate(self, match):
        a = self.interpolate('ugv', match.reference_stamp.to_sec())
        b = self.interpolate('uav', match.header.stamp.to_sec())
        truth = np.linalg.inv(b) @ a
        estimated = pose_matrix(match.refined_transform.translation, match.refined_transform.rotation)
        translation = float(np.linalg.norm(estimated[:3, 3]-truth[:3, 3]))
        rotation = float(np.degrees(np.arccos(np.clip((np.trace(estimated[:3, :3].T @ truth[:3, :3])-1)/2, -1, 1))))
        return {'status': 'passed' if translation <= .5 and rotation <= 5 else 'failed',
                'position_error_m': translation, 'rotation_error_deg': rotation,
                'T_UAVkey_UGVkey_truth': truth.tolist(), 'T_UAVkey_UGVkey_estimated': estimated.tolist(),
                'reference_stamp': match.reference_stamp.to_sec(), 'query_stamp': match.header.stamp.to_sec(),
                'position_limit_m': .5, 'rotation_limit_deg': 5,
                'source': 'Gazebo lidar origin poses, independently interpolated; never fed into algorithm'}
