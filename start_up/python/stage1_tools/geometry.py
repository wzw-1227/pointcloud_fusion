"""Rigid transforms only: Stage 1 must never fit trajectory scale."""
import numpy as np


def quaternion_rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    result = np.eye(4)
    result[:3, :3] = quaternion_rotation([q.x, q.y, q.z, q.w])
    result[:3, 3] = [p.x, p.y, p.z]
    if not np.all(np.isfinite(result)):
        raise ValueError('Nonfinite pose')
    return result


def inverse_pose(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def initial_alignment(estimate, truth):
    return truth @ inverse_pose(estimate)


def rotation_error_degrees(a, b):
    cosine = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(cosine, -1, 1))))


def interpolate_pose(a, b, weight):
    """Interpolate translation and rotation on SO(3), without external dependencies."""
    relative = a[:3, :3].T @ b[:3, :3]
    angle = np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1))
    if angle < 1e-8:
        rotation = a[:3, :3]
    else:
        values, vectors = np.linalg.eig(relative)
        axis = np.real(vectors[:, np.argmin(np.abs(values-1))])
        axis /= np.linalg.norm(axis)
        skew_direction = np.array([relative[2,1]-relative[1,2], relative[0,2]-relative[2,0], relative[1,0]-relative[0,1]])
        if axis @ skew_direction < 0:
            axis = -axis
        x, y, z = axis
        k = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        theta = angle * weight
        rotation = a[:3, :3] @ (np.eye(3)+np.sin(theta)*k+(1-np.cos(theta))*(k@k))
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = (1-weight)*a[:3, 3]+weight*b[:3, 3]
    return result
