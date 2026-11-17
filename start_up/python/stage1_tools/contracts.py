"""Load explicit topic contracts and refuse incomplete real calibrations."""
from pathlib import Path
import numpy as np
import yaml


def selected_robots(selection):
    if selection not in ('ugv', 'uav', 'both'):
        raise ValueError('robot must be ugv, uav, or both')
    return ['ugv', 'uav'] if selection == 'both' else [selection]


def load_contract(path, selection='both', environment='sim'):
    data = yaml.safe_load(Path(path).read_text())
    if environment == 'real' and data.get('hardware_configured') is not True:
        raise ValueError('Real topic contract is incomplete: set hardware_configured after checking driver data')
    for name in selected_robots(selection):
        r = data['robots'][name]
        if not isinstance(r['n_scan'], int) or r['n_scan'] <= 0:
            raise ValueError(name + ': n_scan must match the real/simulated sensor')
        if r['point_time_unit'] != 'seconds':
            raise ValueError(name + ': adapt driver per-point time to seconds before using this Velodyne contract')
        if any('REPLACE' in str(v) for v in r.values()):
            raise ValueError(name + ': replace all topic/frame placeholders')
    return data


def validate_hardware(path):
    data = yaml.safe_load(Path(path).read_text())
    h, p = data['stage1_hardware'], data['lio_sam']
    if h.get('configured') is not True:
        raise ValueError('Hardware calibration is incomplete; simulation extrinsics are not real calibration')
    for key in ('extrinsicRot', 'extrinsicRPY'):
        a = np.asarray(p[key], dtype=float)
        if a.size != 9 or not np.all(np.isfinite(a)):
            raise ValueError(key + ' must contain nine finite calibrated values')
        a = a.reshape(3, 3)
        if not np.allclose(a.T @ a, np.eye(3), atol=1e-3) or not np.isclose(np.linalg.det(a), 1, atol=1e-3):
            raise ValueError(key + ' must be a proper rotation')
    for key, source in [('extrinsicTrans', p), ('lidar_to_base_xyz', h),
                        ('lidar_to_base_rpy', h), ('imu_to_base_xyz', h), ('imu_to_base_rpy', h)]:
        a = np.asarray(source[key], dtype=float)
        if a.shape != (3,) or not np.all(np.isfinite(a)):
            raise ValueError(key + ' must contain three measured values')
    for key in ('N_SCAN', 'Horizon_SCAN'):
        if not isinstance(p[key], int) or p[key] <= 0:
            raise ValueError(key + ' is not configured')
    for key in ('imuAccNoise', 'imuGyrNoise', 'imuAccBiasN', 'imuGyrBiasN', 'imuGravity'):
        if p[key] is None or not np.isfinite(p[key]) or p[key] <= 0:
            raise ValueError(key + ' must be a positive measured/configured value')
    if h['time_unit'] != 'seconds' or 'REPLACE' in h['imu_orientation_convention']:
        raise ValueError('Document the IMU convention and convert point times to seconds')
    return data
