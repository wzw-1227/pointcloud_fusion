"""Respect PointCloud2 offsets, endian markers and row padding."""
import numpy as np

FIELD_TYPES = {1:'i1', 2:'u1', 3:'i2', 4:'u2', 5:'i4', 6:'u4', 7:'f4', 8:'f8'}


def cloud_array(message):
    if message.width * message.height == 0:
        return np.empty(0, dtype=[])
    names, formats, offsets = [], [], []
    endian = '>' if message.is_bigendian else '<'
    for f in message.fields:
        if f.datatype not in FIELD_TYPES:
            raise ValueError('Unsupported PointField datatype')
        names.append(f.name)
        formats.append((endian+FIELD_TYPES[f.datatype], (f.count,)) if f.count > 1 else endian+FIELD_TYPES[f.datatype])
        offsets.append(f.offset)
    dtype = np.dtype({'names':names, 'formats':formats, 'offsets':offsets, 'itemsize':message.point_step})
    if len(message.data) < message.row_step * message.height:
        raise ValueError('Truncated PointCloud2 data')
    return np.ndarray((message.height,message.width), dtype=dtype, buffer=message.data,
                      strides=(message.row_step,message.point_step)).reshape(-1)


def xyz_samples(message, count=128):
    a = cloud_array(message)
    if not len(a) or not {'x','y','z'}.issubset(a.dtype.names):
        raise ValueError('Empty cloud or missing XYZ')
    indexes = np.linspace(0,len(a)-1,min(count,len(a)),dtype=int)
    return np.column_stack([a[k][indexes] for k in ('x','y','z')]).astype(float)


def inspect_cloud(message, contract, raw=False):
    issues, warnings = [], []
    a = cloud_array(message)
    if not len(a):
        return ['empty point cloud'], warnings, {'points':0}
    required = {'x','y','z','intensity'} | ({'ring','time'} if raw else set())
    missing = required-set(a.dtype.names)
    if missing:
        issues.append('missing fields: '+','.join(sorted(missing)))
    if {'x','y','z'}.issubset(a.dtype.names):
        if not np.all(np.isfinite(np.column_stack([a[k] for k in ('x','y','z')]))):
            issues.append('nonfinite XYZ')
    info = {'points':len(a), 'fields':list(a.dtype.names)}
    if raw and 'ring' in a.dtype.names:
        rings = np.unique(a['ring'])
        info['rings'] = [int(x) for x in rings]
        if a['ring'].dtype.kind != 'u' or a['ring'].dtype.itemsize != 2:
            issues.append('ring must be UINT16 for this LIO Velodyne profile')
        if np.any(rings >= contract['n_scan']):
            issues.append('ring exceeds configured N_SCAN')
    if raw and 'time' in a.dtype.names:
        times = a['time']
        if times.dtype.kind != 'f' or times.dtype.itemsize != 4:
            issues.append('time must be FLOAT32 seconds for this LIO Velodyne profile')
        if not np.all(np.isfinite(times)) or np.any(times < 0) or np.any(times > 1):
            issues.append('invalid relative point time; expected finite seconds within one scan')
        info['point_time_min'] = float(np.min(times)) if np.all(np.isfinite(times)) else None
        info['point_time_max'] = float(np.max(times)) if np.all(np.isfinite(times)) else None
        if np.all(np.abs(times) < 1e-8):
            text = 'point time is zero: instantaneous simulated scan; real rotational deskew is unverified'
            (issues if contract.get('require_nonzero_point_time') else warnings).append(text)
    return issues, warnings, info
