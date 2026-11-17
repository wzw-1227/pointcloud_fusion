"""Independent NumPy/SciPy verification of single-scan and confirmation evidence."""
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree
from stage2_verify_exports import read_pcd, validate_transform


def scan(path, config):
    meta = yaml.safe_load(Path(path).read_text())
    recent = meta['recent_matching_cloud']
    if recent['frames'] < 1 or recent['start_stamp'] > recent['end_stamp'] or not np.isfinite([recent['start_stamp'], recent['end_stamp'], *recent['center_local']]).all():
        raise ValueError('Invalid confirmation scan window/origin')
    relative = Path(recent['latest_scan_pcd'])
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('Unsafe confirmation scan path')
    points = read_pcd(Path(path).parent / relative)
    center = np.asarray(recent['center_local'])
    rays = points - center
    depths = np.linalg.norm(rays, axis=1)
    keep = depths >= .5
    points, rays, depths = points[keep], rays[keep], depths[keep]
    step = np.radians(config.get('visibility_angle_deg', .5))
    azimuth = np.arctan2(rays[:, 1], rays[:, 0]) + np.pi
    azimuth[azimuth >= 2*np.pi] = 0
    bins = np.column_stack((np.floor(azimuth/step),
                           np.floor((np.arcsin(np.clip(rays[:, 2]/depths, -1, 1))+np.pi/2)/step)))
    order = np.lexsort((np.arange(len(points)), depths, bins[:, 1], bins[:, 0]))
    sorted_bins = bins[order]
    first = np.r_[True, np.any(sorted_bins[1:] != sorted_bins[:-1], axis=1)]
    ids = order[first]
    points, rays, depths = points[ids], rays[ids], depths[ids]
    gravity = validate_transform(meta['T_matching_gravity_key'])[:3, :3]
    structure = (points @ gravity.T)[:, 2] >= meta['visibility_floor_local_m'] + config.get('structural_height_above_floor', .4)
    return dict(meta=meta, points=points, depths=depths,
                directions=(rays/depths[:, None]).astype(np.float32).astype(float),
                structure=structure, center=center)


def direction(source, target, transform, config):
    points = source['points'] @ transform[:3, :3].T + transform[:3, 3] - target['center']
    depths = np.linalg.norm(points, axis=1)
    keep = depths >= .5
    points, depths = points[keep], depths[keep]
    structure = source['structure'][keep]
    units = (points/depths[:, None]).astype(np.float32).astype(float)
    distance, ids = cKDTree(target['directions']).query(units)
    keep = distance**2 <= (2*np.sin(np.radians(config.get('visibility_angle_deg', .5))/2))**2
    ids, distance, depths, structure = ids[keep], distance[keep]**2, depths[keep], structure[keep]
    # Same target direction is evidence once, even when many source points project onto it.
    selected = {}
    for i, (target_id, angle, depth) in enumerate(zip(ids, distance, depths)):
        previous = selected.get(target_id)
        if previous is None or angle < distance[previous]-1e-12 or abs(angle-distance[previous]) <= 1e-12 and depth < depths[previous]:
            selected[target_id] = i
    indexes = np.asarray(list(selected.values()), dtype=int)
    ids, depths, structure = ids[indexes], depths[indexes], structure[indexes]
    wall = structure | target['structure'][ids]
    conflict = depths + config.get('visibility_depth_m', .75) < target['depths'][ids]
    return dict(checked=len(ids), conflicts=int(conflict.sum()),
                structure_checked=int(wall.sum()), structure_conflicts=int((conflict & wall).sum()))


def visibility(reference, query, transform, config):
    return dict(ugv_in_uav=direction(reference, query, transform, config),
                uav_in_ugv=direction(query, reference, np.linalg.inv(transform), config))


def check_counts(actual, recorded, config):
    for key, result in actual.items():
        expected = recorded[key]
        for name, value in result.items():
            # FLANN and cKDTree can break exact angular ties differently.
            if abs(value-expected[name]) > 2:
                raise ValueError('Single-scan ray statistics do not reproduce: '+key+'/'+name)
        if result['checked'] < config.get('visibility_min_rays', 50) or result['structure_checked'] < config.get('visibility_min_structure_rays', 20):
            raise ValueError('Confirmation has insufficient independent rays')
        limit = config.get('visibility_max_conflict', .05)
        if result['conflicts'] > limit*result['checked'] or result['structure_conflicts'] > limit*result['structure_checked']:
            raise ValueError('Confirmation contradicts observed free space')


def cells(observation):
    meta = observation['meta']
    pose = validate_transform(meta['T_reference_key'])
    points = observation['points'][observation['structure']]
    own = points @ pose[:3, :3].T + pose[:3, 3]
    return {tuple(row) for row in np.floor(own/.5).astype(np.int64)}


def verify_confirmation(path, match, config):
    trust = match['alignment_validation']
    count = trust['independent_observations']
    if trust['state'] != 'confirmed' or count < trust['required_observations'] or trust['required_observations'] < config.get('alignment_initial_observations', 2):
        raise ValueError('Published relation was not independently confirmed')
    paths = trust.get('supports', [])
    if len(paths) != count:
        raise ValueError('Confirmation support count differs from evidence files')
    pose = validate_transform(match['reference']['T_reference_key']) @ np.linalg.inv(validate_transform(match['T_query_reference_refined'])) @ np.linalg.inv(validate_transform(match['query']['T_reference_key']))
    previous = []
    current = Path(trust['current_observation'])
    if current.is_absolute() or '..' in current.parts:
        raise ValueError('Unsafe current scan path')
    current_folder = Path(path).parent / current.parent
    current_observations = [scan(current_folder/'ugv.yaml', config), scan(current_folder/'uav.yaml', config)]
    check_counts(visibility(*current_observations, validate_transform(match['T_query_reference_refined']), config), match['visibility_validation'], config)
    for relative in paths:
        relative = Path(relative)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Unsafe confirmation support path')
        folder = Path(path).parent / relative.parent
        support = yaml.safe_load((Path(path).parent / relative).read_text())
        observations = [scan(folder/'ugv.yaml', config), scan(folder/'uav.yaml', config)]
        local = np.linalg.inv(validate_transform(observations[1]['meta']['T_reference_key'])) @ np.linalg.inv(pose) @ validate_transform(observations[0]['meta']['T_reference_key'])
        if not np.allclose(local, support['T_query_reference_refined'], atol=1e-6):
            raise ValueError('Confirmation witness was not revalidated at the published relation')
        actual = visibility(*observations, local, config)
        check_counts(actual, support['visibility_validation'], config)
        current = [cells(o) for o in observations]
        independent = not previous
        for role in range(2):
            p = observations[role]['meta']
            recent = p['recent_matching_cloud']
            own = validate_transform(p['T_reference_key'])
            origin = own[:3, :3] @ recent['center_local'] + own[:3, 3]
            separate, old_cells = True, set()
            for old, old_grid in previous:
                other = old[role]['meta'];window = other['recent_matching_cloud'];other_pose = validate_transform(other['T_reference_key'])
                if p['session_id'] != other['session_id'] or p['epoch'] != other['epoch']:
                    raise ValueError('Confirmation mixes sessions or epochs')
                other_origin = other_pose[:3, :3] @ window['center_local'] + other_pose[:3, 3]
                separate &= (recent['start_stamp'] > window['end_stamp']+1e-6 or window['start_stamp'] > recent['end_stamp']+1e-6)
                separate &= np.linalg.norm(origin-other_origin) >= config.get('alignment_baseline_m', .5)-1e-6
                old_cells.update(old_grid[role])
            novel = len(current[role]-old_cells)/max(1, len(current[role]))
            independent |= separate and novel >= config.get('alignment_structure_novelty', .1)-1e-6
        if not independent:
            raise ValueError('Repeated views were counted as independent confirmation')
        previous.append((observations, current))
    return dict(independent_observations=count, single_scan_evidence_recomputed=True)
