#!/usr/bin/env python3
"""Validate persisted match clouds, directions, quality and source record."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parent))
from stage2_verify_exports import read_pcd, validate_transform, COMMIT
from alignment_verify import verify_confirmation


def pcd_colors(path):
    with Path(path).open('rb') as stream:
        header={}
        while True:
            line=stream.readline().decode('ascii').strip()
            if not line or line.startswith('#'):continue
            key,*values=line.split();header[key]=values
            if key=='DATA':break
        payload=stream.read()
    fields=header['FIELDS'];sizes=list(map(int,header['SIZE']));counts=list(map(int,header['COUNT']))
    index=fields.index('rgb');offset=sum(size*count for size,count in zip(sizes[:index],counts[:index]))
    stride=sum(size*count for size,count in zip(sizes,counts))
    if sizes[index]!=4 or counts[index]!=1:raise ValueError('Invalid fused RGB field')
    dtype=np.dtype({'names':['rgb'],'formats':['<u4'],'offsets':[offset],'itemsize':stride})
    return np.frombuffer(payload,dtype=dtype)['rgb'] & 0xffffff


def verify_fusion(path, meta, reference, query):
    f=meta['fusion'];pose=validate_transform(meta['reference']['T_reference_key'])
    transform=validate_transform(meta['T_query_reference_refined'])
    inverse=np.eye(4);inverse[:3,:3]=transform[:3,:3].T;inverse[:3,3]=-transform[:3,:3].T @ transform[:3,3]
    query_pose=pose @ inverse
    ugv=(reference @ pose[:3,:3].T + pose[:3,3]).astype(np.float32).astype(float)
    uav=(query @ query_pose[:3,:3].T + query_pose[:3,3]).astype(np.float32).astype(float)
    joined=np.vstack((ugv,uav));leaf=f['voxel_size_m']
    keys,indices=np.unique(np.floor(joined/leaf).astype(np.int64),axis=0,return_inverse=True)
    sums=np.zeros((len(keys),3));np.add.at(sums,indices,joined)
    centroids=sums/np.bincount(indices)[:,None]
    mask=np.zeros(len(keys),dtype=np.uint8)
    np.bitwise_or.at(mask,indices,np.r_[np.ones(len(ugv),dtype=np.uint8),np.full(len(uav),2,dtype=np.uint8)])
    fused=read_pcd(path.parent/f['pcd'])
    if f['reference_frame']!=meta['reference']['reference_frame'] or f['point_count']!=len(fused):
        raise ValueError('Invalid fused cloud frame/count')
    if fused.shape!=centroids.shape or not np.allclose(fused,centroids,atol=2e-5):
        raise ValueError('Fusion does not preserve both clouds in the UGV frame')
    palette=np.array([0,(40<<16)+(220<<8)+70,(230<<16)+(70<<8)+220,(255<<16)+(200<<8)+40],dtype=np.uint32)
    if not np.array_equal(pcd_colors(path.parent/f['pcd']),palette[mask]):
        raise ValueError('Fused point colors disagree with voxel contributors')
    for flag,name in [(1,'ugv_only_voxels'),(2,'uav_only_voxels'),(3,'shared_voxels')]:
        if int(np.count_nonzero(mask==flag))!=f[name]:raise ValueError('Incorrect fusion contributor counts')
    return {'points':len(fused),'frame':f['reference_frame'],'ugv_only_voxels':f['ugv_only_voxels'],
            'uav_only_voxels':f['uav_only_voxels'],'shared_voxels':f['shared_voxels']}


def verify_map(directory, inputs=None):
    """Recompute the cumulative union from original, untransformed robot maps."""
    directory=Path(directory)
    summary=yaml.load((directory/'summary.yaml').read_text(),Loader=yaml.CSafeLoader)
    if not summary.get('latest_map_checkpoint'):return None
    path=directory/summary['latest_map_checkpoint']
    m=yaml.load(path.read_text(),Loader=yaml.CSafeLoader)
    # Older yaml-cpp checkpoints emitted date/PID identifiers without a string
    # tag; YAML 1.1 interprets digits with underscores as a single integer.
    if any(key in m and not isinstance(m[key],str) for key in ('reference_session_id','query_session_id')):
        scalar=yaml.load(path.read_text(),Loader=yaml.BaseLoader)
        for key in ('reference_session_id','query_session_id'):
            if key in scalar:m[key]=scalar[key]
    def cloud(file):
        with Path(file).open('rb') as stream:
            for line in stream:
                if line.startswith(b'POINTS 0'):return np.empty((0,3))
                if line.startswith(b'DATA '):break
        return read_pcd(file)
    own=[cloud(path.parent/(robot+'_map.pcd')) for robot in ('ugv','uav')]
    leaf=m['voxel_size_m']
    for robot,p in zip(('ugv','uav'),own):
        if len(p)!=m[robot+'_points'] or len(np.unique(np.floor(p/leaf),axis=0))!=len(p):
            raise ValueError('Cumulative own map is not a unique voxel cache')
    t=validate_transform(m['T_UGVmap_UAVmap'])
    fused=cloud(path.parent/'fused_map.pcd')
    if not m['valid']:
        if len(fused):raise ValueError('Invalid alignment retained a cross-robot map')
        return dict(valid=False,ugv_points=len(own[0]),uav_points=len(own[1]))
    projected=(own[1]@t[:3,:3].T+t[:3,3]).astype(np.float32).astype(float)
    joined=np.vstack((own[0],projected))
    keys,indices=np.unique(np.floor(joined/leaf).astype(np.int64),axis=0,return_inverse=True)
    sums=np.zeros((len(keys),3));np.add.at(sums,indices,joined)
    centers=sums/np.bincount(indices)[:,None]
    mask=np.zeros(len(keys),dtype=np.uint8)
    np.bitwise_or.at(mask,indices,np.r_[np.ones(len(own[0]),dtype=np.uint8),np.full(len(own[1]),2,dtype=np.uint8)])
    palette=np.array([0,(40<<16)+(220<<8)+70,(230<<16)+(70<<8)+220,(255<<16)+(200<<8)+40],dtype=np.uint32)
    if fused.shape!=centers.shape or not np.allclose(fused,centers,atol=2e-5):
        raise ValueError('Cumulative union differs from reprojection of original maps')
    if not np.array_equal(pcd_colors(path.parent/'fused_map.pcd'),palette[mask]):
        raise ValueError('Cumulative union colors differ from contributors')
    for flag,name in [(1,'ugv_only_voxels'),(2,'uav_only_voxels'),(3,'shared_voxels')]:
        if int(np.count_nonzero(mask==flag))!=m[name]:raise ValueError('Invalid cumulative contributor counts')
    if inputs:
        caches=[{},{}];seen=set()
        for meta_path in sorted(Path(inputs).glob('*submap*.yaml')):
            meta=yaml.load(meta_path.read_text(),Loader=yaml.CSafeLoader)
            prefix='reference' if meta['robot_id']==0 else 'query'
            if prefix+'_session_id' in m and (meta['session_id']!=m[prefix+'_session_id'] or meta['epoch']!=m[prefix+'_epoch']):continue
            identity=tuple(meta[k] for k in ('robot_id','session_id','epoch','submap_id'))
            if identity in seen:continue
            seen.add(identity);pose=validate_transform(meta['T_reference_key']);points=read_pcd(meta_path.with_suffix('.pcd'))
            points=(points@pose[:3,:3].T+pose[:3,3]).astype(np.float32).astype(float)
            for point in points:
                key=tuple(np.floor(point/leaf).astype(np.int64));old=caches[meta['robot_id']].get(key)
                center=(np.asarray(key)+.5)*leaf
                delta=0 if old is None else np.sum((point-center)**2)-np.sum((old-center)**2)
                if old is None or delta < -1e-12 or abs(delta)<=1e-12 and tuple(point)<tuple(old):
                    caches[meta['robot_id']][key]=point
        for robot,cache in enumerate(caches):
            expected=np.asarray([cache[key] for key in sorted(cache)])
            if own[robot].shape!=expected.shape or not np.array_equal(own[robot],expected):
                raise ValueError('Cumulative own map lost input history or uses synthetic centroids')
    return dict(valid=True,ugv_points=len(own[0]),uav_points=len(own[1]),fused_points=len(fused),
                independently_checked_input_history=bool(inputs))


def verify(directory):
    directory = Path(directory)
    summary = yaml.load((directory/'summary.yaml').read_text(),Loader=yaml.CSafeLoader)
    config = yaml.safe_load((directory/'parameters.yaml').read_text())
    source = json.loads((directory/'compiled_source.json').read_text())
    if source['upstream_commit'] != COMMIT:
        raise ValueError('Wrong upstream source version')
    records = []
    for path in sorted(directory.glob('match_*/match.yaml')):
        m = yaml.safe_load(path.read_text())
        refined = validate_transform(m['T_query_reference_refined'])
        validate_transform(m['T_query_reference_coarse'])
        a = read_pcd(path.parent/'reference.pcd')
        b = read_pcd(path.parent/'query.pcd')
        aligned = read_pcd(path.parent/'reference_in_query.pcd')
        expected = a @ refined[:3, :3].T + refined[:3, 3]
        if aligned.shape != expected.shape or not np.allclose(aligned, expected, atol=2e-5):
            raise ValueError('Saved source/target transform direction differs')
        if m['reference']['robot_id'] != 0 or m['query']['robot_id'] != 1:
            raise ValueError('Robot identities are reversed')
        contour = m.get('registration_branch') == 'contour_xy_ground'
        evidence_v2 = m.get('visibility_validation', {}).get('version') == 2
        if evidence_v2:
            verify_confirmation(path, m, config)
        if not contour and not (m['ransac_inliers'] >= config['min_inliers'] and
                0 <= m['fitness_m2'] < config['max_fitness'] and
                m['overlap_smaller_cloud'] >= config['min_overlap'] and
                m['residual_rmse_m'] <= config['max_residual_rmse']):
            raise ValueError('Accepted match violates quality gates')
        structural_min = config.get('min_structural_overlap', 0)
        if contour:
            p = m.get('contour_validation', {})
            coverages = [p.get('overlap_reference', -1), p.get('overlap_query', -1)]
            score = max(coverages) * np.sqrt(max(0, min(coverages))) * np.exp(
                -(p.get('xy_rmse_m', np.inf) / config.get('contour_rmse', .2))**2)
            if not (config.get('matching_profile') == 'partial_contour' and p.get('version') == 1 and
                    m['ransac_inliers'] >= config['min_inliers'] and p.get('converged') and
                    max(coverages) >= config.get('contour_high_coverage', .7) and
                    min(coverages) >= config.get('contour_low_coverage', .25) and
                    p.get('independent_correspondences', 0) >= config.get('contour_min_pairs', 60) and
                    p.get('xy_rmse_m', np.inf) <= config.get('contour_rmse', .2) and
                    p.get('span_m', 0) >= config.get('partial_span', 2) and
                    p.get('direction_separation_deg', 0) >= config.get('partial_direction_angle_deg', 30) and
                    min(p.get('direction_support_first', 0), p.get('direction_support_second', 0)) >= config.get('partial_direction_min_pairs', 20) and
                    p.get('xy_information_ratio', 0) >= config.get('partial_information_ratio', 1e-4) and
                    p.get('query_height_above_ground_m', 0) >= config.get('contour_min_height', 2.5) and
                    min(p.get('ground_support_reference', 0), p.get('ground_support_query', 0)) >= 100 and
                    max(p.get('ground_rmse_reference_m', np.inf), p.get('ground_rmse_query_m', np.inf)) <= .03 and
                    (not m.get('has_competing_cluster', True) or p.get('score_margin', 0) >= config.get('partial_score_margin', .05)) and
                    np.isclose(p.get('candidate_score', -1), score)):
                raise ValueError('Accepted match violates contour/ground gates')
            if m.get('refinement_method')!='gravity_contour_point_to_line_huber_irls':
                raise ValueError('Invalid contour refinement method')
            if 'ground_validation' in m:
                ground=m['ground_validation']
                for role in ('reference','query'):
                    prefix='ground_'+role+'_'
                    if not (ground.get(prefix+'valid')==1 and ground.get(prefix+'independent_voxels',0)>=100 and
                        ground.get(prefix+'rmse_m',np.inf)<=.03 and ground.get(prefix+'span_m',0)>=3 and
                        ground.get(prefix+'second_direction_width_m',0)>=1 and -.1>=ground.get(prefix+'height_map_m',0)>=-1.5):
                        raise ValueError('Accepted contour has invalid historical ground provenance')
            if p.get('reflection_checked') and p.get('reflection_score',0)>p['candidate_score']+config.get('partial_score_margin',.05)+1e-7:
                raise ValueError('Accepted contour is better explained by reflection')
            if p.get('visibility_check_enabled') and not (p.get('visibility_checked_rays',0)>=50 and
                    p.get('visibility_contradictions',np.inf)<=.05*p['visibility_checked_rays']):
                raise ValueError('Accepted contour contradicts comparable 3D rays')
            if p.get('alignment_prior_used') and not evidence_v2:
                prior=validate_transform(p['T_UGVmap_UAVmap_prior'])
                current=validate_transform(m['reference']['T_reference_key'])@np.linalg.inv(refined)@np.linalg.inv(validate_transform(m['query']['T_reference_key']))
                delta=np.linalg.inv(prior)@current
                angle=np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1))
                if p.get('alignment_prior_revision',0)<=0 or np.linalg.norm(delta[:3,3])>.25+1e-7 or angle>np.deg2rad(3)+1e-7:
                    raise ValueError('Accepted contour conflicts with its disclosed validated alignment prior')
        elif config.get('matching_profile') in ('partial', 'partial_contour'):
            p = m.get('partial_validation', {})
            structure = m.get('structural_check', {})
            coverages = [structure.get('overlap_reference', -1), structure.get('overlap_query', -1)]
            score = max(coverages) * np.sqrt(max(0, min(coverages))) * np.exp(
                -(p.get('correspondence_rmse_m', np.inf) / config.get('partial_rmse', .2))**2)
            if not (m.get('refinement_method') == 'gravity_point_to_plane_huber_irls' and
                    p.get('version') == 1 and p.get('converged') and structure.get('enabled') and
                    all(0 <= v <= 1 for v in coverages) and
                    max(coverages) >= config.get('partial_high_coverage', .7) and
                    min(coverages) >= config.get('partial_low_coverage', .25) and
                    p.get('structural_correspondences', 0) >= config.get('partial_min_pairs', 80) and
                    p.get('effective_correspondences', 0) >= p.get('structural_correspondences', 0) and
                    0 <= p.get('correspondence_rmse_m', np.inf) <= config.get('partial_rmse', .2) and
                    p.get('structural_span_m', 0) >= config.get('partial_span', 2) and
                    p.get('direction_separation_deg', 0) >= config.get('partial_direction_angle_deg', 30) and
                    min(p.get('direction_support_first', 0), p.get('direction_support_second', 0)) >=
                    config.get('partial_direction_min_pairs', 20) and
                    p.get('information_ratio', 0) >= config.get('partial_information_ratio', 1e-4) and
                    min(p.get('horizontal_support_reference', 0), p.get('horizontal_support_query', 0)) >= 20 and
                    0 <= p.get('horizontal_plane_residual_m', np.inf) <= .1 and
                    (not m.get('has_competing_cluster', True) or p.get('score_margin', -1) >= config.get('partial_score_margin', .05)) and
                    np.isclose(p.get('candidate_score', -1), score)):
                raise ValueError('Accepted match violates partial geometry gates')
            for role in ('reference', 'query'):
                effective = validate_transform(m[role]['T_matching_gravity_key'])
                pose = validate_transform(m[role]['T_reference_key'])
                yaw = np.arctan2(pose[1, 0], pose[0, 0])
                rz = np.array([[np.cos(yaw), np.sin(yaw), 0],
                               [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
                if not np.allclose(effective[:3, :3], rz @ pose[:3, :3], atol=1e-6):
                    raise ValueError('Invalid canonical matching gravity')
            if p.get('visibility_check_enabled') and not (
                    p.get('visibility_checked_rays', 0) >= 50 and
                    0 <= p.get('visibility_contradictions', np.inf) <= .05 * p['visibility_checked_rays']):
                raise ValueError('Accepted match contradicts observed free space')
            historical = any(m['matching_scope'].get(role + '_cloud_source') != 'recent'
                             for role in ('reference', 'query'))
            if historical != p.get('historical_cloud_gate') or (
                    historical and min(coverages) < config.get('partial_history_low_coverage', .6)):
                raise ValueError('Accepted historical cloud violates compatibility coverage gate')
        elif structural_min > 0:
            structure = m.get('structural_check', {})
            if not (structure.get('enabled') and
                    structure.get('reference_points', 0) >= config.get('min_structural_points', 20) and
                    structure.get('query_points', 0) >= config.get('min_structural_points', 20) and
                    structural_min <= structure.get('overlap_reference', -1) <= 1 and
                    structural_min <= structure.get('overlap_query', -1) <= 1):
                raise ValueError('Accepted match violates the structural overlap gate')
        if contour:
            for role in ('reference','query'):
                effective=validate_transform(m[role]['T_matching_gravity_key'])
                pose=validate_transform(m[role]['T_reference_key'])
                yaw=np.arctan2(pose[1,0],pose[0,0]);c,s=np.cos(yaw),np.sin(yaw)
                if not np.allclose(effective[:3,:3],np.array([[c,s,0],[-s,c,0],[0,0,1]])@pose[:3,:3],atol=1e-6):
                    raise ValueError('Invalid contour canonical gravity')
        record={'match': str(path), 'points_reference': len(a), 'points_query': len(b)}
        if 'fusion' in m:record['fusion']=verify_fusion(path,m,a,b)
        records.append(record)
    if not records or len(records) != summary['accepted_matches']:
        raise ValueError('Accepted match count is empty or inconsistent')
    if summary['status'] == 'failed':
        raise ValueError('Matcher stopped with an error')
    report = {'status': 'passed', 'matches': records,'cumulative_map':verify_map(directory)}
    (directory/'file_checks.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('directory')
    parser.add_argument('--map-only',action='store_true');parser.add_argument('--inputs',type=Path)
    args = parser.parse_args()
    report=verify_map(args.directory,args.inputs) if args.map_only else verify(args.directory)
    if args.inputs and not args.map_only:report['cumulative_map']=verify_map(args.directory,args.inputs)
    if report is None:raise ValueError('No cumulative map checkpoint')
    print(json.dumps(report, indent=2))
