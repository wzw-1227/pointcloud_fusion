#!/usr/bin/env python3
"""Summarize observer stages; truth is used only here, after replay finished."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('directory',type=Path)
    parser.add_argument('--truth-cache',type=Path,required=True)
    args=parser.parse_args()
    truth=np.load(args.truth_cache)['truth']
    interpolator=Slerp(truth[:,0],Rotation.from_quat(truth[:,4:8]))
    result={'scope':'fixed original IMU prediction input, not end-to-end LIO acceptance; one initial rigid alignment per replay', 'runs':{}}
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
    for name in ['legacy','fixed_early','fixed_settled','adaptive_early','adaptive_settled','rpy01_early']:
        path=args.directory/name/'pose_stages.csv'
        if not path.exists():continue
        a=np.genfromtxt(path,delimiter=',',names=True)
        t=a['stamp'];positions=np.array([a['graph_'+ax] for ax in 'xyz']).T
        rotations=Rotation.from_euler('xyz',np.array([a['graph_'+ax] for ax in ['roll','pitch','yaw']]).T)
        true_positions=np.array([np.interp(t,truth[:,0],truth[:,i]) for i in [1,2,3]]).T
        true_rotations=interpolator(t)
        alignment=true_rotations[0]*rotations[0].inv()
        translation=true_positions[0]-alignment.apply(positions[0])
        errors=alignment.apply(positions)+translation-true_positions
        angular_error=(true_rotations.inv()*alignment*rotations).magnitude()*180/np.pi
        mask=(t>=197.053)&(t<=258.063)
        corrections={}
        previous_z=np.r_[0,a['graph_z'][:-1]]
        for start,end in [(177,190),(190,197),(197,258.1)]:
            window=(t>=start)&(t<end)
            if not window.any():continue
            corrections[str(start)+'_'+str(end)]={
                'prediction_z_step_sum_m':float(np.sum(a['pred_z'][window]-previous_z[window])),
                'laser_z_correction_sum_m':float(np.sum(a['opt_z'][window]-a['pred_z'][window])),
                'largest_laser_z_correction_m':float(np.max(abs(a['opt_z'][window]-a['pred_z'][window]))),
                'largest_factor_graph_z_change_m':float(np.max(abs(a['graph_z'][window]-a['finalize_z'][window])))}
        result['runs'][name]={'first_stamp':float(t[0]),'processed_frames':len(a),'hover_frames':int(mask.sum()),
            'hover_position_rmse_m':float(np.sqrt(np.mean(np.sum(errors[mask]**2,axis=1)))),
            'hover_rotation_rmse_deg':float(np.sqrt(np.mean(angular_error[mask]**2))),
            'hover_max_position_error_m':float(np.max(np.linalg.norm(errors[mask],axis=1))),
            'hover_degenerate_fraction':float(np.mean(a['degenerate'][mask])),
            'hover_min_eigenvalue':float(np.nanmin(a['lambda_min'][mask])),
            'keyframes_final':int(a['keyframes'][-1]),'correction_stages':corrections}
        if name!='legacy':
            axes[0].plot(t,errors[:,2],label=name)
        if name=='fixed_early':
            axes[1].plot(t,a['opt_z']-a['pred_z'],label='laser correction Z')
            axes[1].plot(t,a['graph_z']-a['finalize_z'],label='factor graph correction Z')
            axes[2].plot(t,a['lambda_min'],label='original plane, early init')
        elif name=='adaptive_early':
            axes[2].plot(t,a['lambda_min'],label='adaptive plane, early init')
    early=np.genfromtxt(args.directory/'fixed_early/pose_stages.csv',delimiter=',',names=True)
    legacy=np.genfromtxt(args.directory/'legacy/pose_stages.csv',delimiter=',',names=True)
    assert np.array_equal(early['stamp'],legacy['stamp'])
    result['cached_input_consistency']={'same_processed_timestamps':True,
        'max_pose_position_difference_m':float(np.max(np.sqrt(sum((early['graph_'+ax]-legacy['graph_'+ax])**2 for ax in 'xyz'))))}
    a=np.genfromtxt(args.directory/'fixed_early/surface_correspondences.csv',delimiter=',',names=True)
    n=np.stack([np.stack([a['n'+str(i)+'_'+ax] for ax in 'xyz'],axis=1) for i in range(5)],axis=1)
    delta=n-n.mean(axis=1)[:,None,:]
    eigenvalues=np.linalg.eigvalsh(np.einsum('nki,nkj->nij',delta,delta)/5)
    original_points=np.stack([a['ori_'+ax] for ax in 'xyz'],axis=1)
    rotations=interpolator(a['stamp']).as_matrix()
    world_z=np.einsum('ni,ni->n',rotations[:,2,:],original_points)+np.interp(a['stamp'],truth[:,0],truth[:,3])
    normal_tilt=np.degrees(np.arccos(np.clip(abs(a['pc']),0,1)))
    result['actual_surface_correspondences']=[]
    for start,end in [(189,191),(191,192),(192,193),(193,194),(194,197),(197,199)]:
        mask=(a['stamp']>=start)&(a['stamp']<end)&(a['accepted']>0)
        floor=mask&(abs(world_z)<.03)
        result['actual_surface_correspondences'].append({'interval':[start,end],'accepted_surface_count':int(mask.sum()),
            'truth_floor_correspondences':int(floor.sum()),
            'floor_fitted_normal_tilt_median_deg':float(np.median(normal_tilt[floor])) if floor.any() else None,
            'floor_normal_tilt_over_60deg_fraction':float(np.mean(normal_tilt[floor]>60)) if floor.any() else None,
            'floor_neighbor_line_fraction':float(np.mean(eigenvalues[floor,1]/np.maximum(eigenvalues[floor,2],1e-12)<.05)) if floor.any() else None})
    axes[0].set_ylabel('Z error (m)');axes[1].set_ylabel('Correction Z (m)');axes[2].set_ylabel('Minimum Hessian eigenvalue')
    axes[2].axhline(100,color='black',linestyle='--',label='unchanged threshold 100')
    axes[2].set_ylim(0,350);axes[2].set_xlabel('Simulation time (s)')
    for ax in axes:
        ax.axvspan(190,197,color='orange',alpha=.1);ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.suptitle('Mapping diagnostic: fixed inputs, initial alignment kept throughout')
    fig.tight_layout(rect=(0,0,1,.96));fig.savefig(args.directory/'mapping_comparison.png',dpi=160)
    (args.directory/'mapping_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
