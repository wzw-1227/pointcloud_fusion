#!/usr/bin/env python3
"""Independent geometry negatives and an unaligned high-altitude first encounter."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from make_history_fixture import geometry, pcd, pose, COMMIT


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    common=geometry((0,0),corridor=True)
    square=[]
    for z in np.arange(.1,2.51,.15):
        for t in np.arange(-4,4.01,.15):square.extend([[t,-4,z],[t,4,z],[-4,t,z],[4,t,z]])
    floor=common[common[:,2]==0]
    square=np.vstack((floor,np.array(square)))
    fixtures=['cold_positive','mirror','nonoverlap','repeated_corners','ground_only','single_wall','duplicates','ground_ambiguity']
    for name in fixtures:
        root=args.output/name;root.mkdir();(root/'recent').mkdir()
        for robot,label in enumerate(['ugv','uav']):
            points=common.copy()
            if name=='repeated_corners':points=square.copy()
            if name=='single_wall':points=geometry((0,0),style=0)
            if name=='mirror' and robot==1:points[:,0]*=-1
            if name=='nonoverlap' and robot==1:
                cylinders=[]
                for center,radius in [((-3,-3),1.2),((3,3),1.8),((2,-3),.8)]:
                    for angle in np.arange(0,2*np.pi,.035):
                        for z in np.arange(.1,2.51,.15):cylinders.append([center[0]+radius*np.cos(angle),center[1]+radius*np.sin(angle),z])
                points=np.vstack((floor,np.array(cylinders)))
            own=points.copy();own[:,2]-=.35
            key=pose(z=2.45 if robot else 0)
            local=own-key[:3,3]
            if name=='ground_only':recent=local[points[:,2]==0]
            elif name=='duplicates':recent=np.repeat(local[points[:,2]>2][::max(1,len(local)//4)][:4],1000,axis=0)
            else:recent=local[(points[:,2]>=2 if robot else points[:,2]>=.45)&(points[:,2]<=2.5 if robot else points[:,2]<=1.4)]
            if name=='ground_ambiguity':
                second=floor.copy();second[:,2]=-.8
                local=np.vstack((local,second-key[:3,3]))
            stem=label+'_submap_0000';pcd(root/(stem+'.pcd'),local);pcd(root/'recent'/(stem+'.pcd'),recent)
            m=dict(robot_id=robot,session_id=name,epoch=0,submap_id=0,reference_frame='algorithm_'+label+'_odom',
                local_frame=stem,keyframe_timestamp=float(robot+1),segment_end_timestamp=float(robot+1),partial=True,
                point_count=len(local),T_reference_key=key.tolist(),T_gravity_key=np.eye(4).tolist(),upstream_commit=COMMIT,
                observation_quality_valid=True,recent_matching_cloud=dict(pcd='recent/'+stem+'.pcd',point_count=len(recent),
                    start_stamp=float(robot),end_stamp=float(robot+1),frames=50,center_local=[0,0,0]))
            (root/(stem+'.yaml')).write_text(yaml.safe_dump(m,sort_keys=False))
        (root/'frontend.json').write_text(json.dumps(dict(robots={r:dict(T_world_reference=np.eye(4).tolist()) for r in ['ugv','uav']})))


if __name__=='__main__':main()
