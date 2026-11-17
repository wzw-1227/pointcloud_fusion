#!/usr/bin/env python3
"""Expand launch files without starting processes and verify mode isolation."""
import json
from pathlib import Path
import roslaunch
import yaml


def main():
    root = Path(__file__).resolve().parents[3]
    def load(name, arguments):
        return roslaunch.config.load_config_default(
            [(str(root/'src/start_up/launch'/name), arguments)], None)
    report = {}
    for robot in ('ugv', 'uav', 'both'):
        config = load('stage2_sim.launch', ['robot:='+robot, 'gui:=false', 'rviz:=false'])
        gazebo = [n for n in config.nodes if n.package == 'gazebo_ros' and n.type == 'gzserver']
        assert len(gazebo) == 1, (robot, len(gazebo))
        active = ('ugv', 'uav') if robot == 'both' else (robot,)
        for name in active:
            key = ('' if name == 'ugv' else '/uav')+'/lio_sam/loopClosureEnableFlag'
            assert config.params[key].value is False
        assert not any('odom_combined' in n.args for n in config.nodes)
        report[robot] = {'gazebo_instances': len(gazebo), 'loop_closure_disabled': True}
    config = load('stage1_sim.launch', ['gui:=false', 'rviz:=false'])
    assert config.params['/lio_sam/loopClosureEnableFlag'].value is True
    assert config.params['/uav/lio_sam/loopClosureEnableFlag'].value is True
    report['stage1_default_preserved'] = True
    (root/'results/stage2/launch_checks.json').write_text(json.dumps(report, indent=2)+'\n')
    config=load('stage3_sim.launch',['gui:=false'])
    assert len([n for n in config.nodes if n.package=='gazebo_ros' and n.type=='gzserver'])==1
    assert all(config.params[p+'/loopClosureEnableFlag'].value is False for p in ('/lio_sam','/uav/lio_sam'))
    builder_config=[p.value for k,p in config.params.items() if k.endswith('/config') and 'stage3_live_submaps.yaml' in str(p.value)]
    assert len(builder_config)==1
    live=yaml.safe_load(Path(builder_config[0]).read_text())
    base=yaml.safe_load((root/'src/ehpr_bridge/config/stage2.yaml').read_text())
    assert live['voxel_map']==base['voxel_map']
    assert live['robots']['ugv']['preview_initial_displacement_m']==0
    assert live['robots']['uav']['preview_initial_displacement_m']==.5
    online=root/'src/start_up/rviz/stage3_online.rviz'
    assert any(n.package=='rviz' and str(online) in n.args for n in config.nodes)
    display=yaml.safe_load(online.read_text())['Visualization Manager']
    assert display['Global Options']['Fixed Frame']=='viz_world'
    enabled={d.get('Topic') for d in display['Displays'] if d.get('Enabled')}
    assert {'/lio_sam/mapping/cloud_registered_raw','/uav/lio_sam/mapping/cloud_registered_raw','/fused_submap'}<=enabled
    offline=roslaunch.config.load_config_default([(str(root/'src/ehpr_bridge/launch/stage3_replay.launch'),
        ['input_directory:='+str(root/'results/stage3/synthetic_complementary_inputs')])],None)
    assert not any(n.package=='gazebo_ros' for n in offline.nodes)
    stage3={'status':'passed','formal_robot':'both','gazebo_instances':1,'internal_loop_disabled':True,
        'offline_has_no_gazebo':True,'observed_clouds_enabled':True,'fused_topic':'/fused_submap',
        'live_preview_config':builder_config[0],'official_voxel_defaults_preserved':True}
    (root/'results/stage3/launch_checks.json').write_text(json.dumps(stage3,indent=2)+'\n')
    print(json.dumps(report, indent=2))
    print(json.dumps(stage3, indent=2))


if __name__ == '__main__':
    main()
