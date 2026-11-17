#!/usr/bin/env python3
"""Check scan-domain separation and manual guide clearance against real SDF boxes."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml


def planar_pose(text):
    values = list(map(float, text.split())) if text else [0]*6
    c, s = np.cos(values[5]), np.sin(values[5])
    return np.array([[c, -s, values[0]], [s, c, values[1]], [0, 0, 1]])


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or package.parents[1]/'results/stage3_rendezvous'/datetime.now().strftime('%Y%m%d_%H%M%S_geometry')
    output.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load((package/'config/stage3_rendezvous_route.yaml').read_text())
    contract = yaml.safe_load((package/'config/stage1_stable_contract.yaml').read_text())
    launch = ET.parse(package/'launch/stage3_gazebo_rendezvous.launch').getroot()
    arguments = {arg.attrib['name']: arg.attrib['value'] for arg in launch.find('include').findall('arg')}
    assert float(arguments['ugv_x']) == config['ugv_start_model_xy'][0]
    assert float(arguments['ugv_y']) == config['ugv_start_model_xy'][1]
    assert float(arguments['uav_x']) == config['uav_model_xy'][0]
    assert float(arguments['uav_y']) == config['uav_model_xy'][1]
    assert float(arguments['max_range']) == config['effective_range_m']
    root = ET.parse(package/'worlds'/config['world']).getroot()
    boxes = []
    for model in root.find('world').findall('model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if not size: continue
                matrix = planar_pose(model.findtext('pose')) @ planar_pose(link.findtext('pose')) @ planar_pose(collision.findtext('pose'))
                boxes.append((model.attrib['name']+'/'+link.attrib['name'], matrix, np.array(list(map(float, size.split())))[:2]/2))
    path = np.array(config['ugv_path_model_xy']); samples = []
    for a, b in zip(path[:-1], path[1:]):
        samples.extend(np.linspace(a, b, int(np.linalg.norm(b-a)/.02)+2))
    samples = np.array(samples)
    clearances = []
    for name, matrix, half in boxes:
        local = (samples-matrix[:2, 2]) @ matrix[:2, :2]
        outside = np.maximum(np.abs(local)-half, 0)
        distance = np.linalg.norm(outside, axis=1)
        clearances.append((float(np.min(distance)), name))
    minimum, nearest = min(clearances)
    ugv = np.array(config['ugv_start_model_xy']) + np.array(contract['robots']['ugv']['model_to_lidar_xyz'])[:2]
    uav = np.array(config['uav_model_xy']) + np.array(contract['robots']['uav']['model_to_lidar_xyz'])[:2]
    margin = float(np.linalg.norm(ugv-uav)-2*config['effective_range_m'])
    result = {'status': 'passed' if minimum >= config['clearance_radius_m'] and margin > config['overlap_check_radius_m'] else 'failed',
              'initial_scan_domain_separation_lower_bound_m': margin, 'route_min_wall_clearance_m': minimum,
              'nearest_obstacle': nearest, 'clearance_radius_m': config['clearance_radius_m'],
              'route_distance_m': float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
              'check': '2D box collisions, 0.02 m sampled guide; not an automated driving or dynamic collision test'}
    (output/'checks.json').write_text(json.dumps(result, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Circle
    figure, axis = plt.subplots(figsize=(12, 6))
    for _, matrix, half in boxes:
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])*half
        corners = corners @ matrix[:2, :2].T + matrix[:2, 2]
        axis.add_patch(Polygon(corners, facecolor='#8b919a', edgecolor='#515761'))
    axis.plot(path[:, 0], path[:, 1], '-o', color='#25954a', label='Manual UGV guide')
    axis.scatter(*config['uav_model_xy'], color='#c44bb5', marker='^', s=80, label='Parked UAV')
    for i, point in enumerate(path): axis.annotate(str(i+1), point, xytext=(3, -14), textcoords='offset points')
    for point, color in [(ugv, '#25954a'), (uav, '#c44bb5')]:
        axis.add_patch(Circle(point, config['effective_range_m'], fill=False, linestyle='--', color=color, alpha=.7))
    axis.set(xlim=(-43, 12), ylim=(-22, 14), xlabel='Gazebo world X (m)', ylabel='Gazebo world Y (m)',
             title='Rendezvous: separate initial scan domains; 12 m effective range')
    axis.set_aspect('equal'); axis.grid(alpha=.2); axis.legend(loc='upper left'); figure.tight_layout()
    figure.savefig(output/'route.png', dpi=160); plt.close(figure)
    print(json.dumps(result, indent=2)); print('CHECKS_DIR='+str(output))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__': raise SystemExit(main())
