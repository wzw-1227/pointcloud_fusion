#!/usr/bin/env python3
"""Inspect exact Gazebo ray hit identities in static, isolated model copies.

No ROS, PX4, control, LIO or truth-assisted estimator. A static lidar geometry
counterfactual is not a flight acceptance test. Package models stay unchanged.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET


PLUGIN = r'''
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <fstream>
#include <map>
namespace gazebo {
class HitProbe : public SensorPlugin {
 sensors::RaySensorPtr sensor; event::ConnectionPtr connection;
 std::string output; int count=0;
 public: void Load(sensors::SensorPtr parent,sdf::ElementPtr sdf) override {
  sensor=std::dynamic_pointer_cast<sensors::RaySensor>(parent);
  if(!sensor) throw std::runtime_error("ray sensor required");
  output=sdf->Get<std::string>("output");
  connection=sensor->ConnectUpdated(std::bind(&HitProbe::Update,this));
  sensor->SetActive(true);
 }
 void Update() {
  if(++count!=4) return;
  std::ofstream out(output);
  out << "ring,column,range,collision\n";
  auto shape=sensor->LaserShape();
  std::ofstream pose(output+".pose");
  pose << "parent " << sensor->ParentName() << " sensor-relative " << sensor->Pose() << '\n';
  auto world=physics::get_world(sensor->WorldName());
  auto link=boost::dynamic_pointer_cast<physics::Link>(world->EntityByName(sensor->ParentName()));
  if(link) pose << "link-world " << link->WorldPose() << '\n';
  ignition::math::Vector3d start,end;
  shape->Ray(0)->GlobalPoints(start,end);
  pose << "ray0-global " << start << " -> " << end << '\n';
  for(unsigned int j=0;j<sensor->VerticalRayCount();++j)
   for(unsigned int i=0;i<sensor->RayCount();++i) {
    auto ray=shape->Ray(j*sensor->RayCount()+i);
    out << j << ',' << i << ',' << sensor->Range(j*sensor->RayCount()+i)
        << ',' << ray->CollisionName() << '\n';
   }
  out.flush();
 }
};
GZ_REGISTER_SENSOR_PLUGIN(HitProbe)
}
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--variants',nargs='+',choices=['original','compact_body','ray_start_035','body_mesh'],
                        default=['original','compact_body','ray_start_035','body_mesh'])
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    if subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode==0:
        raise RuntimeError('another Gazebo is active; do not interfere')
    source=output/'hit_probe.cpp';source.write_text(PLUGIN)
    binary=output/'libhit_probe.so'
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','gazebo'],text=True))
    command=['c++','-shared','-fPIC','-O2',str(source),'-o',str(binary)]+flags
    subprocess.run(command,check=True)
    model_root=output/'models';model_root.mkdir(exist_ok=True)
    checks=[]
    # The two counterfactuals isolate self-occlusion, not accepted fixes. Neither
    # is applied to package defaults or used to label LIO/fusion as passing.
    for variant in args.variants:
        name='occlusion_'+variant;directory=model_root/name;directory.mkdir(exist_ok=True)
        model=ET.parse(root/'models/stage1_iris_wide/stage1_iris_wide.sdf')
        body=model.getroot().find('model');body.set('name',name)
        static=body.find('static')
        if static is None:static=ET.SubElement(body,'static')
        static.text='true'
        for parent in model.getroot().iter():
            for child in list(parent):
                if child.tag in ('plugin','sensor'):parent.remove(child)
        # Expand auxiliary payload models so their collision geometry is kept
        # without loading unrelated camera/IMU/GPS plugins in a non-ROS probe.
        for include_aux in list(body.findall('include')):
            uri=include_aux.findtext('uri')
            if uri=='model://stage1_lidar_wide':continue
            aux_name=uri.replace('model://','')
            candidates=[Path(path)/aux_name for path in os.environ.get('GAZEBO_MODEL_PATH','').split(':') if path]
            aux_dir=next((d for d in candidates if (d/'model.config').exists()),None)
            if aux_dir is None:raise RuntimeError('cannot resolve '+uri)
            sdf_name=ET.parse(aux_dir/'model.config').getroot().findtext('sdf')
            auxiliary=ET.parse(aux_dir/sdf_name).getroot().find('model')
            for parent in auxiliary.iter():
                for child in list(parent):
                    if child.tag in ('plugin','sensor'):parent.remove(child)
            pose=auxiliary.find('pose')
            if pose is None:pose=ET.SubElement(auxiliary,'pose')
            pose.text=include_aux.findtext('pose','0 0 0 0 0 0')
            body.remove(include_aux);body.append(auxiliary)
        if variant=='compact_body':
            body.find("link[@name='base_link']/collision/geometry/box/size").text='0.14 0.14 0.11'
        elif variant=='body_mesh':
            # Match the existing visible airframe rather than invent a body
            # width or make the airframe invisible to laser rays.
            geometry=body.find("link[@name='base_link']/collision/geometry")
            geometry.remove(geometry.find('box'))
            mesh=ET.SubElement(geometry,'mesh')
            ET.SubElement(mesh,'uri').text='model://iris/meshes/iris.stl'
            ET.SubElement(mesh,'scale').text='1 1 1'
        include=body.find("include[uri='model://stage1_lidar_wide']")
        if include is None:raise RuntimeError('unexpected lidar include')
        body.remove(include)
        lidar=ET.parse(root/'models/stage1_lidar_wide/stage1_lidar_wide.sdf').getroot().find('model')
        lidar.set('name','3d_lidar');ET.SubElement(lidar,'pose').text='0 0 0.08 0 0 0'
        sensor=lidar.find('link/sensor')
        sensor.remove(sensor.find('plugin'))
        if variant=='ray_start_035':sensor.find('ray/range/min').text='.35'
        plugin=ET.SubElement(sensor,'plugin',name='hit_probe',filename=str(binary))
        csv=output/(variant+'.csv');ET.SubElement(plugin,'output').text=str(csv)
        body.append(lidar)
        ET.indent(model,space='  ') if hasattr(ET,'indent') else None
        model.write(directory/'model.sdf')
        (directory/'model.config').write_text('<model><name>'+name+'</name><version>1</version><sdf version="1.5">model.sdf</sdf></model>')
        world=output/(variant+'.world')
        world.write_text('''<sdf version="1.5"><world name="occlusion">
          <physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
          <include><uri>model://ground_plane</uri></include>
          <include><uri>model://'''+name+'''</uri><pose>0 0 1.8 0 0 0</pose></include>
          </world></sdf>''')
        env=os.environ.copy()
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
        env['GAZEBO_MASTER_URI']='http://127.0.0.1:'+str(port)
        env['GAZEBO_MODEL_PATH']=str(model_root)+':'+env.get('GAZEBO_MODEL_PATH','')
        with (output/(variant+'.log')).open('w') as log:
            process=subprocess.Popen(['gzserver','--verbose',str(world)],env=env,stdout=log,stderr=subprocess.STDOUT)
            try:
                deadline=time.monotonic()+45
                while (not csv.exists() or csv.stat().st_size<10000) and time.monotonic()<deadline:
                    if process.poll() is not None:raise RuntimeError('probe Gazebo exited; see log')
                    time.sleep(.1)
                if not csv.exists():raise RuntimeError('no probe output; see log')
                time.sleep(.2)
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=15)
                    except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5)
        import csv as csv_module
        rows=list(csv_module.DictReader(csv.open()))
        rings=[]
        for ring in range(16):
            group=[r for r in rows if int(r['ring'])==ring]
            counts={}
            for row in group:
                identity=row['collision'];counts[identity]=counts.get(identity,0)+1
            rings.append({'ring':ring,'hits':counts})
        check={'variant':variant,'rays':len(rows),'rings':rings,
               'self_hits':sum(name in r['collision'] for r in rows),
               'floor_hits':sum('ground_plane' in r['collision'] for r in rows)}
        checks.append(check)
        print(json.dumps({k:v for k,v in check.items() if k!='rings'}),flush=True)
    (output/'occlusion.json').write_text(json.dumps({'scope':'static geometry diagnostic only; no flight, LIO or fusion acceptance',
        'lidar_mount_z_m':.08,'body_z_m':1.8,'compile_command':command,'results':checks},indent=2)+'\n')


if __name__=='__main__':main()
