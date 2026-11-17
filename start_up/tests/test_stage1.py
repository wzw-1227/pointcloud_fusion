import unittest
from types import SimpleNamespace as NS
from pathlib import Path
import numpy as np
from stage1_tools.geometry import initial_alignment,rotation_error_degrees,interpolate_pose
from stage1_tools.evaluation import trajectory_metrics,Observations,make_report
from stage1_tools.cloud import cloud_array
from stage1_tools.contracts import load_contract,validate_hardware
from stage1_tools.scan_gate import ScanGate

ROOT=Path(__file__).resolve().parents[1]


class GeometryTests(unittest.TestCase):
    def test_initial_alignment_keeps_later_drift_visible(self):
        truth=[];estimate=[]
        for i in range(11):
            t=np.eye(4);t[0,3]=i;truth.append((float(i),t))
            e=t.copy();e[0,3]=10+i*1.2;estimate.append((float(i),e))
        metric=trajectory_metrics(estimate,truth,.05)
        self.assertAlmostEqual(metric['endpoint_position_error_m'],2)
        self.assertGreater(metric['position_rmse_m'],1)

    def test_alignment_handles_different_origins_and_initial_yaw(self):
        a=np.eye(4);a[:2,:2]=[[0,-1],[1,0]];a[:3,3]=[10,20,3]
        g=np.eye(4);g[:3,3]=[-4,5,1]
        alignment=initial_alignment(a,g)
        np.testing.assert_allclose(alignment@a,g)
        b=a.copy();b[:3,3]+=a[:3,:3]@np.array([2,0,0])
        h=g.copy();h[0,3]+=2
        metric=trajectory_metrics([(0,a),(1,b)],[(0,g),(1,h)],.05)
        self.assertLess(metric['position_rmse_m'],1e-10)

    def test_rotation_interpolation_wraps_yaw(self):
        def matrix(deg):
            a=np.radians(deg);m=np.eye(4);m[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]];return m
        m=interpolate_pose(matrix(170),matrix(-170),.5)
        self.assertLess(rotation_error_degrees(m,matrix(180)),1e-5)

    def test_bad_truth_timing_is_not_silently_nearest_matched(self):
        m=trajectory_metrics([(0.5,np.eye(4))],[(0,np.eye(4)),(1,np.eye(4))],.05)
        self.assertEqual(m['matched_samples'],0)


class ContractAndCloudTests(unittest.TestCase):
    def test_vertical_scan_trial_preserves_physics_extrinsics_and_defaults(self):
        import importlib.util,tempfile,xml.etree.ElementTree as ET,yaml
        spec=importlib.util.spec_from_file_location('hover_trial_test',ROOT/'tests/run_hover_probe.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        original=(ROOT/'models/stage1_lidar_wide/stage1_lidar_wide.sdf').read_text()
        with tempfile.TemporaryDirectory() as folder:
            self.assertIsNone(module.prepare_vertical_scan_trial(ROOT,Path(folder),16))
            self.assertFalse(list(Path(folder).iterdir()))
            trial=module.prepare_vertical_scan_trial(ROOT,Path(folder),32)
            copied=(trial['models']/'stage1_lidar_wide32/stage1_lidar_wide.sdf').read_text()
            self.assertEqual(copied.replace('<samples>32</samples>','<samples>16</samples>'),original)
            configuration=yaml.safe_load(trial['config'].read_text())
            configuration['lio_sam']['N_SCAN']=16
            self.assertEqual(configuration,yaml.safe_load((ROOT/'config/stage1_uav_stable.yaml').read_text()))
            contract=load_contract(trial['entry'].parent/'contract.yaml','both','sim')
            self.assertEqual(contract['robots']['uav']['n_scan'],32)
            self.assertEqual(contract['robots']['ugv']['n_scan'],16)
            for launch in trial['entry'].parent.glob('*.launch'):ET.parse(launch)
        self.assertEqual((ROOT/'models/stage1_lidar_wide/stage1_lidar_wide.sdf').read_text(),original)

    def test_duplicate_scan_filter_does_not_retime_messages(self):
        gate=ScanGate()
        self.assertEqual([gate.accept(t) for t in [100,100,99,101,101,102]],[True,False,False,True,False,True])
        self.assertEqual((gate.forwarded,gate.duplicates,gate.backwards),(3,2,1))

    def test_real_templates_cannot_start(self):
        for name in ('ugv','uav'):
            with self.assertRaisesRegex(ValueError,'incomplete'):
                validate_hardware(ROOT/'config'/('stage1_'+name+'_real.template.yaml'))
        with self.assertRaises(ValueError):load_contract(ROOT/'config/stage1_real_contract.template.yaml',environment='real')

    def test_real_profiles_can_upload_before_calibration_guard(self):
        import xmlrpc.client
        import yaml
        for name in ('ugv','uav'):
            data=yaml.safe_load((ROOT/'config'/('stage1_'+name+'_real.template.yaml')).read_text())
            xmlrpc.client.dumps((data,),allow_none=False)

    def test_cloud_row_padding_and_endian(self):
        data=bytearray(32)
        import struct
        for row in range(2):
            for column in range(2):struct.pack_into('>f',data,row*16+column*4,row*2+column+1)
        m=NS(width=2,height=2,row_step=16,point_step=4,is_bigendian=True,data=bytes(data),fields=[NS(name='x',offset=0,datatype=7,count=1)])
        np.testing.assert_equal(cloud_array(m)['x'],[1,2,3,4])

    def test_missing_streams_and_map_review_cannot_pass(self):
        c=load_contract(ROOT/'config/stage1_contract.yaml','ugv')
        report=make_report(Observations(c,['ugv']), 'ugv','static')
        self.assertFalse(report['numeric_pass'])
        self.assertEqual(report['status'],'failed')

    def test_tf_two_parents_is_detected_even_with_one_publisher(self):
        c=load_contract(ROOT/'config/stage1_contract.yaml','ugv');o=Observations(c,['ugv'])
        o.tf_parents['base_link'].update(['odom','base_footprint']);o.tf_callers['base_link'].add('publisher')
        report=make_report(o,'ugv','smoke')
        self.assertIn('TF conflict on base_link',report['errors'])

    def test_bag_tf_caller_bytes_are_serializable(self):
        import json
        c=load_contract(ROOT/'config/stage1_contract.yaml','ugv');o=Observations(c,['ugv'])
        t=NS(child_frame_id='lidar',header=NS(frame_id='base'),
             transform=NS(rotation=NS(x=0,y=0,z=0,w=1),translation=NS(x=0,y=0,z=0)))
        o.observe('/tf_static',NS(transforms=[t]),1,b'/publisher')
        self.assertEqual(o.tf_callers['lidar'],{'/publisher'})
        json.dumps(make_report(o,'ugv','smoke'))

    def test_phase_inherits_latched_tf_without_hiding_later_conflicts(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('stage3_eval_test',ROOT/'scripts/stage3_evaluate.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        c=load_contract(ROOT/'config/stage1_contract.yaml','ugv')
        previous=Observations(c,['ugv']);phase=Observations(c,['ugv'],100)
        t=NS(child_frame_id='velodyne',header=NS(frame_id='base_link'),
             transform=NS(rotation=NS(x=0,y=0,z=0,w=1),translation=NS(x=0,y=0,z=0)))
        previous.observe('/tf_static',NS(transforms=[t]),1,b'/sensor')
        module.inherit_static_tf(phase,previous)
        self.assertEqual(phase.tf_parents['velodyne'],{'base_link'})
        # A later broadcaster is a conflict, not a replacement for old evidence.
        phase.observe('/tf_static',NS(transforms=[t]),101,b'/other_sensor')
        self.assertIn('TF conflict on velodyne',make_report(phase,'ugv','smoke')['errors'])
        self.assertEqual(previous.tf_callers['velodyne'],{'/sensor'})
        bad=NS(child_frame_id='imu',header=NS(frame_id='base_link'),
               transform=NS(rotation=NS(x=float('nan'),y=0,z=0,w=1),translation=NS(x=0,y=0,z=0)))
        previous.observe('/tf_static',NS(transforms=[bad]),2,'/bad_sensor')
        module.inherit_static_tf(phase,previous)
        self.assertIn('TF invalid quaternion: imu',phase.errors)

    def test_nan_tf_and_imu_quaternions_are_rejected(self):
        c=load_contract(ROOT/'config/stage1_contract.yaml','ugv');o=Observations(c,['ugv'])
        q=NS(x=float('nan'),y=0,z=0,w=1)
        t=NS(child_frame_id='velodyne',header=NS(frame_id='base_link'),
             transform=NS(rotation=q,translation=NS(x=0,y=0,z=0)))
        o.observe('/tf_static',NS(transforms=[t]),1,'sensor')
        imu=NS(header=NS(stamp=NS(to_sec=lambda:1),frame_id='imu'),orientation=q,
               linear_acceleration=NS(x=0,y=0,z=9.8),angular_velocity=NS(x=0,y=0,z=0))
        o.observe('/imu',imu,1,'sensor')
        self.assertTrue(any('TF invalid quaternion' in e for e in o.errors))
        self.assertTrue(any('invalid IMU orientation' in e for e in o.errors))

    def test_uav_ground_record_cannot_count_as_hover_acceptance(self):
        c=load_contract(ROOT/'config/stage1_contract.yaml','uav');o=Observations(c,['uav'])
        times=np.arange(0,60.1,.1).tolist();o.end=60
        r=c['robots']['uav']
        for kind in ('raw_cloud','imu','odometry','registered','registered_raw','deskewed','debug','ground_truth'):
            o.stamps[r[kind]]=times.copy()
        o.poses['uav']=[(t,np.eye(4)) for t in times]
        o.truth['uav']=[(t,np.eye(4)) for t in times]
        o.registration_errors['uav']=[0]
        o.tf_parents[r['base_frame']].add(r['reference_frame'])
        o.tf_parents[r['lidar_frame']].add(r['base_frame'])
        report=make_report(o,'uav','static',map_review='pass')
        self.assertEqual(report['status'],'failed')
        self.assertTrue(any('started on the ground' in e for e in report['errors']))

    def test_numeric_pass_requires_manual_review_and_smoke_is_not_acceptance(self):
        c=load_contract(ROOT/'config/stage1_contract.yaml','ugv');o=Observations(c,['ugv'])
        r=c['robots']['ugv'];times=np.arange(0,60.1,.1).tolist();o.end=60
        for kind in ('raw_cloud','imu','odometry','registered','registered_raw','deskewed','debug','ground_truth'):
            o.stamps[r[kind]]=times.copy()
        o.poses['ugv']=[(t,np.eye(4)) for t in times]
        o.truth['ugv']=[(t,np.eye(4)) for t in times]
        o.registration_errors['ugv']=[0]
        o.tf_parents['base_link'].add('odom');o.tf_callers['base_link'].add('lio')
        o.tf_parents['velodyne'].add('base_link');o.tf_callers['velodyne'].add('sensor_tf')
        report=make_report(o,'ugv','static')
        self.assertEqual(report['status'],'numeric_pass_map_review_pending',report['errors'])
        self.assertEqual(make_report(o,'ugv','static',map_review='pass')['status'],'passed')
        self.assertEqual(make_report(o,'ugv','smoke',map_review='pass')['status'],'smoke_complete')
        o.stamps[r['registered_raw']]=times[:-20]
        self.assertEqual(make_report(o,'ugv','static',map_review='pass')['status'],'failed')


class StabilityRegressionTests(unittest.TestCase):
    def test_diagnostic_air_initialization_does_not_require_missing_lio(self):
        import importlib.util,threading
        from unittest.mock import Mock,patch
        spec=importlib.util.spec_from_file_location('uav_teleop_test',ROOT/'scripts/stage1_uav_teleop.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        for require_lio in (True,False):
            t=module.Teleop.__new__(module.Teleop);t.require_lio_ready=require_lio
            t.lock=threading.RLock();t.state=NS(connected=True,mode='OFFBOARD');t.active=False;t.busy=True
            t.target=[0,0,.1,0];t.hold_current=Mock();t.mode=Mock(return_value=NS(mode_sent=True))
            t.arm=Mock(return_value=NS(success=True))
            with patch.object(module.rospy,'wait_for_message',side_effect=RuntimeError('LIO not initialized')) as wait,\
                 patch.object(module.rospy,'get_time',side_effect=[0,10]),patch.object(module.rospy,'logerr'):
                t.takeoff_worker()
            if require_lio:
                wait.assert_called_once();t.arm.assert_not_called();self.assertFalse(t.active)
            else:
                wait.assert_not_called();t.arm.assert_called_once_with(value=True)
                self.assertTrue(t.active);self.assertAlmostEqual(t.takeoff_z,1.6)
            self.assertFalse(t.busy)

    def test_unoptimized_nan_is_weak_constraint_not_solver_failure(self):
        c=load_contract(ROOT/'config/stage1_contract.yaml','ugv');o=Observations(c,['ugv'])
        m=NS(header=NS(stamp=NS(to_sec=lambda:1),frame_id='odom'),optimization_performed=False,
             weak_direction_count=0,lambda_min=float('nan'))
        o.observe(c['robots']['ugv']['debug'],m,1)
        self.assertFalse(o.errors);self.assertEqual(o.optimization_skipped['ugv'],1)
        self.assertEqual(o.debug['ugv'][0][1],6)
        m.optimization_performed=True;o.observe(c['robots']['ugv']['debug'],m,2)
        self.assertTrue(any('after optimization' in e for e in o.errors))

    def test_phase_alignment_cannot_hide_prior_drift(self):
        e0=np.eye(4);g0=np.eye(4);alignment=initial_alignment(e0,g0)
        estimated=[];truth=[]
        for stamp in (10,11):
            e=np.eye(4);e[2,3]=3;estimated.append((stamp,e));truth.append((stamp,np.eye(4)))
        self.assertAlmostEqual(trajectory_metrics(estimated,truth,.05,alignment)['position_rmse_m'],3)
        self.assertAlmostEqual(trajectory_metrics(estimated,truth,.05)['position_rmse_m'],0)

if __name__=='__main__':unittest.main()
