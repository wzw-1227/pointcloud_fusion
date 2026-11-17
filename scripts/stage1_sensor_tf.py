#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from stage1_tools.contracts import load_contract,validate_hardware


def transform(parent,child,xyz,rpy=(0,0,0)):
    t=TransformStamped();t.header.stamp=rospy.Time.now()
    t.header.frame_id=parent;t.child_frame_id=child
    t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=xyz
    q=quaternion_from_euler(*rpy)
    t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=q
    return t


def main():
    rospy.init_node('stage1_sensor_tf')
    robot=rospy.get_param('~robot');environment=rospy.get_param('/stage1/environment')
    contract=load_contract(rospy.get_param('/stage1/contract_file'),robot,environment)
    r=contract['robots'][robot];messages=[]
    if environment=='real':
        h=validate_hardware(rospy.get_param('~hardware_config'))['stage1_hardware']
        messages.append(transform(r['base_frame'],r['lidar_frame'],h['lidar_to_base_xyz'],h['lidar_to_base_rpy']))
        messages.append(transform(r['base_frame'],r['imu_tf_frame'],h['imu_to_base_xyz'],h['imu_to_base_rpy']))
    else:
        if r['publish_lidar_tf']:
            messages.append(transform(r['base_frame'],r['lidar_frame'],r['base_to_lidar_xyz']))
        messages.append(transform(r['base_frame'],r['imu_tf_frame'],r['base_to_imu_xyz']))
        if robot=='ugv':messages.append(transform('base_link','base_footprint',[0,0,-.18]))
    broadcaster=tf2_ros.StaticTransformBroadcaster();broadcaster.sendTransform(messages)
    rospy.spin()


if __name__=='__main__':
    try:main()
    except (ValueError,KeyError,TypeError) as exc:
        rospy.logfatal(str(exc));raise SystemExit(1)
