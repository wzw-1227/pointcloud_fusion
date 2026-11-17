#!/usr/bin/env python3
"""ModelStates has no header: timestamp receipt with /clock, at at most 50 Hz."""
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from stage1_tools.contracts import load_contract,selected_robots
from stage1_tools.geometry import quaternion_rotation


class GroundTruth:
    def __init__(self):
        self.robots=selected_robots(rospy.get_param('/stage1/robot'))
        self.contract=load_contract(rospy.get_param('/stage1/contract_file'))
        self.last=-1
        self.publishers={n:rospy.Publisher(self.contract['robots'][n]['ground_truth'],Odometry,queue_size=100) for n in self.robots}
        self.sub=rospy.Subscriber('/gazebo/model_states',ModelStates,self.callback,queue_size=1)

    def callback(self,message):
        now=rospy.Time.now()
        if now.to_sec()-self.last<.019:return
        self.last=now.to_sec()
        for name in self.robots:
            r=self.contract['robots'][name]
            if r['model_name'] not in message.name:continue
            p=message.pose[message.name.index(r['model_name'])];q=p.orientation
            rotation=quaternion_rotation([q.x,q.y,q.z,q.w])
            xyz=rotation@r['model_to_lidar_xyz']+[p.position.x,p.position.y,p.position.z]
            m=Odometry();m.header.stamp=now;m.header.frame_id='world';m.child_frame_id=r['lidar_frame']
            m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=xyz
            m.pose.pose.orientation=q
            self.publishers[name].publish(m)


if __name__=='__main__':
    rospy.init_node('stage1_ground_truth');node=GroundTruth();rospy.spin()
