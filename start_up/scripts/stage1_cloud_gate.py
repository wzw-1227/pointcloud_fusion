#!/usr/bin/env python3
import json
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from stage1_tools.contracts import load_contract
from stage1_tools.scan_gate import ScanGate

class CloudGate:
    def __init__(self):
        if rospy.get_param('/stage1/environment')!='sim':raise RuntimeError('GPU duplicate scan gate is simulation-only')
        name=rospy.get_param('~robot')
        r=load_contract(rospy.get_param('/stage1/contract_file'),name)['robots'][name]
        self.gate=ScanGate()
        self.pub=rospy.Publisher(r['lio_input_cloud'],PointCloud2,queue_size=2)
        self.stats=rospy.Publisher('/stage1/'+name+'/lidar_input_stats',String,queue_size=1,latch=True)
        self.sub=rospy.Subscriber(r['raw_cloud'],PointCloud2,self.callback,queue_size=2)

    def callback(self,m):
        if self.gate.accept(m.header.stamp.to_nsec()):self.pub.publish(m)
        else:rospy.logwarn_throttle(5,'Dropped duplicate/out-of-order simulated GPU scan; raw sensor topic remains recorded')
        self.stats.publish(String(data=json.dumps({'stamp':rospy.get_time(),'forwarded':self.gate.forwarded,
                                                  'duplicates':self.gate.duplicates,'backwards':self.gate.backwards})))

if __name__=='__main__':
    rospy.init_node('stage1_cloud_gate');node=CloudGate();rospy.spin()
