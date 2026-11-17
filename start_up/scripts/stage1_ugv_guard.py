#!/usr/bin/env python3
import threading
import time
import math
import rospy
from geometry_msgs.msg import Twist


def require_simulation():
    if rospy.get_param('/stage1/environment','')!='sim' or not rospy.get_param('/use_sim_time',False):
        raise RuntimeError('This controller is simulation-only')
    rospy.wait_for_service('/gazebo/get_model_state',timeout=60)


class Guard:
    def __init__(self):
        require_simulation();self.lock=threading.Lock();self.last=0;self.command=Twist()
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.sub=rospy.Subscriber('/stage1/ugv/cmd_vel_raw',Twist,self.callback,queue_size=1)
        rospy.on_shutdown(self.stop)

    def callback(self,m):
        with self.lock:self.command=m;self.last=time.monotonic()

    def stop(self):self.pub.publish(Twist())

    def run(self):
        while not rospy.is_shutdown():
            m=Twist()
            with self.lock:
                if time.monotonic()-self.last<.5:
                    if math.isfinite(self.command.linear.x) and math.isfinite(self.command.angular.z):
                        m.linear.x=max(-.3,min(.3,self.command.linear.x))
                        m.angular.z=max(-.3,min(.3,self.command.angular.z))
            self.pub.publish(m);time.sleep(.05)


if __name__=='__main__':
    rospy.init_node('stage1_ugv_guard');Guard().run()
