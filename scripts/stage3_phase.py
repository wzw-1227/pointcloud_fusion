#!/usr/bin/env python3
"""Mark a manual phase without commanding either robot."""
import argparse
import json
import time
import rospy
from std_msgs.msg import String

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('phase',choices=['hover','ugv_motion','uav_motion','height','coexist','end','separated','approach','rendezvous'])
    a=p.parse_args(rospy.myargv()[1:])
    rospy.init_node('stage3_phase',anonymous=True)
    pub=rospy.Publisher('/stage3/test_phase',String,queue_size=1)
    deadline=time.monotonic()+10
    while pub.get_num_connections()==0 and time.monotonic()<deadline and not rospy.is_shutdown():time.sleep(.1)
    if not pub.get_num_connections():p.error('No recorder subscribed to /stage3/test_phase; start a recording entry with record:=true')
    pub.publish(String(data=json.dumps({'phase':a.phase,'stamp':rospy.get_time()})))
    time.sleep(.5)
    print('Marked '+a.phase+'; controls unchanged')
if __name__=='__main__':main()
