#!/usr/bin/env python3
import json
import threading
import time
import rospy
import rosnode
from std_msgs.msg import String
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from lio_sam.msg import Scan2MapDebug
from stage1_tools.contracts import load_contract,selected_robots
from stage1_tools.evaluation import Observations


class Monitor:
    def __init__(self):
        self.selection=rospy.get_param('/stage1/robot');self.environment=rospy.get_param('/stage1/environment')
        self.contract=load_contract(rospy.get_param('/stage1/contract_file'),self.selection,self.environment)
        self.robots=selected_robots(self.selection)
        self.obs=Observations(self.contract,self.robots)
        self.lock=threading.Lock();self.started=time.monotonic();self.first_sim_time=None
        self.pub=rospy.Publisher('/stage1/health',String,queue_size=5,latch=True)
        self.subscribers=[]
        types={'raw_cloud':PointCloud2,'imu':Imu,'odometry':Odometry,'registered':PointCloud2,
               'registered_raw':PointCloud2,'deskewed':PointCloud2,'debug':Scan2MapDebug}
        for name in self.robots:
            for kind,typ in types.items():
                topic=self.contract['robots'][name][kind]
                self.subscribers.append(rospy.Subscriber(topic,typ,self.callback,callback_args=topic,queue_size=2))
        for topic in ('/tf','/tf_static'):
            self.subscribers.append(rospy.Subscriber(topic,TFMessage,self.callback,callback_args=topic,queue_size=50))

    def callback(self,message,topic):
        with self.lock:
            now=rospy.get_time()
            if self.first_sim_time is None and now>0:
                self.first_sim_time=now
                self.obs.cutoff=now+self.contract['acceptance']['warmup_seconds']
            self.obs.observe(topic,message,now,message._connection_header.get('callerid','unknown'))

    def run(self):
        while not rospy.is_shutdown():
            errors=set();summary={};now=rospy.get_time()
            initializing=self.first_sim_time is None or now-self.first_sim_time<30
            with self.lock:
                for name in self.robots:
                    r=self.contract['robots'][name];summary[name]={}
                    for kind in ('raw_cloud','imu','odometry','registered','registered_raw','debug'):
                        stamps=self.obs.stamps[r[kind]]
                        summary[name][kind]={'messages':len(stamps),'age':now-stamps[-1] if stamps else None}
                        if not initializing and (not stamps or now-stamps[-1]>1):errors.add(name+': stale/missing '+kind)
                    if not initializing and (r['base_frame'] not in self.obs.tf_parents or r['lidar_frame'] not in self.obs.tf_parents):
                        errors.add(name+': missing base/lidar TF')
                if not initializing:errors.update(self.obs.errors)
                for child,parents in self.obs.tf_parents.items():
                    if len(parents)>1 or len(self.obs.tf_callers[child])>1:errors.add('TF conflict on '+child)
                if not initializing:
                    nodes=set(rosnode.get_node_names())
                    for name in self.robots:
                        prefix='' if name=='ugv' else '/uav'
                        for suffix in ('imuPreintegration','imageProjection','featureExtraction','mapOptmization'):
                            node=prefix+'/lio_sam_'+suffix
                            if node not in nodes:errors.add('missing node '+node)
                health={'stamp':now,'status':'initializing' if initializing else 'error' if errors else 'ready',
                        'errors':sorted(errors),'warnings':sorted(self.obs.warnings),'streams':summary}
            self.pub.publish(String(data=json.dumps(health,allow_nan=False)))
            time.sleep(1)


if __name__=='__main__':
    rospy.init_node('stage1_monitor')
    try:Monitor().run()
    except (ValueError,TypeError,KeyError) as exc:
        rospy.logfatal(str(exc));raise SystemExit(1)
