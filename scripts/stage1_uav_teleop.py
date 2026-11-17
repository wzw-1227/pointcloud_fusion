#!/usr/bin/env python3
"""Sim-only keyboard position targets; publishing never waits on terminal input."""
import math
import select
import sys
import termios
import threading
import time
import tty
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool,SetMode
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler,euler_from_quaternion


class Teleop:
    def __init__(self,require_lio_ready=True):
        if rospy.get_param('/stage1/environment','')!='sim' or not rospy.get_param('/use_sim_time',False):
            raise RuntimeError('UAV keyboard control is simulation-only')
        rospy.wait_for_service('/gazebo/get_model_state',timeout=60)
        self.require_lio_ready=bool(require_lio_ready)
        if not self.require_lio_ready and not rospy.get_param('/algorithm_test/enabled',False):
            raise RuntimeError('Skipping LIO readiness requires the explicit ideal-pose algorithm test')
        if rospy.get_param('/stage1/robot') not in ('uav','both'):raise RuntimeError('No UAV in this launch')
        self.lock=threading.RLock();self.pose=None;self.state=State();self.target=None
        self.velocity=[0.,0.,0.,0.];self.last_key=0.;self.last_tick=rospy.get_time()
        self.active=False;self.busy=False;self.takeoff_z=None;self.landing=False
        self.pub=rospy.Publisher('/iris_0/mavros/setpoint_position/local',PoseStamped,queue_size=1)
        self.subs=[rospy.Subscriber('/iris_0/mavros/local_position/pose',PoseStamped,self.pose_callback),
                   rospy.Subscriber('/iris_0/mavros/state',State,self.state_callback)]
        self.arm=rospy.ServiceProxy('/iris_0/mavros/cmd/arming',CommandBool)
        self.mode=rospy.ServiceProxy('/iris_0/mavros/set_mode',SetMode)
        self.thread=threading.Thread(target=self.publish_loop,daemon=True);self.thread.start()

    def pose_callback(self,m):
        with self.lock:self.pose=m

    def state_callback(self,m):
        with self.lock:self.state=m

    def hold_current(self):
        if self.pose is None:raise RuntimeError('Waiting for MAVROS local pose')
        p=self.pose.pose.position;q=self.pose.pose.orientation
        self.target=[p.x,p.y,p.z,euler_from_quaternion([q.x,q.y,q.z,q.w])[2]]
        self.velocity=[0.,0.,0.,0.];self.takeoff_z=None

    def request_takeoff(self):
        if self.busy or self.active:return
        self.busy=True
        threading.Thread(target=self.takeoff_worker,daemon=True).start()

    def takeoff_worker(self):
        try:
            if self.require_lio_ready:
                rospy.wait_for_message('/uav/lio_sam/mapping/odometry',Odometry,timeout=30)
            else:
                rospy.wait_for_message('/algorithm_test/uav/odometry',Odometry,timeout=30)
            with self.lock:
                if not self.state.connected:raise RuntimeError('MAVROS is not connected')
                self.hold_current();start_z=self.target[2];self.active=True;self.landing=False
            # Prestream position targets before OFFBOARD; publisher runs separately.
            deadline=rospy.get_time()+3
            wall_deadline=time.monotonic()+30
            while rospy.get_time()<deadline:
                if time.monotonic()>wall_deadline:raise RuntimeError('Simulation clock stalled during OFFBOARD prestream')
                time.sleep(.05)
            if not self.mode(custom_mode='OFFBOARD').mode_sent:raise RuntimeError('OFFBOARD request rejected')
            deadline=time.monotonic()+5
            while self.state.mode!='OFFBOARD' and time.monotonic()<deadline:time.sleep(.1)
            if self.state.mode!='OFFBOARD':raise RuntimeError('OFFBOARD not confirmed')
            if not self.arm(value=True).success:raise RuntimeError('Arming rejected')
            with self.lock:self.takeoff_z=start_z+1.5
            print('\nTakeoff: climb at 0.3 m/s to 1.5 m above initial position.',flush=True)
        except Exception as exc:
            rospy.logerr(str(exc))
            with self.lock:self.active=False
        finally:self.busy=False

    def request_land(self):
        if self.busy:return
        self.busy=True
        def worker():
            try:
                result=self.mode(custom_mode='AUTO.LAND')
                if not result.mode_sent:raise RuntimeError('AUTO.LAND rejected; position hold retained')
                deadline=time.monotonic()+5
                while self.state.mode!='AUTO.LAND' and time.monotonic()<deadline:time.sleep(.1)
                if self.state.mode!='AUTO.LAND':raise RuntimeError('AUTO.LAND not confirmed')
                with self.lock:self.landing=True;self.active=False
            except Exception as exc:rospy.logerr(str(exc))
            finally:self.busy=False
        threading.Thread(target=worker,daemon=True).start()

    def publish_loop(self):
        while not rospy.is_shutdown():
            now=time.monotonic();ros_now=rospy.get_time()
            dt=max(0,min(.1,ros_now-self.last_tick));self.last_tick=ros_now
            with self.lock:
                if self.active and self.target is not None:
                    velocity=self.velocity if now-self.last_key<.3 else [0.,0.,0.,0.]
                    if self.takeoff_z is not None and self.state.armed:
                        difference=self.takeoff_z-self.target[2]
                        self.target[2]+=max(-.3*dt,min(.3*dt,difference))
                        if abs(difference)<.02:self.takeoff_z=None
                    if self.state.mode=='OFFBOARD' and self.state.armed:
                        for i in range(4):self.target[i]+=velocity[i]*dt
                    m=PoseStamped();m.header.stamp=rospy.Time.now();m.header.frame_id='map'
                    m.pose.position.x,m.pose.position.y,m.pose.position.z=self.target[:3]
                    q=quaternion_from_euler(0,0,self.target[3])
                    m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q
                    self.pub.publish(m)
            time.sleep(.05)

    def run(self):
        if not sys.stdin.isatty():raise RuntimeError('Run UAV teleop in an interactive terminal')
        print('t: OFFBOARD + arm + takeoff; g: land; space: hold current position\n'
              'i/k: ENU +X/-X; j/l: +Y/-Y; w/s: +Z/-Z; a/d: yaw +/-; Ctrl-C: request land and exit\n'
              'Hold/repeat keys to move. Limits: 0.3 m/s, 0.3 rad/s. No automatic rearming.')
        bindings={'i':(0,.3),'k':(0,-.3),'j':(1,.3),'l':(1,-.3),'w':(2,.3),'s':(2,-.3),'a':(3,.3),'d':(3,-.3)}
        original=termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while not rospy.is_shutdown():
                if not select.select([sys.stdin],[],[],.1)[0]:continue
                key=sys.stdin.read(1)
                if key=='\x03':break
                if key=='t':self.request_takeoff()
                elif key=='g':self.request_land()
                elif key==' ':
                    with self.lock:
                        if self.pose is not None:self.hold_current()
                elif key in bindings:
                    with self.lock:
                        if self.active:
                            self.velocity=[0.,0.,0.,0.];i,v=bindings[key];self.velocity[i]=v
                            self.last_key=time.monotonic();self.takeoff_z=None
        finally:
            termios.tcsetattr(sys.stdin,termios.TCSADRAIN,original)
            if self.active and self.state.armed:
                self.request_land();time.sleep(1)


if __name__=='__main__':
    rospy.init_node('stage1_uav_teleop');Teleop(rospy.get_param('~require_lio_ready',True)).run()
