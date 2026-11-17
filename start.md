 #运行
 roslaunch start_up stage3_gazebo_rendezvous.launch matching_profile:=partial_contour

 #无人机起飞（t 解锁，ijkl 前后左右 wsad 向上向下左右转）
 rosrun start_up stage1_uav_teleop.py _require_lio_ready:=false 