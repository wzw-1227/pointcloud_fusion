#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>
#include <memory>

namespace gazebo {
class StampedTruth : public WorldPlugin {
  struct Robot {std::string model,frame;ignition::math::Vector3d offset;ros::Publisher pub;};
  physics::WorldPtr world_;event::ConnectionPtr update_;
  std::unique_ptr<ros::NodeHandle> nh_;
  std::vector<Robot> robots_;double last_=-1;
 public:
  void Load(physics::WorldPtr world,sdf::ElementPtr) override {
    if(!ros::isInitialized()){gzerr<<"Stamped truth requires gazebo_ros initialization\n";return;}
    nh_.reset(new ros::NodeHandle);std::string source,contract,selection;
    nh_->param<std::string>("/stage1/ground_truth_source",source,"receipt");
    if(source!="stamped")return;
    nh_->getParam("/stage1/contract_file",contract);nh_->param<std::string>("/stage1/robot",selection,"both");
    try {
      const auto config=YAML::LoadFile(contract)["robots"];
      for(const std::string name:{"ugv","uav"}) {
        if(selection!="both"&&selection!=name)continue;
        const auto r=config[name];const auto xyz=r["model_to_lidar_xyz"];
        Robot robot;robot.model=r["model_name"].as<std::string>();robot.frame=r["lidar_frame"].as<std::string>();
        robot.offset=ignition::math::Vector3d(xyz[0].as<double>(),xyz[1].as<double>(),xyz[2].as<double>());
        robot.pub=nh_->advertise<nav_msgs::Odometry>(r["ground_truth"].as<std::string>(),200);
        robots_.push_back(std::move(robot));
      }
    }catch(const std::exception& e){gzerr<<"Stamped truth configuration: "<<e.what()<<"\n";return;}
    world_=world;nh_->setParam("/stage1/stamped_truth_loaded",true);
    update_=event::Events::ConnectWorldUpdateEnd(std::bind(&StampedTruth::publish,this));
  }
  void publish(){
    const auto time=world_->SimTime();const double stamp=time.Double();
    if(stamp<last_)last_=-1;
    if(stamp-last_<.009999)return;last_=stamp;
    for(auto& r:robots_){auto model=world_->ModelByName(r.model);if(!model)continue;
      const auto pose=model->WorldPose();const auto xyz=pose.Pos()+pose.Rot().RotateVector(r.offset);
      nav_msgs::Odometry m;m.header.stamp=ros::Time(time.sec,time.nsec);m.header.frame_id="world";m.child_frame_id=r.frame;
      m.pose.pose.position.x=xyz.X();m.pose.pose.position.y=xyz.Y();m.pose.pose.position.z=xyz.Z();
      m.pose.pose.orientation.w=pose.Rot().W();m.pose.pose.orientation.x=pose.Rot().X();
      m.pose.pose.orientation.y=pose.Rot().Y();m.pose.pose.orientation.z=pose.Rot().Z();r.pub.publish(m);
    }
  }
};
GZ_REGISTER_WORLD_PLUGIN(StampedTruth)
}
