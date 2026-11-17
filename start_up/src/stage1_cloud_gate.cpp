#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <mutex>

class ScanGate {
  ros::NodeHandle nh_, private_{"~"};
  ros::Publisher cloud_, status_;
  ros::Subscriber input_;
  ros::WallTimer timer_;
  std::mutex mutex_;
  uint64_t last_=0, forwarded_=0, duplicates_=0, backwards_=0;
  bool initialized_=false;
  double max_latency_=0, last_clock_=-1;
  uint64_t clock_resets_=0, future_drops_=0;
 public:
  ScanGate() {
    std::string environment, robot, contract;
    nh_.param<std::string>("/stage1/environment",environment,"");
    if(environment!="sim")throw std::runtime_error("Scan gate requires simulation");
    private_.getParam("robot",robot);nh_.getParam("/stage1/contract_file",contract);
    const auto r=YAML::LoadFile(contract)["robots"][robot];
    int queue=20;private_.param("queue_size",queue,20);
    if(queue<1)throw std::runtime_error("Invalid scan gate queue");
    cloud_=nh_.advertise<sensor_msgs::PointCloud2>(r["lio_input_cloud"].as<std::string>(),queue);
    status_=nh_.advertise<std_msgs::String>("/stage1/"+robot+"/lidar_input_stats",1,true);
    boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)> callback=
      [this](const sensor_msgs::PointCloud2::ConstPtr& m){
        const uint64_t stamp=m->header.stamp.toNSec();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const double now=ros::Time::now().toSec();
          if(last_clock_>=0&&now<last_clock_){initialized_=false;++clock_resets_;}
          last_clock_=now;
          if(m->header.stamp.toSec()>now+.05){++future_drops_;return;}
          max_latency_=std::max(max_latency_,now-m->header.stamp.toSec());
          if(initialized_&&stamp<=last_){if(stamp==last_)++duplicates_;else ++backwards_;return;}
          initialized_=true;last_=stamp;++forwarded_;
        }
        cloud_.publish(m);
      };
    input_=nh_.subscribe<sensor_msgs::PointCloud2>(r["raw_cloud"].as<std::string>(),queue,callback,
      ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
    timer_=nh_.createWallTimer(ros::WallDuration(1),[this](const ros::WallTimerEvent&){
      std_msgs::String m;std::ostringstream s;
      {std::lock_guard<std::mutex> lock(mutex_);
       s<<"{\"stamp\":"<<ros::Time::now().toSec()<<",\"forwarded\":"<<forwarded_
        <<",\"duplicates\":"<<duplicates_<<",\"backwards\":"<<backwards_
        <<",\"clock_resets\":"<<clock_resets_<<",\"future_drops\":"<<future_drops_
        <<",\"max_receive_latency_sim_s\":"<<max_latency_<<"}";}
      m.data=s.str();status_.publish(m);
    });
  }
};
int main(int argc,char**argv){ros::init(argc,argv,"stage1_cloud_gate");try{ScanGate gate;ros::spin();}
catch(const std::exception& e){ROS_FATAL("%s",e.what());return 1;}return 0;}
