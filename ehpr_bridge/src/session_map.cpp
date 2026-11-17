#include <ehpr_bridge/session_map.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ehpr_bridge {
SessionMap::Key SessionMap::key(const Eigen::Vector3d& p) const {
  Key out;for(int j=0;j<3;++j){double x=std::floor(p[j]/voxel_);
    if(!std::isfinite(x)||x<=double(std::numeric_limits<int64_t>::min())||x>=double(std::numeric_limits<int64_t>::max()))
      throw std::invalid_argument("Invalid cumulative map coordinate");out[j]=int64_t(x);}return out;
}
bool SessionMap::add(SubmapPtr input) {
  if(!input||!input->cloud||!properTransform(input->pose.matrix()))return false;
  if(!seen_.insert(input->identity()).second)return false;
  bool changed=false;auto& points=points_.at(input->robot);
  for(const auto& p:*input->cloud) {
    Eigen::Vector3d v=(input->pose*p.getVector3fMap().cast<double>()).cast<float>().cast<double>();const auto k=key(v);
    Eigen::Vector3d center;for(int j=0;j<3;++j)center[j]=(double(k[j])+.5)*voxel_;
    auto it=points.find(k);bool replace=it==points.end();
    if(!replace){double d=(v-center).squaredNorm()-(it->second-center).squaredNorm();
      replace=d < -1e-12 || (std::abs(d)<=1e-12&&std::make_tuple(v.x(),v.y(),v.z())<
          std::make_tuple(it->second.x(),it->second.y(),it->second.z()));}
    if(replace){points[k]=v;changed=true;}
  }if(changed)++revision_;return changed;
}
bool SessionMap::accept(const MatchResult& m,bool replace_verified) {
  const Eigen::Isometry3d t=m.reference->pose*m.refined.inverse()*m.query->pose.inverse();
  if(!properTransform(t.matrix()))throw std::invalid_argument("Invalid map alignment");
  if(alignment_.valid&&!replace_verified) {
    auto d=alignment_.transform.inverse()*t;
    if(d.translation().norm()>.25||Eigen::AngleAxisd(d.rotation()).angle()>3*M_PI/180){
      alignment_.reason="alignment_conflict";return false;}
  }
  if(replace_verified)constraints_.clear();
  if(!constraints_.insert(m.reference->identity()+"|"+m.query->identity()).second)return true;
  alignment_.valid=true;alignment_.transform=t;++alignment_.revision;
  auto light=[](SubmapPtr p){auto out=std::make_shared<SubmapInput>(*p);out->cloud.reset();out->recent_cloud.reset();out->latest_scan.reset();out->visibility_scan.reset();return out;};
  alignment_.reference=light(m.reference);alignment_.query=light(m.query);
  alignment_.reason="verified_map_alignment";++revision_;return true;
}
void SessionMap::invalidateAlignment(const std::string& reason) {
  constraints_.clear();alignment_.valid=false;alignment_.reason=reason;++alignment_.revision;++revision_;
}
void SessionMap::reset(const std::string& reason) {
  for(auto& p:points_)p.clear();seen_.clear();constraints_.clear();
  uint64_t old=alignment_.revision;alignment_=MapAlignmentState();alignment_.revision=old+1;alignment_.reason=reason;++revision_;
}
void SessionMap::resetRobot(int robot,const std::string& reason) {
  points_.at(robot).clear();const auto prefix=std::to_string(robot)+":";
  for(auto it=seen_.begin();it!=seen_.end();)if(it->rfind(prefix,0)==0)it=seen_.erase(it);else ++it;
  constraints_.clear();uint64_t old=alignment_.revision;alignment_=MapAlignmentState();
  alignment_.revision=old+1;alignment_.reason=reason;++revision_;
}
Cloud SessionMap::cloud(int robot) const {
  Cloud out;out.reserve(points_.at(robot).size());for(const auto& item:points_.at(robot)){
    const auto& p=item.second;out.push_back(pcl::PointXYZ(p.x(),p.y(),p.z()));}return out;
}
FusedSubmap SessionMap::fused() const {
  FusedSubmap out;if(!alignment_.valid)return out;
  struct Cell {Eigen::Vector3d sum=Eigen::Vector3d::Zero();size_t count=0;unsigned mask=0;};
  std::map<Key,Cell> cells;
  for(int robot=0;robot<2;++robot)for(const auto& item:points_[robot]){
    Eigen::Vector3d p=robot?(alignment_.transform*item.second).cast<float>().cast<double>():item.second;auto& c=cells[key(p)];
    c.sum+=p;++c.count;c.mask|=1u<<robot;
  }
  out.cloud.reserve(cells.size());for(const auto& item:cells){const auto& c=item.second;auto p=c.sum/c.count;
    pcl::PointXYZRGB v;v.x=p.x();v.y=p.y();v.z=p.z();v.a=255;
    if(c.mask==1){v.r=40;v.g=220;v.b=70;++out.ugv_only_voxels;}
    else if(c.mask==2){v.r=230;v.g=70;v.b=220;++out.uav_only_voxels;}
    else{v.r=255;v.g=200;v.b=40;++out.shared_voxels;}out.cloud.push_back(v);
  }out.cloud.is_dense=true;return out;
}
}
