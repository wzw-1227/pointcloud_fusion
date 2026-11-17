#include <ehpr_bridge/visibility.h>
#include <ehpr_bridge/partial_registration.h>
#include <cmath>
#include <algorithm>
#include <limits>

namespace ehpr_bridge {
namespace {
constexpr double pi=3.14159265358979323846;
VisibilityDirection direction(const VisibilityScan& source,const VisibilityScan& target,
                              const Eigen::Isometry3d& transform,const MatcherConfig& c) {
  struct Return {double angle=std::numeric_limits<double>::infinity(),depth=0;bool structure=false;};
  std::vector<Return> best(target.points->size());std::vector<int> ids(1);std::vector<float> ds(1);
  const double limit=std::pow(2*std::sin(c.visibility_angle_deg*pi/360),2);
  for(size_t i=0;i<source.points->size();++i) {
    const Eigen::Vector3d ray=transform*(*source.points)[i].getVector3fMap().cast<double>()-target.center;
    const double depth=ray.norm();if(depth<.5)continue;const auto unit=ray/depth;
    if(target.tree->nearestKSearch(pcl::PointXYZ(unit.x(),unit.y(),unit.z()),1,ids,ds)&&ds[0]<=limit) {
      auto& hit=best[ids[0]];
      if(ds[0]<hit.angle-1e-12||(std::abs(ds[0]-hit.angle)<=1e-12&&depth<hit.depth))
        hit={ds[0],depth,source.structure[i]};
    }
  }
  VisibilityDirection out;
  for(size_t i=0;i<best.size();++i)if(std::isfinite(best[i].angle)) {
    const bool conflict=best[i].depth+c.visibility_depth_m<target.depths[i];
    ++out.checked;out.conflicts+=conflict;
    // Count a source wall even if a false alignment puts it before a ground return.
    if(best[i].structure||target.structure[i]){++out.structure_checked;out.structure_conflicts+=conflict;}
  }
  out.sufficient=out.checked>=size_t(c.visibility_min_rays)&&out.structure_checked>=size_t(c.visibility_min_structure_rays);
  return out;
}
}
std::shared_ptr<const VisibilityScan> prepareVisibilityScan(const SubmapInput& input,double floor,const MatcherConfig& c) {
  if(!input.latest_scan||input.latest_scan->empty()||!input.recent_frames||!std::isfinite(floor))return {};
  auto out=std::make_shared<VisibilityScan>();out->floor_local=floor;out->center=input.recent_center_local;
  out->points.reset(new Cloud);out->directions.reset(new Cloud);
  // Keep an actual first return in each angular cell, never a voxel centroid.
  using Key=std::pair<int,int>;std::map<Key,std::pair<double,pcl::PointXYZ>> bins;
  const auto gravity=input.matching_gravity.value_or(canonicalMatchingGravity(input.pose.rotation()));
  const double step=c.visibility_angle_deg*pi/180;
  for(const auto& p:*input.latest_scan) {
    const Eigen::Vector3d ray=p.getVector3fMap().cast<double>()-out->center;const double depth=ray.norm();
    if(!ray.allFinite()||depth<.5)continue;
    double azimuth=std::atan2(ray.y(),ray.x())+pi;if(azimuth>=2*pi)azimuth=0;
    Key k{int(std::floor(azimuth/step)),int(std::floor((std::asin(std::clamp(ray.z()/depth,-1.,1.))+pi/2)/step))};
    auto it=bins.find(k);if(it==bins.end()||depth<it->second.first)bins[k]={depth,p};
  }
  for(const auto& entry:bins){const auto& p=entry.second.second;const double depth=entry.second.first;
    Eigen::Vector3d unit=(p.getVector3fMap().cast<double>()-out->center)/depth;
    out->points->push_back(p);out->directions->push_back(pcl::PointXYZ(unit.x(),unit.y(),unit.z()));out->depths.push_back(depth);
    out->structure.push_back((gravity*p.getVector3fMap().cast<double>()).z()>=floor+c.structural_height_above_floor);
  }
  if(out->points->empty())return {};
  out->tree=std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();out->tree->setInputCloud(out->directions);return out;
}
VisibilityReport checkVisibility(const SubmapInput& r,const SubmapInput& q,const Eigen::Isometry3d& transform,const MatcherConfig& c) {
  VisibilityReport out;out.reference_stamp=r.recent_end_stamp;out.query_stamp=q.recent_end_stamp;
  if(!r.visibility_scan||!q.visibility_scan)return out;
  out.available=true;out.source="bidirectional_single_scan";
  out.ugv_in_uav=direction(*r.visibility_scan,*q.visibility_scan,transform,c);
  out.uav_in_ugv=direction(*q.visibility_scan,*r.visibility_scan,transform.inverse(),c);
  out.sufficient=out.ugv_in_uav.sufficient&&out.uav_in_ugv.sufficient;
  auto bad=[&](const VisibilityDirection& v){return v.sufficient&&
    (double(v.conflicts)>c.visibility_max_conflict*v.checked||double(v.structure_conflicts)>c.visibility_max_conflict*v.structure_checked);};
  out.conflict=bad(out.ugv_in_uav)||bad(out.uav_in_ugv);
  out.reason=out.conflict?"bidirectional single-scan free-space contradiction gate failed":
    (out.sufficient?"":"insufficient visibility evidence");return out;
}
std::string validateVisibility(MatchResult& m,const MatcherConfig& c) {
  m.visibility=checkVisibility(*m.reference,*m.query,m.refined,c);
  m.visibility_check_enabled=m.visibility.available;
  m.visibility_checked_rays=m.visibility.ugv_in_uav.checked+m.visibility.uav_in_ugv.checked;
  m.visibility_contradictions=m.visibility.ugv_in_uav.conflicts+m.visibility.uav_in_ugv.conflicts;
  // Missing evidence is retained as a geometry candidate, never a confirmed relation.
  return m.visibility.conflict?m.visibility.reason:std::string();
}
Eigen::Isometry3d mapTransform(const MatchResult& m){return m.reference->pose*m.refined.inverse()*m.query->pose.inverse();}
bool closeAlignment(const Eigen::Isometry3d& a,const Eigen::Isometry3d& b,const MatcherConfig& c){const auto d=a.inverse()*b;
  return d.translation().norm()<=c.partial_cluster_translation&&Eigen::AngleAxisd(d.rotation()).angle()<=c.partial_cluster_rotation_deg*pi/180;}
}
