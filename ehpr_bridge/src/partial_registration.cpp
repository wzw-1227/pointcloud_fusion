#include <ehpr_bridge/partial_registration.h>
#include <ehpr_bridge/visibility.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>

namespace ehpr_bridge {
namespace {
constexpr double pi=3.14159265358979323846;
struct Pair { size_t source,target; Eigen::Vector3d point,normal; double distance; };
Eigen::Matrix3d yawRotation(double yaw) {
  return Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
std::vector<Pair> pairs(const PartialLevel& a,const PartialLevel& b,
                       const Eigen::Matrix3d& rotation,const Eigen::Vector3d& translation,
                       double radius,bool vertical_only=false,double normal_cosine=std::cos(pi/6)) {
  std::vector<std::optional<Pair>> best(b.cloud->size());
  std::vector<int> ids(8);std::vector<float> distances(8);
  for(size_t i=0;i<a.cloud->size();++i) {
    if(!a.valid[i]||(vertical_only&&!a.vertical[i]))continue;
    const Eigen::Vector3d point=rotation*(*a.cloud)[i].getVector3fMap().cast<double>()+translation;
    const Eigen::Vector3d normal=rotation*a.normals[i];
    const int found=b.tree->nearestKSearch(pcl::PointXYZ(point.x(),point.y(),point.z()),8,ids,distances);
    for(int k=0;k<found;++k) {
      if(distances[k]>radius*radius)break;
      const size_t j=ids[k];
      if(!b.valid[j]||(vertical_only&&!b.vertical[j])||std::abs(normal.dot(b.normals[j]))<normal_cosine)continue;
      if(!best[j]||distances[k]<best[j]->distance)
        best[j]=Pair{i,j,point,b.normals[j],distances[k]};
      break;
    }
  }
  std::vector<Pair> result;for(const auto& p:best)if(p)result.push_back(*p);return result;
}
double direction(double a,double b) {
  double d=std::abs(a-b);return std::min(d,pi-d);
}
int normalGroup(const Eigen::Vector3d& n) {
  if(std::abs(n.z())>=std::cos(pi/9))return 6;
  if(std::abs(n.z())>std::sin(pi/9))return 7;
  double angle=std::fmod(std::atan2(n.y(),n.x())+pi,pi);
  return std::min(5,int(angle/(pi/6)));
}
struct Equations { Eigen::Matrix4d h=Eigen::Matrix4d::Zero();Eigen::Vector4d g=Eigen::Vector4d::Zero();double scale=1,cost=0; };
Equations equations(const std::vector<Pair>& correspondences,const PartialLevel& target,
                    const Eigen::Vector3d& translation,double huber) {
  Equations e;std::array<int,8> counts{};
  double scale=0;for(const auto& p:correspondences){++counts[normalGroup(p.normal)];scale+=(p.point-translation).head<2>().squaredNorm();}
  e.scale=std::max(1.,std::sqrt(scale/std::max(size_t(1),correspondences.size())));
  for(const auto& p:correspondences) {
    const Eigen::Vector3d q=(*target.cloud)[p.target].getVector3fMap().cast<double>();
    const double residual=p.normal.dot(p.point-q),absolute=std::abs(residual);
    const double balance=1./counts[normalGroup(p.normal)];
    const double weight=balance*(absolute<=huber?1.:huber/absolute);
    const Eigen::Vector3d rotated=p.point-translation;
    Eigen::Vector4d j;j.head<3>()=p.normal;j[3]=p.normal.dot(Eigen::Vector3d(-rotated.y(),rotated.x(),0))/e.scale;
    e.h.noalias()+=weight*j*j.transpose();e.g.noalias()+=weight*j*residual;
    e.cost+=balance*(absolute<=huber?.5*residual*residual:huber*(absolute-.5*huber));
  }
  return e;
}
double fixedCost(const std::vector<Pair>& correspondences,const PartialLevel& source,
                 const PartialLevel& target,const Eigen::Matrix3d& rotation,
                 const Eigen::Vector3d& translation,double huber) {
  std::array<int,8> counts{};for(const auto& p:correspondences)++counts[normalGroup(p.normal)];
  double cost=0;for(const auto& p:correspondences) {
    const Eigen::Vector3d x=rotation*(*source.cloud)[p.source].getVector3fMap().cast<double>()+translation;
    const double r=std::abs(p.normal.dot(x-(*target.cloud)[p.target].getVector3fMap().cast<double>()));
    cost+=(r<=huber?.5*r*r:huber*(r-.5*huber))/counts[normalGroup(p.normal)];
  }return cost;
}
double coverage(const PartialLevel& a,const PartialLevel& b,const Eigen::Matrix3d& rotation,
                const Eigen::Vector3d& translation,double radius,bool vertical_only) {
  size_t count=0,total=0;std::vector<int> ids(8);std::vector<float> ds(8);
  for(size_t i=0;i<a.cloud->size();++i) {
    if(vertical_only&&(!a.valid[i]||!a.vertical[i]))continue;
    ++total;const Eigen::Vector3d x=rotation*(*a.cloud)[i].getVector3fMap().cast<double>()+translation;
    const int k=b.tree->nearestKSearch(pcl::PointXYZ(x.x(),x.y(),x.z()),vertical_only?8:1,ids,ds);
    for(int j=0;j<k&&ds[j]<=radius*radius;++j) {
      if(!vertical_only||(b.valid[ids[j]]&&b.vertical[ids[j]]&&
           std::abs((rotation*a.normals[i]).dot(b.normals[ids[j]]))>=std::cos(pi/6))){++count;break;}
    }
  }return total?double(count)/total:0;
}
}

Eigen::Quaterniond canonicalMatchingGravity(const Eigen::Matrix3d& pose) {
  const double yaw=std::atan2(pose(1,0),pose(0,0));
  return Eigen::Quaterniond(yawRotation(-yaw)*pose).normalized();
}
std::shared_ptr<PartialGeometry> preparePartialGeometry(const Cloud::Ptr& input,
                                                      const Eigen::Quaterniond& gravity,
                                                      const Eigen::Vector3d& center) {
  auto out=std::make_shared<PartialGeometry>();
  Cloud::Ptr aligned(new Cloud);aligned->reserve(input->size());
  for(const auto& p:*input){const Eigen::Vector3d g=gravity*p.getVector3fMap().cast<double>();aligned->push_back(pcl::PointXYZ(g.x(),g.y(),g.z()));}
  out->center=center;
  const std::array<float,2> sizes={.5f,.25f};
  for(size_t level=0;level<2;++level) {
    auto& v=out->levels[level];v.cloud.reset(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZ> f;f.setLeafSize(sizes[level],sizes[level],sizes[level]);f.setInputCloud(aligned);f.filter(*v.cloud);
    v.tree=std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();v.tree->setInputCloud(v.cloud);
    v.normals.resize(v.cloud->size());v.valid.resize(v.cloud->size(),false);v.vertical=v.valid;
    std::vector<int> ids(12);std::vector<float> distances(12);
    for(size_t i=0;i<v.cloud->size();++i) {
      const int n=v.tree->nearestKSearch((*v.cloud)[i],level==0?8:12,ids,distances);if(n<8)continue;
      Eigen::Vector3d center=Eigen::Vector3d::Zero();for(int k=0;k<n;++k)center+=(*v.cloud)[ids[k]].getVector3fMap().cast<double>();center/=n;
      Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();for(int k=0;k<n;++k){const Eigen::Vector3d d=(*v.cloud)[ids[k]].getVector3fMap().cast<double>()-center;covariance.noalias()+=d*d.transpose();}
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance/n);
      if(eigen.info()!=Eigen::Success||eigen.eigenvalues()[1]<1e-8||eigen.eigenvalues()[0]>.15*eigen.eigenvalues()[1])continue;
      v.normals[i]=eigen.eigenvectors().col(0);v.valid[i]=true;
      v.vertical[i]=std::abs(v.normals[i].z())<=std::sin(pi/9);
    }
  }
  // Horizontal planes constrain height even when sparse scan rings do not share
  // individual floor samples. Small object tops cannot stand in for a floor.
  const auto& fine=out->levels[1];std::vector<double> heights;
  for(size_t i=0;i<fine.cloud->size();++i)if(fine.valid[i]&&std::abs(fine.normals[i].z())>=std::cos(pi/9))heights.push_back((*fine.cloud)[i].z);
  std::sort(heights.begin(),heights.end());
  for(size_t i=0;i<heights.size();) {
    size_t j=i+1;while(j<heights.size()&&heights[j]-heights[i]<=.1)++j;
    const double height=heights[i+(j-i)/2];
    if(j-i>=20&&height<center.z()-.05)out->horizontal_planes.push_back({height,j-i});i=j;
  }return out;
}

static std::string validateWithHeight(const PartialGeometry& reference,const PartialGeometry& query,
    const MatcherConfig& c,MatchResult& m,const HorizontalPlane& floor_a,const HorizontalPlane& floor_b) {
  const Eigen::Matrix3d gr=m.reference->matching_gravity.value_or(m.reference->gravity).toRotationMatrix();
  const Eigen::Matrix3d gq=m.query->matching_gravity.value_or(m.query->gravity).toRotationMatrix();
  const Eigen::Matrix3d rotation=gq*m.refined.rotation()*gr.transpose();
  const Eigen::Vector3d translation=gq*m.refined.translation();
  const double height=floor_b.height-floor_a.height;
  m.horizontal_support_reference=floor_a.support;m.horizontal_support_query=floor_b.support;
  m.direction_support_first=m.direction_support_second=0;m.direction_separation_deg=0;m.information_ratio=0;
  const auto& a=reference.levels[1];const auto& b=query.levels[1];
  const auto all=pairs(a,b,rotation,translation,.5),structure=pairs(a,b,rotation,translation,.5,true);
  m.effective_correspondences=all.size();m.structural_correspondences=structure.size();
  m.structural_reference_points=std::count(a.vertical.begin(),a.vertical.end(),true);
  m.structural_query_points=std::count(b.vertical.begin(),b.vertical.end(),true);
  m.structural_overlap_reference=coverage(a,b,rotation,translation,.5,true);
  m.structural_overlap_query=coverage(b,a,rotation.transpose(),-rotation.transpose()*translation,.5,true);
  m.overlap_reference=coverage(a,b,rotation,translation,c.overlap_radius,false);
  m.overlap_query=coverage(b,a,rotation.transpose(),-rotation.transpose()*translation,c.overlap_radius,false);
  m.overlap=a.cloud->size()<=b.cloud->size()?m.overlap_reference:m.overlap_query;
  double sum=0;for(const auto& pair:structure)sum+=pair.distance;
  m.partial_residual_rmse=structure.empty()?1e9:std::sqrt(sum/structure.size());
  m.residual_rmse=m.partial_residual_rmse;
  // Preserve the legacy fitness field as a separate all-cloud NN diagnostic.
  double fitness=0;size_t fitness_count=0;std::vector<int> ids(1);std::vector<float> ds(1);
  for(const auto& p:*a.cloud){const Eigen::Vector3d x=rotation*p.getVector3fMap().cast<double>()+translation;
    if(b.tree->nearestKSearch(pcl::PointXYZ(x.x(),x.y(),x.z()),1,ids,ds)>0&&ds[0]<=20){fitness+=ds[0];++fitness_count;}}
  m.fitness=fitness_count?fitness/fitness_count:1e9;
  if(!all.empty()) {
    auto e=equations(all,b,translation,.15);e.h(2,2)+=1;Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen(e.h);
    if(eigen.info()==Eigen::Success&&eigen.eigenvalues()[3]>0)m.information_ratio=std::max(0.,eigen.eigenvalues()[0])/eigen.eigenvalues()[3];
  }
  Eigen::Vector2d lo=Eigen::Vector2d::Constant(1e9),hi=-lo;
  std::array<int,18> histogram{};
  for(const auto& pair:structure){lo=lo.cwiseMin(pair.point.head<2>());hi=hi.cwiseMax(pair.point.head<2>());
    double angle=std::fmod(std::atan2(pair.normal.y(),pair.normal.x())+pi,pi);
    ++histogram[std::min(17,int(angle/(pi/18)))];}
  if(!structure.empty())m.structural_span=(hi-lo).norm();
  size_t best_support=0;
  for(int i=0;i<18;++i)for(int j=i+1;j<18;++j) {
    const double separation=direction(i*pi/18,j*pi/18)*180/pi;
    if(separation+1e-8<c.partial_direction_angle_deg)continue;
    const size_t first=histogram[(i+17)%18]+histogram[i]+histogram[(i+1)%18];
    const size_t second=histogram[(j+17)%18]+histogram[j]+histogram[(j+1)%18];
    if(std::min(first,second)>best_support){best_support=std::min(first,second);m.direction_support_first=first;m.direction_support_second=second;m.direction_separation_deg=separation;}
  }
  const double high=std::max(m.structural_overlap_reference,m.structural_overlap_query);
  const double low=std::min(m.structural_overlap_reference,m.structural_overlap_query);
  m.candidate_score=high*std::sqrt(low)*std::exp(-std::pow(m.partial_residual_rmse/c.partial_rmse,2));
  m.horizontal_plane_residual=std::abs(translation.z()-height);
  if(m.horizontal_plane_residual>.1)return "horizontal plane height constraint failed";
  if(!m.refinement_converged)return "partial refinement did not converge";
  if(m.information_ratio<c.partial_information_ratio)return "partial geometry is underconstrained";
  if(m.structural_correspondences<size_t(c.partial_min_pairs))return "insufficient independent structural correspondences";
  if((!m.reference_recent||!m.query_recent)&&low<c.partial_history_low_coverage)
    return "partial historical cloud coverage gate failed";
  if(high<c.partial_high_coverage||low<c.partial_low_coverage)return "partial structural coverage gate failed";
  if(m.partial_residual_rmse>c.partial_rmse)return "partial correspondence residual gate failed";
  if(m.structural_span<c.partial_span||best_support<size_t(c.partial_direction_min_pairs))return "insufficient structural extent or directions";
  if(m.fitness>=c.max_fitness||m.overlap<c.min_overlap)return "partial full-cloud quality gate failed";
  return validateVisibility(m,c);
}

static std::string refineWithHeight(const PartialGeometry& reference,const PartialGeometry& query,
                          const MatcherConfig& c,MatchResult& m,const HorizontalPlane& floor_a,const HorizontalPlane& floor_b) {
  m.refinement_method="gravity_point_to_plane_huber_irls";m.structural_check_enabled=true;
  const Eigen::Matrix3d gr=m.reference->matching_gravity.value_or(m.reference->gravity).toRotationMatrix();
  const Eigen::Matrix3d gq=m.query->matching_gravity.value_or(m.query->gravity).toRotationMatrix();
  Eigen::Matrix3d rotation=gq*m.coarse.rotation()*gr.transpose();
  double yaw=std::atan2(rotation(1,0),rotation(0,0));rotation=yawRotation(yaw);
  Eigen::Vector3d translation=gq*m.coarse.translation();
  const double height=floor_b.height-floor_a.height;translation.z()=height;
  m.horizontal_support_reference=floor_a.support;m.horizontal_support_query=floor_b.support;
  const std::array<double,2> radii={1.5,.75},hubers={.3,.15};
  for(size_t level=0;level<2;++level) {
    const auto& a=reference.levels[level];const auto& b=query.levels[level];
    if(a.cloud->size()<20||b.cloud->size()<20)return "insufficient partial registration points";
    bool converged=false;
    for(int iteration=0;iteration<c.partial_iterations;++iteration) {
      ++m.refinement_iterations;
      const auto correspondences=pairs(a,b,rotation,translation,radii[level],false,level==0?.5:std::cos(pi/6));
      if(correspondences.size()<20)return "insufficient normal-compatible correspondences";
      auto e=equations(correspondences,b,translation,hubers[level]);
      const double height_residual=translation.z()-height;
      e.h(2,2)+=1;e.g[2]+=height_residual;e.cost+=.5*height_residual*height_residual;
      Eigen::Matrix4d damped=e.h;damped.diagonal().array()+=1e-6;
      Eigen::Vector4d step=-damped.ldlt().solve(e.g);
      if(!step.allFinite())return "invalid partial optimization step";
      double limit=std::max({1.,step.head<3>().norm()/.5,std::abs(step[3]/e.scale)/(pi/36)});step/=limit;
      double multiplier=1;bool improved=false;
      for(int trial=0;trial<8;++trial) {
        const double next_yaw=yaw+multiplier*step[3]/e.scale;
        const Eigen::Vector3d next_translation=translation+multiplier*step.head<3>();
        if(fixedCost(correspondences,a,b,yawRotation(next_yaw),next_translation,hubers[level])+
             .5*std::pow(next_translation.z()-height,2)<=e.cost+1e-12) {
          yaw=next_yaw;translation=next_translation;rotation=yawRotation(yaw);improved=true;break;
        }multiplier*=.5;
      }
      m.translation_update_m=multiplier*step.head<3>().norm();
      m.rotation_update_deg=std::abs(multiplier*step[3]/e.scale)*180/pi;
      // PCA normals at voxel boundaries cause small assignment oscillations.
      // Stop well below the 0.25 m sampling scale; independent gates still
      // decide whether this local optimum is a trustworthy registration.
      if(!improved||(m.translation_update_m<.02&&m.rotation_update_deg<.2)){converged=true;break;}
    }
    if(level==1)m.refinement_converged=converged;
  }
  m.refined.setIdentity();m.refined.linear()=gq.transpose()*rotation*gr;m.refined.translation()=gq.transpose()*translation;
  if(!properTransform(m.refined.matrix()))return "invalid partial rigid transform";
  return validateWithHeight(reference,query,c,m,floor_a,floor_b);
}

std::string refinePartial(const PartialGeometry& reference,const PartialGeometry& query,
                          const MatcherConfig& config,MatchResult& result) {
  result.refinement_method="gravity_point_to_plane_huber_irls";result.structural_check_enabled=true;
  if(reference.horizontal_planes.empty()||query.horizontal_planes.empty())
    return "insufficient horizontal plane support for height";
  const auto seed=result;std::vector<MatchResult> valid;std::string failure;double best=-1;
  for(const auto& a:reference.horizontal_planes)for(const auto& b:query.horizontal_planes) {
    auto attempt=seed;const auto reason=refineWithHeight(reference,query,config,attempt,a,b);
    if(reason.empty())valid.push_back(attempt);
    if(attempt.candidate_score>best){best=attempt.candidate_score;result=attempt;failure=reason;}
  }
  if(valid.empty())return failure;
  std::stable_sort(valid.begin(),valid.end(),[](const auto& a,const auto& b){return a.candidate_score>b.candidate_score;});
  result=valid.front();
  if(valid.size()>1&&std::abs((result.query->matching_gravity.value_or(result.query->gravity)*
       (valid[0].refined.translation()-valid[1].refined.translation())).z())>config.partial_cluster_translation&&
       valid[0].candidate_score-valid[1].candidate_score<config.partial_score_margin)
    return "ambiguous horizontal plane alignment";
  return {};
}

std::string validatePartialTransform(const PartialGeometry& reference,const PartialGeometry& query,
                                    const MatcherConfig& config,MatchResult& result) {
  const auto original=result;std::string reason="insufficient horizontal plane support for height";
  for(const auto& a:reference.horizontal_planes)for(const auto& b:query.horizontal_planes) {
    auto trial=original;auto failure=validateWithHeight(reference,query,config,trial,a,b);
    if(failure.empty()){result=std::move(trial);return {};}
    reason=failure;
  }return reason;
}

void selectPartialMatch(MatchEvent& event,const MatcherConfig& config) {
  std::vector<size_t> valid;for(size_t i=0;i<event.candidate_attempts.size();++i)
    if(event.candidate_attempts[i].reason.empty()&&event.candidate_attempts[i].result)valid.push_back(i);
  std::stable_sort(valid.begin(),valid.end(),[&](size_t a,size_t b){return event.candidate_attempts[a].result->candidate_score>event.candidate_attempts[b].result->candidate_score;});
  event.diagnostics["geometrically_valid_candidates"]=valid.size();
  if(valid.empty()){if(!event.candidate_attempts.empty())event.reason=event.candidate_attempts.front().reason;return;}
  auto mapPose=[](const MatchResult& m){return Eigen::Isometry3d(m.reference->pose*m.refined.inverse()*m.query->pose.inverse());};
  std::vector<size_t> clusters;
  for(auto i:valid){const auto pose=mapPose(*event.candidate_attempts[i].result);bool duplicate=false;
    for(auto j:clusters){const auto delta=mapPose(*event.candidate_attempts[j].result).inverse()*pose;
      if(delta.translation().norm()<=config.partial_cluster_translation&&
         Eigen::AngleAxisd(delta.rotation()).angle()*180/pi<=config.partial_cluster_rotation_deg){duplicate=true;break;}}
    if(!duplicate)clusters.push_back(i);
  }
  auto& best=*event.candidate_attempts[valid.front()].result;
  best.has_competing_cluster=clusters.size()>1;
  best.score_margin=clusters.size()>1?best.candidate_score-event.candidate_attempts[clusters[1]].result->candidate_score:0.;
  event.diagnostics["candidate_pose_clusters"]=clusters.size();event.diagnostics["candidate_score"]=best.candidate_score;
  event.diagnostics["score_margin"]=best.score_margin;event.attempted=best;
  if(best.has_competing_cluster&&best.score_margin<config.partial_score_margin){event.reason="ambiguous partial registration hypotheses";return;}
  event.match=best;event.status="matched";event.reason="EHPR and partial geometry quality gates passed";
  for(auto i:valid)if(i!=valid.front())event.candidate_attempts[i].reason="geometrically valid alternative";
  event.diagnostics["effective_correspondences"]=best.effective_correspondences;
  event.diagnostics["structural_correspondences"]=best.structural_correspondences;
  event.diagnostics["structural_overlap_reference"]=best.structural_overlap_reference;
  event.diagnostics["structural_overlap_query"]=best.structural_overlap_query;
  event.diagnostics["structural_span_m"]=best.structural_span;
  event.diagnostics["direction_separation_deg"]=best.direction_separation_deg;
  event.diagnostics["information_ratio"]=best.information_ratio;
}
}
