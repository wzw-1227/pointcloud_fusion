#include <ehpr_bridge/contour_registration.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <random>
#include <numeric>
#include <algorithm>
#include <iostream>

namespace ehpr_bridge {
namespace {
constexpr double pi=3.14159265358979323846;
Eigen::Quaterniond gravity(const SubmapInput& p){return p.matching_gravity.value_or(canonicalMatchingGravity(p.pose.rotation()));}
double median(std::vector<double> v){auto k=v.begin()+v.size()/2;std::nth_element(v.begin(),k,v.end());return *k;}
Eigen::Matrix2d rotation(double a){Eigen::Matrix2d r;r<<std::cos(a),-std::sin(a),std::sin(a),std::cos(a);return r;}
double angleDistance(double a,double b){double d=std::fmod(std::abs(a-b),pi);return std::min(d,pi-d);}
ContourLevel level(const Cloud::Ptr& points,double voxel) {
  ContourLevel out;out.cloud.reset(new Cloud);pcl::VoxelGrid<pcl::PointXYZ> f;
  f.setInputCloud(points);f.setLeafSize(voxel,voxel,voxel);f.filter(*out.cloud);
  out.tree=std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();out.tree->setInputCloud(out.cloud);
  out.valid.resize(out.cloud->size(),false);out.normals.resize(out.cloud->size());
  std::vector<int> ids;std::vector<float> ds;
  for(size_t i=0;i<out.cloud->size();++i){
    int count=out.tree->radiusSearch((*out.cloud)[i],std::max(.6,2.5*voxel),ids,ds);
    if(count<4)continue;Eigen::Vector2d mean=Eigen::Vector2d::Zero();
    for(int id:ids)mean+=(*out.cloud)[id].getVector3fMap().head<2>().cast<double>();mean/=count;
    Eigen::Matrix2d cov=Eigen::Matrix2d::Zero();for(int id:ids){Eigen::Vector2d d=(*out.cloud)[id].getVector3fMap().head<2>().cast<double>()-mean;cov+=d*d.transpose();}cov/=count;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
    if(eig.eigenvalues()[1]<.015||eig.eigenvalues()[0]>.15*eig.eigenvalues()[1])continue;
    out.normals[i]=eig.eigenvectors().col(0);out.valid[i]=true;
  }return out;
}
struct Pair{size_t source,target;Eigen::Vector2d point,normal;double distance;};
std::vector<Pair> pairs(const ContourLevel& a,const ContourLevel& b,const Eigen::Matrix2d& r,
                        const Eigen::Vector2d& t,double radius,double cosine) {
  std::vector<std::optional<Pair>> best(b.cloud->size());std::vector<int> ids(8);std::vector<float> ds(8);
  for(size_t i=0;i<a.cloud->size();++i){if(!a.valid[i])continue;
    Eigen::Vector2d p=r*(*a.cloud)[i].getVector3fMap().head<2>().cast<double>()+t,n=r*a.normals[i];
    int count=b.tree->nearestKSearch(pcl::PointXYZ(p.x(),p.y(),0),8,ids,ds);
    for(int j=0;j<count;++j){size_t k=ids[j];if(ds[j]>radius*radius)break;
      if(!b.valid[k]||std::abs(n.dot(b.normals[k]))<cosine)continue;
      if(!best[k]||ds[j]<best[k]->distance)best[k]=Pair{i,k,p,b.normals[k],ds[j]};break;
    }
  }std::vector<Pair> out;for(const auto& p:best)if(p)out.push_back(*p);return out;
}
struct Equation{Eigen::Matrix3d h=Eigen::Matrix3d::Zero();Eigen::Vector3d g=Eigen::Vector3d::Zero();double scale=1;};
Equation equation(const std::vector<Pair>& ps,const ContourLevel& b,const Eigen::Vector2d& t,double huber){
  Equation e;double s=0;for(const auto& p:ps)s+=(p.point-t).squaredNorm();e.scale=std::max(1.,std::sqrt(s/std::max(size_t(1),ps.size())));
  std::array<int,6> counts{};auto bin=[](const Eigen::Vector2d& n){double a=std::fmod(std::atan2(n.y(),n.x())+pi,pi);return std::min(5,int(a/(pi/6)));};
  for(const auto& p:ps)++counts[bin(p.normal)];
  for(const auto& p:ps){Eigen::Vector2d q=(*b.cloud)[p.target].getVector3fMap().head<2>().cast<double>();double d=p.normal.dot(p.point-q);
    double w=(std::abs(d)<=huber?1.:huber/std::abs(d))/counts[bin(p.normal)];Eigen::Vector2d v=p.point-t;
    Eigen::Vector3d j;j<<p.normal.x(),p.normal.y(),p.normal.dot(Eigen::Vector2d(-v.y(),v.x()))/e.scale;
    e.h+=w*j*j.transpose();e.g+=w*j*d;
  }return e;
}
double cost(const std::vector<Pair>& ps,const ContourLevel& a,const ContourLevel& b,
              const Eigen::Matrix2d& r,const Eigen::Vector2d& t,double huber){
  double out=0;for(const auto& p:ps){Eigen::Vector2d x=r*(*a.cloud)[p.source].getVector3fMap().head<2>().cast<double>()+t;
    Eigen::Vector2d q=(*b.cloud)[p.target].getVector3fMap().head<2>().cast<double>();double d=std::abs(p.normal.dot(x-q));
    out+=d<=huber?.5*d*d:huber*(d-.5*huber);}return out;
}
double coverage(const ContourLevel& a,const ContourLevel& b,const Eigen::Matrix2d& r,const Eigen::Vector2d& t,double radius){
  if(a.cloud->empty()||b.cloud->empty())return 0;size_t n=0;std::vector<int> ids(1);std::vector<float> ds(1);
  for(const auto& p:*a.cloud){Eigen::Vector2d x=r*p.getVector3fMap().head<2>().cast<double>()+t;
    if(b.tree->nearestKSearch(pcl::PointXYZ(x.x(),x.y(),0),1,ids,ds)&&ds[0]<=radius*radius)++n;}return double(n)/a.cloud->size();
}
}

GroundAnchor estimateHistoricalGround(const SubmapInput& input){
  GroundAnchor out;if(!input.cloud||input.cloud->size()<100||!input.observation_quality_valid)return out;
  Cloud::Ptr full(new Cloud);pcl::transformPointCloud(*input.cloud,*full,Eigen::Affine3d(input.pose));
  Cloud reduced;pcl::VoxelGrid<pcl::PointXYZ> f;f.setInputCloud(full);f.setLeafSize(.25,.25,.25);f.filter(reduced);
  if(reduced.size()<100)return out;std::mt19937 rng(0x47524f55);std::uniform_int_distribution<size_t> pick(0,reduced.size()-1);
  Cloud::Ptr candidates(new Cloud(reduced));pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(candidates);
  std::vector<bool> horizontal(reduced.size(),false);std::vector<int> ids(8);std::vector<float> distances(8);
  for(size_t i=0;i<reduced.size();++i){int count=tree.nearestKSearch(reduced[i],8,ids,distances);if(count<8)continue;
    Eigen::Vector3d mean=Eigen::Vector3d::Zero();for(int id:ids)mean+=reduced[id].getVector3fMap().cast<double>();mean/=count;
    Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();for(int id:ids){Eigen::Vector3d v=reduced[id].getVector3fMap().cast<double>()-mean;cov+=v*v.transpose();}
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    horizontal[i]=eig.eigenvalues()[1]>1e-6&&eig.eigenvalues()[0]<.15*eig.eigenvalues()[1]&&std::abs(eig.eigenvectors()(2,0))>=std::cos(5*pi/180);
  }
  std::vector<GroundAnchor> hypotheses;
  for(int trial=0;trial<1024;++trial){Eigen::Vector3d p=reduced[pick(rng)].getVector3fMap().cast<double>(),a=reduced[pick(rng)].getVector3fMap().cast<double>(),b=reduced[pick(rng)].getVector3fMap().cast<double>();
    Eigen::Vector3d n=(a-p).cross(b-p);if(n.norm()<1e-5)continue;n.normalize();if(n.z()<0)n=-n;
    if(n.z()<std::cos(5*pi/180))continue;double d=n.dot(p),h=d/n.z();if(h>-.1||h< -1.5)continue;
    std::vector<Eigen::Vector3d> points;std::vector<double> heights;
    size_t witnesses=0;for(size_t i=0;i<reduced.size();++i){Eigen::Vector3d x=reduced[i].getVector3fMap().cast<double>();if(std::abs(n.dot(x)-d)<=.04){points.push_back(x);heights.push_back(x.z());witnesses+=horizontal[i];}}
    if(points.size()<100||witnesses<20)continue;h=median(heights);double sq=0;Eigen::Vector2d mean=Eigen::Vector2d::Zero(),lo=Eigen::Vector2d::Constant(1e9),hi=-lo;
    for(const auto& x:points){sq+=(x.z()-h)*(x.z()-h);mean+=x.head<2>();lo=lo.cwiseMin(x.head<2>());hi=hi.cwiseMax(x.head<2>());}mean/=points.size();
    Eigen::Matrix2d cov=Eigen::Matrix2d::Zero();for(const auto& x:points){Eigen::Vector2d u=x.head<2>()-mean;cov+=u*u.transpose();}cov/=points.size();
    GroundAnchor v;v.height=h;v.support=points.size();v.rmse=std::sqrt(sq/points.size());v.span=(hi-lo).norm();
    v.width=std::sqrt(std::max(0.,Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(cov).eigenvalues()[0])*12);
    if(v.rmse>.03||v.span<3||v.width<1)continue;v.valid=true;v.source=input.identity();
    auto it=std::find_if(hypotheses.begin(),hypotheses.end(),[&](const auto& other){return std::abs(other.height-h)<=.05;});
    if(it==hypotheses.end())hypotheses.push_back(v);else if(v.support>it->support)*it=v;
  }
  if(hypotheses.size()>1){out.conflict=true;return out;}if(!hypotheses.empty())out=hypotheses.front();return out;
}

std::shared_ptr<ContourGeometry> prepareContour(const SubmapInput& input,const GroundAnchor& ground,const MatcherConfig& c){
  auto out=std::make_shared<ContourGeometry>();out->ground_local=ground.height-input.pose.translation().z();
  auto g=gravity(input);Eigen::Vector3d center=g*input.recent_center_local;
  out->height_above_ground=center.z()-out->ground_local;
  using Key=std::array<int64_t,2>;struct Cell{Eigen::Vector2d sum=Eigen::Vector2d::Zero();size_t count=0;};std::map<Key,Cell> cells;
  const double radius=input.robot==0?std::max(c.matching_radius_ugv,c.matching_radius_ugv_fallback):c.matching_radius_uav;
  if(input.recent_cloud)for(const auto& p:*input.recent_cloud){Eigen::Vector3d x=g*p.getVector3fMap().cast<double>();
    if(x.z()<out->ground_local+.4||(radius>0&&(x-center).head<2>().norm()>radius))continue;
    Key k{int64_t(std::floor(x.x()/c.contour_grid)),int64_t(std::floor(x.y()/c.contour_grid))};auto& cell=cells[k];cell.sum+=x.head<2>();++cell.count;}
  Cloud::Ptr points(new Cloud);for(const auto& item:cells){const auto& k=item.first;
    // Keep a one-cell observed band around the occupancy boundary. Removing
    // the middle cells of a quantized thin wall biases overlap by viewpoint.
    bool boundary=false;for(int dx=-2;dx<=2&&!boundary;++dx)for(int dy=-2;dy<=2;++dy)
      if(std::abs(dx)+std::abs(dy)<=2&&!cells.count({k[0]+dx,k[1]+dy})){boundary=true;break;}
    if(boundary){auto p=item.second.sum/item.second.count;points->push_back(pcl::PointXYZ(p.x(),p.y(),0));}}
  out->levels[0]=level(points,.5);out->levels[1]=level(points,.25);out->levels[2]=level(points,c.contour_grid);return out;
}

std::vector<ContourSeed> contourLineSeeds(const ContourGeometry& reference,const ContourGeometry& query) {
  struct Line{Eigen::Vector2d normal;double offset;size_t count;};
  auto lines=[](const ContourLevel& level){
    std::vector<Line> out;std::vector<size_t> valid;for(size_t i=0;i<level.valid.size();++i)if(level.valid[i])valid.push_back(i);
    if(valid.empty())return out;std::mt19937 rng(0x4c494e45);std::uniform_int_distribution<size_t> pick(0,valid.size()-1);
    for(int trial=0;trial<256;++trial){size_t i=valid[pick(rng)];Eigen::Vector2d n=level.normals[i];
      double d=n.dot((*level.cloud)[i].getVector3fMap().head<2>().cast<double>());std::vector<Eigen::Vector2d> pts;
      for(size_t j:valid){Eigen::Vector2d p=(*level.cloud)[j].getVector3fMap().head<2>().cast<double>();
        if(std::abs(n.dot(p)-d)<=.10&&std::abs(n.dot(level.normals[j]))>=std::cos(pi/12))pts.push_back(p);}
      if(pts.size()<8)continue;Eigen::Vector2d mean=Eigen::Vector2d::Zero();for(const auto& p:pts)mean+=p;mean/=pts.size();
      Eigen::Matrix2d cov=Eigen::Matrix2d::Zero();for(const auto& p:pts){auto a=p-mean;cov+=a*a.transpose();}
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);n=eig.eigenvectors().col(0);
      if(n.x()<0||(std::abs(n.x())<1e-9&&n.y()<0))n=-n;d=n.dot(mean);
      auto found=std::find_if(out.begin(),out.end(),[&](const auto& l){return std::abs(n.dot(l.normal))>=std::cos(pi/36)&&std::abs(d-(n.dot(l.normal)>0?l.offset:-l.offset))<=.15;});
      if(found==out.end())out.push_back({n,d,pts.size()});else if(pts.size()>found->count)*found={n,d,pts.size()};
    }
    std::stable_sort(out.begin(),out.end(),[](const auto& a,const auto& b){return a.count>b.count;});if(out.size()>8)out.resize(8);return out;
  };
  const auto& a=reference.levels[2];const auto& b=query.levels[2];auto source=lines(a),target=lines(b);std::vector<ContourSeed> out;
  for(size_t i=0;i<source.size();++i)for(size_t j=i+1;j<source.size();++j){
    if(std::abs(source[i].normal.dot(source[j].normal))>std::cos(pi/6))continue;
    for(size_t k=0;k<target.size();++k)for(size_t l=0;l<target.size();++l){if(k==l||std::abs(target[k].normal.dot(target[l].normal))>std::cos(pi/6))continue;
      for(int sign:{-1,1}){double yaw=std::atan2(sign*target[k].normal.y(),sign*target[k].normal.x())-std::atan2(source[i].normal.y(),source[i].normal.x());auto r=rotation(yaw);
        double second=(r*source[j].normal).dot(target[l].normal);if(std::abs(second)<std::cos(pi/18))continue;
        Eigen::Matrix2d n;n.row(0)=target[k].normal.transpose();n.row(1)=target[l].normal.transpose();Eigen::Vector2d d;
        d<<target[k].offset-sign*source[i].offset,target[l].offset-(second>=0?1:-1)*source[j].offset;Eigen::Vector2d t=n.inverse()*d;
        auto ps=pairs(a,b,r,t,.75,std::cos(pi/6));if(ps.size()<5)continue;
        ContourSeed seed;seed.aligned.linear()=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();seed.aligned.translation()<<t.x(),t.y(),query.ground_local-reference.ground_local;
        seed.inliers=ps.size();seed.quality=double(ps.size())/std::sqrt(std::max(size_t(1),a.cloud->size()*b.cloud->size()));
        bool duplicate=false;for(const auto& old:out){auto delta=old.aligned.inverse()*seed.aligned;
          if(delta.translation().head<2>().norm()<.25&&Eigen::AngleAxisd(delta.rotation()).angle()<3*pi/180){duplicate=true;break;}}
        if(!duplicate)out.push_back(seed);
      }
    }
  }
  std::stable_sort(out.begin(),out.end(),[](const auto& a,const auto& b){return a.quality>b.quality;});return out;
}
size_t contourSeedSupport(const ContourGeometry& a,const ContourGeometry& b,const Eigen::Isometry3d& aligned) {
  return pairs(a.levels[2],b.levels[2],aligned.rotation().topLeftCorner<2,2>(),aligned.translation().head<2>(),.75,std::cos(pi/6)).size();
}
std::shared_ptr<ContourGeometry> reflectedContour(const ContourGeometry& input) {
  auto out=std::make_shared<ContourGeometry>(input);
  for(auto& l:out->levels){l.cloud.reset(new Cloud(*l.cloud));for(auto& p:*l.cloud)p.x=-p.x;
    for(auto& normal:l.normals)normal.x()=-normal.x();
    l.tree=std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();l.tree->setInputCloud(l.cloud);}
  return out;
}

std::string validateContourTransform(const ContourGeometry& a,const ContourGeometry& b,const MatcherConfig& c,MatchResult& m) {
  const Eigen::Matrix3d gr=gravity(*m.reference).toRotationMatrix(),gq=gravity(*m.query).toRotationMatrix();
  const Eigen::Matrix3d aligned=gq*m.refined.rotation()*gr.transpose();
  const Eigen::Vector3d translation=gq*m.refined.translation();
  if(std::abs(translation.z()-(b.ground_local-a.ground_local))>.1)return "contour height constraint failed";
  const double yaw=std::atan2(aligned(1,0),aligned(0,0));const Eigen::Vector2d t=translation.head<2>();
  m.contour_information_ratio=0;m.direction_support_first=m.direction_support_second=0;m.direction_separation_deg=0;
  const auto& x=a.levels[2];const auto& y=b.levels[2];auto r=rotation(yaw);auto ps=pairs(x,y,r,t,c.contour_distance,std::cos(pi/6));
  m.contour_reference_points=x.cloud->size();m.contour_query_points=y.cloud->size();m.contour_correspondences=ps.size();m.effective_correspondences=ps.size();
  m.contour_overlap_reference=coverage(x,y,r,t,c.contour_distance);m.contour_overlap_query=coverage(y,x,r.transpose(),-r.transpose()*t,c.contour_distance);
  double sq=0;Eigen::Vector2d lo=Eigen::Vector2d::Constant(1e9),hi=-lo;std::vector<double> angles;
  for(const auto& p:ps){sq+=p.distance;lo=lo.cwiseMin(p.point);hi=hi.cwiseMax(p.point);
    Eigen::Vector2d source=r*x.normals[p.source],target=p.normal;if(source.dot(target)<0)target=-target;
    Eigen::Vector2d consensus=(source+target).normalized();
    angles.push_back(std::fmod(std::atan2(consensus.y(),consensus.x())+pi,pi));}
  m.contour_residual_rmse=ps.empty()?1e9:std::sqrt(sq/ps.size());m.structural_span=ps.empty()?0:(hi-lo).norm();
  if(!ps.empty()){auto e=equation(ps,y,t,.15);Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(e.h);
    m.contour_information_ratio=std::max(0.,eig.eigenvalues()[0])/std::max(1e-12,eig.eigenvalues()[2]);}
  size_t support=0;for(int i=0;i<18;++i)for(int j=i+1;j<18;++j){double ai=i*pi/18,aj=j*pi/18;if(angleDistance(ai,aj)+1e-10<c.partial_direction_angle_deg*pi/180)continue;
    size_t ni=0,nj=0;for(double angle:angles){ni+=angleDistance(ai,angle)<=pi/18;nj+=angleDistance(aj,angle)<=pi/18;}
    if(std::min(ni,nj)>support){support=std::min(ni,nj);m.direction_support_first=ni;m.direction_support_second=nj;m.direction_separation_deg=angleDistance(ai,aj)*180/pi;}}
  const double high=std::max(m.contour_overlap_reference,m.contour_overlap_query),low=std::min(m.contour_overlap_reference,m.contour_overlap_query);
  m.candidate_score=high*std::sqrt(low)*std::exp(-std::pow(m.contour_residual_rmse/c.contour_rmse,2));
  if(!m.refinement_converged)return "contour refinement did not converge";
  if(m.contour_information_ratio<c.partial_information_ratio)return "contour geometry is underconstrained";
  if(ps.size()<size_t(c.contour_min_pairs))return "insufficient independent contour correspondences";
  if(high<c.contour_high_coverage||low<c.contour_low_coverage)return "contour coverage gate failed";
  if(m.contour_residual_rmse>c.contour_rmse)return "contour residual gate failed";
  if(m.structural_span<c.partial_span||support<size_t(c.partial_direction_min_pairs))return "insufficient contour extent or directions";
  return {};
}

std::string refineContour(const ContourGeometry& a,const ContourGeometry& b,const GroundAnchor& ga,const GroundAnchor& gb,const MatcherConfig& c,MatchResult& m){
  m.registration_branch="contour_xy_ground";m.refinement_method="gravity_contour_point_to_line_huber_irls";
  m.ground_height_reference=ga.height;m.ground_height_query=gb.height;m.ground_support_reference=ga.support;m.ground_support_query=gb.support;
  m.ground_rmse_reference=ga.rmse;m.ground_rmse_query=gb.rmse;m.query_height_above_ground=b.height_above_ground;
  if(!ga.valid||!gb.valid||ga.conflict||gb.conflict)return "missing or conflicting historical ground anchor";
  if(b.height_above_ground<c.contour_min_height)return "below contour altitude threshold";
  Eigen::Matrix3d gr=gravity(*m.reference).toRotationMatrix(),gq=gravity(*m.query).toRotationMatrix();
  Eigen::Matrix3d init=gq*m.coarse.rotation()*gr.transpose();Eigen::Vector3d ti=gq*m.coarse.translation();
  double yaw=std::atan2(init(1,0),init(0,0));Eigen::Vector2d t=ti.head<2>();bool converged=false;
  for(int l=0;l<2;++l){const auto& x=a.levels[l];const auto& y=b.levels[l];double radius=l?.75:1.5,huber=l?.15:.3;converged=false;
    if(x.cloud->size()<20||y.cloud->size()<20)return "insufficient contour points";
    for(int iteration=0;iteration<c.partial_iterations;++iteration){++m.refinement_iterations;auto ps=pairs(x,y,rotation(yaw),t,radius,l?std::cos(pi/6):.5);
      if(ps.size()<10)return "insufficient contour correspondences";auto e=equation(ps,y,t,huber);
      Eigen::Vector3d delta=-(e.h+1e-6*Eigen::Matrix3d::Identity()).ldlt().solve(e.g);delta[2]/=e.scale;
      if(!delta.allFinite())return "nonfinite contour update";double norm=delta.head<2>().norm();if(norm>.5)delta.head<2>()*=.5/norm;delta[2]=std::clamp(delta[2],-pi/36,pi/36);
      double old=cost(ps,x,y,rotation(yaw),t,huber),step=1;bool moved=false;
      for(int back=0;back<8;++back){Eigen::Vector2d nt=t+step*delta.head<2>();double ny=yaw+step*delta[2];
        if(cost(ps,x,y,rotation(ny),nt,huber)<=old+1e-12){t=nt;yaw=ny;moved=true;break;}step*=.5;}
      m.translation_update_m=(step*delta.head<2>()).norm();m.rotation_update_deg=std::abs(step*delta[2])*180/pi;
      if(!moved||(m.translation_update_m<.02&&m.rotation_update_deg<.2)){converged=true;break;}
    }
  }
  m.refinement_converged=converged;
  Eigen::Isometry3d aligned=Eigen::Isometry3d::Identity();aligned.linear()=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  aligned.translation()<<t.x(),t.y(),b.ground_local-a.ground_local;
  m.refined.linear()=gq.transpose()*aligned.rotation()*gr;m.refined.translation()=gq.transpose()*aligned.translation();
  return validateContourTransform(a,b,c,m);
}
}
