#include "../patches/scanStateGuard.h"
#include <cassert>
#include <limits>
int main() {
    cv::Mat h=cv::Mat::eye(6,6,CV_32F),rhs=cv::Mat::ones(6,1,CV_32F),out;
    assert(lio_state_guard::solve(h,rhs,out));assert(cv::norm(out-rhs)<1e-6);
    h.at<float>(0,0)=std::numeric_limits<float>::quiet_NaN();
    assert(!lio_state_guard::solve(h,rhs,out));
    h=cv::Mat::zeros(6,6,CV_32F);assert(!lio_state_guard::solve(h,rhs,out));
    float pose[6]={};assert(lio_state_guard::finitePose(pose));
    pose[5]=std::numeric_limits<float>::infinity();assert(!lio_state_guard::finitePose(pose));
    Eigen::Affine3f accumulated=Eigen::Affine3f::Identity();
    // Repeated prediction-only scans must advance by their own increments,
    // not reuse a fixed historical back transform and run away backwards.
    for (int i=0;i<20;++i) {
        Eigen::Affine3f front=Eigen::Affine3f::Identity(),back=front;
        front.translation().x()=float(i)*.1f;back.translation().x()=float(i+1)*.1f;
        accumulated=accumulated*lio_state_guard::increment(front,back);
    }
    assert(std::abs(accumulated.translation().x()-2.f)<1e-5);
}
