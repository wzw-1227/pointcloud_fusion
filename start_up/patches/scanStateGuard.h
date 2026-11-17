#pragma once
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <cmath>
namespace lio_state_guard {
inline bool finitePose(const float* pose) {
    for (int i=0; i<6; ++i) if (!std::isfinite(pose[i])) return false;
    return true;
}
inline bool solve(const cv::Mat& hessian, const cv::Mat& rhs, cv::Mat& output) {
    if (hessian.rows!=6 || hessian.cols!=6 || rhs.rows!=6 || rhs.cols!=1 ||
        !cv::checkRange(hessian) || !cv::checkRange(rhs)) return false;
    try {
        return cv::solve(hessian, rhs, output, cv::DECOMP_QR) &&
               output.rows==6 && output.cols==1 && cv::checkRange(output);
    } catch (const cv::Exception&) { return false; }
}
// Front/back belong to the same scan, including prediction-only scans.
inline Eigen::Affine3f increment(const Eigen::Affine3f& front, const Eigen::Affine3f& back) {
    return front.inverse()*back;
}
}
