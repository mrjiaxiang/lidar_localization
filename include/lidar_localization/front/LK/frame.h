#pragma once

#include <mutex>
#include <opencv2/opencv.hpp>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "lidar_localization/camera_model/camera_models/Camera.h"
#include "lidar_localization/parameters/parameters.h"

namespace lidar_localization {
class Frame {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;
    Frame();
    Frame(const Frame &frame);
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight,
          const double &timeStamp);

  private:
    // Current and Next Frame id.
    static long unsigned int n_next_id_;
    long unsigned int n_id_;
};
} // namespace lidar_localization
