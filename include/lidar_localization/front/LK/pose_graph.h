#pragma once

#include <deque>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

// #include <opencv2/core/eigen.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "lidar_localization/front/LK/feature_tracker.h"
#include "lidar_localization/front/LK/visualization.h"

using namespace std;
using namespace Eigen;

namespace lidar_localization {
class Estimator {
  public:
    Estimator();
    ~Estimator();

    void inputImage(double t, const cv::Mat &_img,
                    const cv::Mat &_img1 = cv::Mat());

    void setParameter();

  private:
    std::thread track_thread_;
    int input_image_cnt_;

    bool init_thread_flag_;

    FeatureTracker feature_tracker_;

    std::mutex process_mtx_;

    Matrix3d ric_[2];
    Vector3d tic_[2];

    Vector3d g_;
    double td_;
};
} // namespace lidar_localization