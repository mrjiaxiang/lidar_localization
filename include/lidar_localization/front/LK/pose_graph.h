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
#include "lidar_localization/sensor_data/gnss_data.hpp"
#include "lidar_localization/sensor_data/imu_data.hpp"
#include "lidar_localization/utility/utility.h"

using namespace std;
using namespace Eigen;

namespace lidar_localization {
class Estimator {
  public:
    Estimator();
    ~Estimator();

    void initialise();

    void inputImage(double t, const cv::Mat &_img,
                    const cv::Mat &_img1 = cv::Mat());

    void inputImu(const IMUData &imu_data);

    void inputGNSS(const GNSSData &gnss_data);

    void setParameter();

    void trackingTread();
    void trackImage(double t, const cv::Mat &_img, const cv::Mat &_img1);

    void measurementThread();
    void imuThread();

    void fastPredictIMU(IMUData imu_data);

    enum SolverFlag { INITIAL, NON_LINEAR };

    SolverFlag solver_flag_;

  private:
    long unsigned int input_image_cnt_;

    bool init_thread_flag_;

    FeatureTracker feature_tracker_;

    std::mutex process_mtx_;

    Matrix3d ric_[2];
    Vector3d tic_[2];

    Vector3d g_;
    double td_;

    // MULTIPLE_THREAD
    volatile std::atomic<bool> running_;
    std::shared_ptr<std::thread> track_thread_;
    std::shared_ptr<std::thread> measurement_thread_;
    std::shared_ptr<std::thread> imu_thread_;

    //
    std::mutex m_buf_mtx_;
    std::queue<std::pair<double, std::pair<cv::Mat, cv::Mat>>> image_buff_;

    std::mutex m_imu_mtx_;
    std::queue<IMUData> imu_buff_;

    std::mutex m_gnss_mtx_;
    std::queue<GNSSData> gnss_buff_;

    std::mutex m_propagate_;
    double latest_time;
    Eigen::Vector3d latest_p_, latest_v_, latest_ba_, latest_bg_, latest_acc_0_,
        latest_gyr_0_;
    Eigen::Quaterniond latest_q_;

    std::queue<std::pair<
        double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>>
        feature_buf_;
    queue<cv::Mat> after_image_buff_;
};
} // namespace lidar_localization