#include "lidar_localization/front/LK/pose_graph.h"
#include "lidar_localization/time/tic_toc.h"
#include <glog/logging.h>

namespace lidar_localization {
Estimator::Estimator() {
    LOG(INFO) << "init begins.";
    init_thread_flag_ = false;
    input_image_cnt_ = 0;

    running_ = false;
}
Estimator::~Estimator() {}

void Estimator::initialise() {
    if (!init_thread_flag_) {
        running_ = true;
        init_thread_flag_ = true;
        track_thread_.reset(new std::thread(&Estimator::trackingTread, this));
        measurement_thread_.reset(
            new std::thread(&Estimator::measurementThread, this));
        imu_thread_.reset(new std::thread(&Estimator::imuThread, this));
    }
}

void Estimator::setParameter() {
    process_mtx_.lock();
    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic_[i] = TIC[i];
        ric_[i] = RIC[i];
        LOG(INFO) << " exitrinsic cam " << i << endl
                  << ric_[i] << endl
                  << tic_[i].transpose();
    }

    td_ = TD;
    g_ = G;

    LOG(INFO) << "set g " << g_.transpose();
    feature_tracker_.readIntrinsicParameter(CAM_NAMES);

    process_mtx_.unlock();
}

void Estimator::inputImage(double t, const cv::Mat &_img,
                           const cv::Mat &_img1) {

    m_buf_mtx_.lock();
    image_buff_.push(make_pair(t, make_pair(_img, _img1)));
    m_buf_mtx_.unlock();
}

void Estimator::inputImu(const IMUData &imu_data) {
    m_imu_mtx_.lock();
    imu_buff_.push(imu_data);
    m_imu_mtx_.unlock();

    if (solver_flag_ == NON_LINEAR) {
        m_propagate_.lock();
        fastPredictIMU(imu_data);
        pubLatestOdometry(latest_p_, latest_q_, latest_v_, imu_data.time);
        m_propagate_.unlock();
    }
}

void Estimator::inputGNSS(const GNSSData &gnss_data) {
    m_gnss_mtx_.lock();
    gnss_buff_.push(gnss_data);
    m_gnss_mtx_.unlock();
}

void Estimator::trackingTread() {
    while (running_) {
        std::pair<double, std::pair<cv::Mat, cv::Mat>> tmp_data;
        m_buf_mtx_.lock();
        if (!image_buff_.empty()) {
            tmp_data = image_buff_.front();
            image_buff_.pop();
        }
        m_buf_mtx_.unlock();

        if (!tmp_data.second.first.empty() && !tmp_data.second.second.empty()) {
            trackImage(tmp_data.first, tmp_data.second.first,
                       tmp_data.second.second);
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void Estimator::trackImage(double t, const cv::Mat &_img,
                           const cv::Mat &_img1) {
    input_image_cnt_++;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> feature_frame;
    TicToc feature_tracker_time;

    feature_frame = feature_tracker_.trackImage(t, _img, _img1);

    if (SHOW_TRACK) {
        cv::Mat img_track = feature_tracker_.getTrackImage();
        pubTrackImage(img_track, t);
    }

    {
        m_buf_mtx_.lock();
        feature_buf_.push(make_pair(t, feature_frame));
        after_image_buff_.push(_img);
        m_buf_mtx_.unlock();
    }

    LOG(INFO) << "has tracked the image.";
}

void Estimator::fastPredictIMU(IMUData imu_data) {
    double dt = imu_data.time - latest_time;
    latest_time = imu_data.time;
    Eigen::Vector3d un_acc_0 = latest_q_ * (latest_acc_0_ - latest_ba_) - g_;
    Eigen::Vector3d un_gyr =
        0.5 * (latest_gyr_0_ + Eigen::Vector3d(imu_data.angular_velocity.x,
                                               imu_data.angular_velocity.y,
                                               imu_data.angular_velocity.y)) -
        latest_bg_;
    latest_q_ = latest_q_ * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 =
        latest_q_ * (Eigen::Vector3d(imu_data.linear_acceleration.x,
                                     imu_data.linear_acceleration.y,
                                     imu_data.linear_acceleration.z) -
                     latest_ba_) -
        g_;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_p_ = latest_p_ + dt * latest_v_ + 0.5 * dt * dt * un_acc;
    latest_v_ = latest_v_ + dt * un_acc;
    latest_acc_0_ = Eigen::Vector3d(imu_data.linear_acceleration.x,
                                    imu_data.linear_acceleration.y,
                                    imu_data.linear_acceleration.z);
    latest_gyr_0_ = Eigen::Vector3d(imu_data.angular_velocity.x,
                                    imu_data.angular_velocity.y,
                                    imu_data.angular_velocity.z);
}

void Estimator::measurementThread() {}
void Estimator::imuThread() {
    while (running_) {
    }
}

} // namespace lidar_localization