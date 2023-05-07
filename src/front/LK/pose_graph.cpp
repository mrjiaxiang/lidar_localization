#include "lidar_localization/front/LK/pose_graph.h"
#include "lidar_localization/time/tic_toc.h"
#include <glog/logging.h>

namespace lidar_localization {
Estimator::Estimator() {
    LOG(INFO) << "init begins.";
    init_thread_flag_ = false;
    input_image_cnt_ = 0;
}
Estimator::~Estimator() {}

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
    input_image_cnt_++;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> feature_frame;
    TicToc feature_tracker_time;

    feature_frame = feature_tracker_.trackImage(t, _img, _img1);

    if (SHOW_TRACK) {
        cv::Mat img_track = feature_tracker_.getTrackImage();
        pubTrackImage(img_track, t);
    }
}

} // namespace lidar_localization