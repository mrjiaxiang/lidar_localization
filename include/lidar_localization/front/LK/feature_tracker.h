#pragma once

#include <Eigen/Dense>
#include <csignal>
#include <cstdio>
#include <execinfo.h>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <queue>

#include "lidar_localization/camera_model/camera_models/Camera.h"
#include "lidar_localization/camera_model/camera_models/CameraFactory.h"
#include "lidar_localization/parameters/parameters.h"
#include "lidar_localization/time/tic_toc.h"

using namespace std;
using namespace Eigen;
using namespace camodocal;

namespace lidar_localization {

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);
class FeatureTracker {
  public:
    FeatureTracker();
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
    trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    bool inBorder(const cv::Point2f &pt);
    void setMask();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts,
                                       camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                                    map<int, cv::Point2f> &cur_id_pts,
                                    map<int, cv::Point2f> &prev_id_pts);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                   vector<int> &curLeftIds, vector<cv::Point2f> &curLeftPts,
                   vector<cv::Point2f> &curRightPts,
                   map<int, cv::Point2f> &prevLeftPtsMap);
    void readIntrinsicParameter(const vector<string> &calib_file);
    cv::Mat getTrackImage();

  private:
    int n_id_;
    bool has_prediction_;
    double cur_time_, prev_time_;
    cv::Mat cur_img_;
    int col_;
    int row_;
    cv::Mat mask_;

    cv::Mat img_track_;

    cv::Mat prev_img_;
    vector<cv::Point2f> cur_pts_;
    vector<cv::Point2f> cur_right_pts_;
    vector<cv::Point2f> prev_pts_;
    vector<cv::Point2f> cur_un_pts_;
    vector<cv::Point2f> pts_velocity_, right_pts_velocity_;
    std::vector<cv::Point2f> predict_pts_;
    std::vector<int> ids_, ids_right_;
    std::vector<int> track_cnt_;
    std::vector<cv::Point2f> n_pts_;
    vector<cv::Point2f> prev_un_pts_, cur_un_right_pts_;
    cv::Ptr<cv::CLAHE> clahe_;
    vector<cv::Mat> cur_image_pyr_;
    vector<cv::Mat> prev_image_pyr_;

    std::map<int, Eigen::Vector3d> local_pt3d_map_;
    map<int, cv::Point2f> cur_un_pts_map_, prev_un_pts_map_;
    map<int, cv::Point2f> cur_un_right_pts_map_, prev_un_right_pts_map_;
    map<int, cv::Point2f> prev_leftPts_map_;
    std::vector<CameraPtr> camera_;
};

} // namespace lidar_localization