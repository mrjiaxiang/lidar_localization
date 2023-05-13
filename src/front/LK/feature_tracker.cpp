#include "lidar_localization/front/LK/feature_tracker.h"

#include <glog/logging.h>

namespace lidar_localization {

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++) {
        if (status[i]) {
            v[j++] = v[i];
        }
    }
    v.resize(j);
}
void reduceVector(vector<int> &v, vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

bool FeatureTracker::inBorder(const cv::Point2f &pt) {
    const int BORDER_SIZE = 1;
    int img_x = round(pt.x);
    int img_y = round(pt.y);
    return BORDER_SIZE <= img_x && img_x < col_ - BORDER_SIZE &&
           BORDER_SIZE <= img_y && img_y < row_ - BORDER_SIZE;
}

FeatureTracker::FeatureTracker() {
    n_id_ = 0;
    has_prediction_ = false;

    klt_win_size_ = KLT_WIN_SIZE;
    pyramid_level_ = PYRAMID_LEVEL;
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img,
                           const cv::Mat &_img1) {
    TicToc t_r;
    cur_time_ = _cur_time;
    cur_img_ = _img;
    row_ = cur_img_.rows;
    col_ = cur_img_.cols;
    cv::Mat cur_right_img = _img1;

    if (EQUALIZE) {
        clahe_->apply(cur_img_, cur_img_);
        if (!cur_right_img.empty()) {
            clahe_->apply(cur_right_img, cur_right_img);
        }
    }

    cv::buildOpticalFlowPyramid(cur_img_, cur_image_pyr_,
                                cv::Size(klt_win_size_, klt_win_size_),
                                pyramid_level_);

    std::vector<cv::Point2f>().swap(cur_pts_);
    if (prev_pts_.size() > 0) {
        std::vector<uchar> status;
        vector<float> err;

        if (has_prediction_) {
            cur_pts_ = predict_pts_;
            std::vector<uchar> status;
            std::vector<float> err;
        }
    }

    std::vector<cv::Point2f>().swap(cur_pts_);
    if (prev_pts_.size() > 0) {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        if (has_prediction_) {
            cur_pts_ = predict_pts_;
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(prev_img_, cur_img_, prev_pts_, cur_pts_,
                                     status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT +
                                                          cv::TermCriteria::EPS,
                                                      30, 0.01),
                                     cv::OPTFLOW_USE_INITIAL_FLOW);

            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++) {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
                cv::calcOpticalFlowPyrLK(prev_img_, cur_img_, prev_pts_,
                                         cur_pts_, status, err,
                                         cv::Size(21, 21), 3);
        } else {
            cv::calcOpticalFlowPyrLK(prev_img_, cur_img_, prev_pts_, cur_pts_,
                                     status, err, cv::Size(21, 21), 3);
        }

        if (FLOW_BACK) {
            std::vector<uchar> reverse_status;
            std::vector<cv::Point2f> reverse_pts = prev_pts_;
            cv::calcOpticalFlowPyrLK(cur_img_, prev_img_, cur_pts_, reverse_pts,
                                     reverse_status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT +
                                                          cv::TermCriteria::EPS,
                                                      30, 0.01),
                                     cv::OPTFLOW_USE_INITIAL_FLOW);
            for (size_t i = 0; i < status.size(); i++) {
                if (status[i] && reverse_status[i] &&
                    distance(prev_pts_[i], reverse_pts[i]) <= 0.5) {
                    status[i] = 1;
                } else {
                    status[i] = 0;
                }
            }
        }

        for (int i = 0; i < int(cur_pts_.size()); i++)
            if (status[i] && !inBorder(cur_pts_[i]))
                status[i] = 0;
        reduceVector(prev_pts_, status);
        reduceVector(cur_pts_, status);
        reduceVector(ids_, status);
        reduceVector(track_cnt_, status);
        LOG(INFO) << "temporal optical flow costs: " << t_o.toc() << " ms";
    }

    for (auto &n : track_cnt_)
        n++;
    if (1) {
        LOG(INFO) << "set mask begins";
        TicToc t_m;
        setMask();
        LOG(INFO) << "set mask costs " << t_m.toc() << " ms.";

        LOG(INFO) << "detect feature begins";
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts_.size());
        if (n_max_cnt > 0) {
            if (mask_.empty())
                cout << "mask is empty " << endl;
            if (mask_.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img_, n_pts_, MAX_CNT - cur_pts_.size(),
                                    0.01, MIN_DIST, mask_);
        } else {
            n_pts_.clear();
        }
        LOG(INFO) << "detect feature costs " << t_t.toc() << " ms.";

        for (auto &p : n_pts_) {
            cur_pts_.push_back(p);
            ids_.push_back(n_id_++);
            track_cnt_.push_back(1);
        }
    }

    cur_un_pts_ = undistortedPts(cur_pts_, camera_[0]);
    pts_velocity_ =
        ptsVelocity(ids_, cur_un_pts_, cur_un_pts_map_, prev_un_pts_map_);

    if (!_img1.empty()) {
        ids_right_.clear();
        cur_right_pts_.clear();
        cur_un_right_pts_.clear();
        right_pts_velocity_.clear();
        cur_un_right_pts_map_.clear();
        if (!cur_pts_.empty()) {
            // printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img_, cur_right_img, cur_pts_,
                                     cur_right_pts_, status, err,
                                     cv::Size(21, 21), 3);
            // reverse check cur right ---- cur left
            if (FLOW_BACK) {
                cv::calcOpticalFlowPyrLK(
                    cur_right_img, cur_img_, cur_right_pts_, reverseLeftPts,
                    statusRightLeft, err, cv::Size(21, 21), 3);
                for (size_t i = 0; i < status.size(); i++) {
                    if (status[i] && statusRightLeft[i] &&
                        inBorder(cur_right_pts_[i]) &&
                        distance(cur_pts_[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right_ = ids_;
            reduceVector(cur_right_pts_, status);
            reduceVector(ids_right_, status);
            cur_un_right_pts_ = undistortedPts(cur_right_pts_, camera_[1]);
            right_pts_velocity_ =
                ptsVelocity(ids_right_, cur_un_right_pts_,
                            cur_un_right_pts_map_, prev_un_right_pts_map_);
        }
        prev_un_right_pts_map_ = cur_un_right_pts_map_;
    }

    if (SHOW_TRACK)
        drawTrack(cur_img_, cur_right_img, ids_, cur_pts_, cur_right_pts_,
                  prev_leftPts_map_);

    prev_img_ = cur_img_;
    prev_pts_ = cur_pts_;
    prev_un_pts_ = cur_un_pts_;
    prev_un_pts_map_ = cur_un_pts_map_;
    prev_time_ = cur_time_;
    has_prediction_ = false;
    prev_leftPts_map_.clear();

    for (size_t i = 0; i < cur_pts_.size(); i++)
        prev_leftPts_map_[ids_[i]] = cur_pts_[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids_.size(); i++) {
        int feature_id = ids_[i];
        double x, y, z;
        x = cur_un_pts_[i].x;
        y = cur_un_pts_[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts_[i].x;
        p_v = cur_pts_[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity_[i].x;
        velocity_y = pts_velocity_[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }

    if (!_img1.empty()) {
        for (size_t i = 0; i < ids_right_.size(); i++) {
            int feature_id = ids_right_[i];
            double x, y, z;
            x = cur_un_right_pts_[i].x;
            y = cur_un_right_pts_[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts_[i].x;
            p_v = cur_right_pts_[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity_[i].x;
            velocity_y = right_pts_velocity_[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
        }
    }

    return featureFrame;
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2) {
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void FeatureTracker::setMask() {
    mask_ = cv::Mat(row_, col_, CV_8UC1, cv::Scalar(255));
    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts_.size(); i++)
        cnt_pts_id.push_back(
            make_pair(track_cnt_[i], make_pair(cur_pts_[i], ids_[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(),
         [](const pair<int, pair<cv::Point2f, int>> &a,
            const pair<int, pair<cv::Point2f, int>> &b) {
             return a.first > b.first;
         });

    cur_pts_.clear();
    ids_.clear();
    track_cnt_.clear();

    for (auto &it : cnt_pts_id) {
        if (mask_.at<uchar>(it.second.first) == 255) {
            cur_pts_.push_back(it.second.first);
            ids_.push_back(it.second.second);
            track_cnt_.push_back(it.first);
            cv::circle(mask_, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts,
                                                   camodocal::CameraPtr cam) {
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++) {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

vector<cv::Point2f>
FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                            map<int, cv::Point2f> &cur_id_pts,
                            map<int, cv::Point2f> &prev_id_pts) {
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++) {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }
    if (!prev_id_pts.empty()) {
        double dt = cur_time_ - prev_time_;
        for (unsigned int i = 0; i < pts.size(); i++) {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end()) {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            } else {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }

    } else {
        for (unsigned int i = 0; i < cur_pts_.size(); i++) {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }

    return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts,
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap) {
    int cols = imLeft.cols;
    if (!imRight.empty())
        cv::hconcat(imLeft, imRight, img_track_);
    cv::cvtColor(img_track_, img_track_, cv::COLOR_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++) {
        double len = std::min(1.0, 1.0 * track_cnt_[j] / 20);
        cv::circle(img_track_, curLeftPts[j], 2,
                   cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty()) {
        for (size_t i = 0; i < curRightPts.size(); i++) {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(img_track_, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            // cv::Point2f leftPt = curLeftPtsTrackRight[i];
            // cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8,
            // 0);
        }
    }

    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++) {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if (mapIt != prevLeftPtsMap.end()) {
            cv::arrowedLine(img_track_, curLeftPts[i], mapIt->second,
                            cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file) {
    for (size_t i = 0; i < calib_file.size(); i++) {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        CameraPtr camera =
            CameraFactory::instance()->generateCameraFromYamlFile(
                calib_file[i]);
        camera_.push_back(camera);
    }
}

cv::Mat FeatureTracker::getTrackImage() { return img_track_; }

void FeatureTracker::extractFeature(const std::vector<cv::Mat> &prev_pyr,
                                    const std::vector<cv::Mat> &cur_pyr,
                                    const std::vector<cv::Point2f> &prev_kps,
                                    std::vector<cv::Point2f> &cur_kps,
                                    int win_size, int pyr_level,
                                    std::vector<float> &err,
                                    std::vector<uchar> &status) {
    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                              30, 0.01);
    cv::calcOpticalFlowPyrLK(prev_pyr, cur_pyr, prev_pts_, cur_kps, status, err,
                             cv::Size(win_size, win_size), pyr_level, criteria,
                             cv::OPTFLOW_USE_INITIAL_FLOW +
                                 cv::OPTFLOW_LK_GET_MIN_EIGENVALS);
    size_t nkps = prev_kps.size();
    std::vector<int> tracked_kps_idx;
    std::vector<cv::Point2f> tracked_prev_kps;
    std::vector<cv::Point2f> tracked_cur_kps;

    tracked_kps_idx.reserve(nkps);
    tracked_prev_kps.reserve(nkps);
    tracked_cur_kps.reserve(nkps);

    for (size_t i = 0; i < nkps; i++) {
        if (!status[i] && !inBorder(cur_kps[i])) {
            status[i] = 0;
            continue;
        }

        tracked_prev_kps.push_back(prev_kps[i]);
        tracked_cur_kps.push_back(cur_kps[i]);
        tracked_kps_idx.push_back(i);
    }

    if (tracked_cur_kps.empty()) {
        return;
    }

    if (FLOW_BACK) {
        std::vector<uchar> reverse_status;
        err.clear();
        cv::calcOpticalFlowPyrLK(
            cur_pyr, prev_pyr, tracked_cur_kps, tracked_prev_kps,
            reverse_status, err, cv::Size(win_size, win_size), 0, criteria,
            cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

        for (size_t i = 0; i < tracked_cur_kps.size(); ++i) {
            int idx = tracked_kps_idx[i];
            if (!reverse_status[i]) {
                status[idx] = 0;
                continue;
            }

            if (cv::norm(prev_kps[idx] - tracked_prev_kps[i]) > 0.5) {
                status[idx] = 0;
                continue;
            }
        }
    }
}

} // namespace lidar_localization