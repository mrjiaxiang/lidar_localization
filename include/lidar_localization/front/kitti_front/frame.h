#pragma once

#include "lidar_localization/front/kitti_front/camera.h"
#include "lidar_localization/front/kitti_front/common_include.h"
#include "lidar_localization/front/kitti_front/feature.h"

struct Feature;
struct MapPoint;

struct Frame {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;          // id of this frame
    unsigned long keyframe_id_ = 0; // id of key frame
    bool is_keyframe_ = false;      // 是否为关键帧
    double time_stamp_;             // 时间戳，暂不使用

    SE3 pose_;
    std::mutex pose_mutex_;
    cv::Mat left_img_, right_img_;

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

  public:
    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
          const Mat &right);

    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    /// 设置关键帧并分配并键帧id
    void SetKeyFrame();

    /// 工厂构建模式，分配id
    static std::shared_ptr<Frame> CreateFrame();
};
