#pragma once

#include "lidar_localization/front/kitti_front/frame.h"
#include "lidar_localization/front/kitti_front/mappoint.h"
#include <lidar_localization/front/kitti_front/common_include.h>
#include <memory>
#include <opencv2/features2d/features2d.hpp>

struct Frame;
struct MapPoint;

struct Feature {
    /* data */
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;        // 持有该feature的frame
    cv::KeyPoint position_;             // 2D提取位置（图像像素点）
    std::weak_ptr<MapPoint> map_point_; // 关联地图点

    bool is_outlier_ = false;
    bool is_on_left_image_ = true;

  public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
