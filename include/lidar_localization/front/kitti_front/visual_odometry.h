#pragma once

#include "lidar_localization/front/kitti_front/common_include.h"
#include "lidar_localization/front/kitti_front/frontend.h"

class VisualOdometry {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    /// constructor with config file
    VisualOdometry(std::string &config_path);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * start vo in the dataset
     */
    // void Run();

    /**
     * Make a step forward in dataset
     */
    bool Step(cv::Mat &left, cv::Mat &right);

    /// 获取前端状态
    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

  private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    // Backend::Ptr backend_ = nullptr;
    // Map::Ptr map_ = nullptr;
    // Viewer::Ptr viewer_ = nullptr;
};