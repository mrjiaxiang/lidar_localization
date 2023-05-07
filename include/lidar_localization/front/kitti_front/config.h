#pragma once

#include "lidar_localization/front/kitti_front/common_include.h"

class Config {
  private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {}

  public:
    ~Config(); // close the file when deconstructing

    // set a new config file
    static bool SetParameterFile(const std::string &filename);

    // access the parameter values
    template <typename T> static T Get(const std::string &key) {
        return T(Config::config_->file_[key]);
    }
};