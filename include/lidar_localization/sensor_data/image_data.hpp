#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_IMAGE_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_IMAGE_DATA_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

#include <iostream>

namespace lidar_localization {
class ImageData {
  public:
    ImageData() {}

  public:
    sensor_msgs::Image image;
    double time_stamp{0.0};
};
} // namespace lidar_localization

#endif