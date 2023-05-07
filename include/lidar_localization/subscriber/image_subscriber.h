#pragma once

#include <deque>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "lidar_localization/sensor_data/image_data.hpp"

namespace lidar_localization {
class ImageSubscriber {
  public:
    ImageSubscriber(ros::NodeHandle &nh, const std::string &cam_topic_name,
                    const size_t &buff_size);
    ImageSubscriber() = default;
    void ParseData(std::deque<sensor_msgs::ImageConstPtr> &image_data_buff);

  private:
    void msg_callback(const sensor_msgs::ImageConstPtr &cam_img);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber img_sub_;
    std::deque<sensor_msgs::ImageConstPtr> new_image_buff_;
};
} // namespace lidar_localization