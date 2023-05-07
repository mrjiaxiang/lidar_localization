#include "lidar_localization/subscriber/image_subscriber.h"

#include <glog/logging.h>

namespace lidar_localization {
ImageSubscriber::ImageSubscriber(ros::NodeHandle &nh,
                                 const std::string &cam_topic_name,
                                 const size_t &buff_size)
    : nh_(nh) {
    img_sub_ = nh_.subscribe(cam_topic_name, buff_size,
                             &ImageSubscriber::msg_callback, this);
}

void ImageSubscriber::ParseData(
    std::deque<sensor_msgs::ImageConstPtr> &image_data_buff) {
    if (new_image_buff_.size() > 0) {
        image_data_buff.insert(image_data_buff.end(), new_image_buff_.begin(),
                               new_image_buff_.end());
        new_image_buff_.clear();
    }
}

void ImageSubscriber::msg_callback(const sensor_msgs::ImageConstPtr &cam_img) {
    new_image_buff_.push_back(cam_img);
}

} // namespace lidar_localization