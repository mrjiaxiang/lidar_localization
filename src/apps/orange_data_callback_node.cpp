#include <ros/ros.h>

#include <glog/logging.h>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/sensor_data/gnss_data.hpp"
#include "lidar_localization/sensor_data/image_data.hpp"
#include "lidar_localization/sensor_data/imu_data.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/subscriber/image_subscriber.h"
#include "lidar_localization/subscriber/imu_subscriber.hpp"

#include "lidar_localization/front/LK/pose_graph.h"
#include "lidar_localization/parameters/parameters.h"

#include "lidar_localization/front/LK/visualization.h"

using namespace lidar_localization;
using namespace std;

std::shared_ptr<Estimator> estimator_ptr = make_shared<Estimator>();

std::deque<sensor_msgs::ImageConstPtr> img_left_data_buff;
std::deque<sensor_msgs::ImageConstPtr> img_right_data_buff;
std::deque<IMUData> imu_data_buff;
std::deque<GNSSData> gnss_data_buff;

void SyncProcess();
cv::Mat GetImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_callback_node");
    ros::NodeHandle nh;

    std::string config_file =
        WORK_SPACE_PATH + "/config/kitti_10_03_config.yaml";

    readParameters(config_file);

    estimator_ptr->initialise();
    estimator_ptr->setParameter();

    registerPub(nh);

    shared_ptr<IMUSubscriber> imu_sub_ptr =
        make_shared<IMUSubscriber>(nh, IMU_TOPIC, 1000000);
    shared_ptr<GNSSSubscriber> gnss_sub_ptr =
        make_shared<GNSSSubscriber>(nh, GNSS_TOPIC, 10000);
    shared_ptr<ImageSubscriber> img_left_sub_ptr =
        make_shared<ImageSubscriber>(nh, IMAGE0_TOPIC, 100000);
    shared_ptr<ImageSubscriber> img_right_sub_ptr =
        make_shared<ImageSubscriber>(nh, IMAGE1_TOPIC, 100000);

    std::thread sync_thread{SyncProcess};
    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        imu_sub_ptr->ParseData(imu_data_buff);
        img_left_sub_ptr->ParseData(img_left_data_buff);
        img_right_sub_ptr->ParseData(img_right_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        if (!imu_data_buff.empty()) {
            estimator_ptr->inputImu(imu_data_buff.front());
            imu_data_buff.pop_front();
        }

        if (!gnss_data_buff.empty()) {
            estimator_ptr->inputGNSS(gnss_data_buff.front());
            gnss_data_buff.pop_front();
        }

        rate.sleep();
    }
    return 0;
}

void SyncProcess() {
    while (true) {
        cv::Mat image0, image1;
        std_msgs::Header header;
        double time = 0.f;
        if (!img_left_data_buff.empty() && !img_right_data_buff.empty()) {
            double time0 = img_left_data_buff.front()->header.stamp.toSec();
            double time1 = img_right_data_buff.front()->header.stamp.toSec();
            if (time0 < time1 - 0.003) {
                img_left_data_buff.pop_front();
                LOG(INFO) << "throw img left.";
            } else if (time0 > time1 + 0.003) {
                img_right_data_buff.pop_front();
                LOG(INFO) << "throw img right.";
            } else {
                time = img_left_data_buff.front()->header.stamp.toSec();
                header = img_left_data_buff.front()->header;
                image0 = GetImageFromMsg(img_left_data_buff.front());
                img_left_data_buff.pop_front();
                image1 = GetImageFromMsg(img_right_data_buff.front());
                img_right_data_buff.pop_front();
            }
        }
        if (!image0.empty()) {
            LOG(INFO) << "start vslam front end.";
            estimator_ptr->inputImage(time, image0, image1);
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

cv::Mat GetImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1") {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    } else {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    cv::Mat img = ptr->image.clone();
    return img;
}