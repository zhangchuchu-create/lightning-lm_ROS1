//
// Created by xiang on 23-12-14.
//

#include "bag_io.h"

//#include <glog/logging.h>

namespace lightning {

void RosbagIO::Go(int sleep_usec) {
    rosbag::Bag bag;
    try {
        std::cout << "正在打开 bag 文件: " << bag_file_ << std::endl;
        bag.open(bag_file_, rosbag::bagmode::Read);
        std::cout << "Bag 文件打开成功" << std::endl;
    } catch (rosbag::BagException& e) {
        std::cerr << "打开 bag 文件失败: " << e.what() << std::endl;
        return;
    }

    std::vector<std::string> topics;
    for (auto& kv : cloud2_proc_) topics.push_back(kv.first);
    for (auto& kv : livox_proc_) topics.push_back(kv.first);
    for (auto& kv : livox_packet_proc_) topics.push_back(kv.first);
    for (auto& kv : imu_proc_) topics.push_back(kv.first);
    for (auto& kv : driver_imu_proc_) topics.push_back(kv.first);

    std::cout << "订阅的 topics 数量: " << topics.size() << std::endl;
    for (const auto& topic : topics) {
        std::cout << "  - " << topic << std::endl;
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::cout << "Bag 文件中的消息总数: " << view.size() << std::endl;

    int cloud_count = 0, imu_count = 0, livox_count = 0, livox_packet_count = 0, driver_imu_count = 0;
    for (rosbag::MessageInstance const m : view) {
        // PointCloud2
        sensor_msgs::PointCloud2ConstPtr cloud2 = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud2 != nullptr) {
            cloud_count++;
            auto iter = cloud2_proc_.find(m.getTopic());
            if (iter != cloud2_proc_.end()) {
                iter->second(cloud2);
            }
        }

        // Livox CustomMsg
        livox_ros_driver::CustomMsgConstPtr livox = m.instantiate<livox_ros_driver::CustomMsg>();
        if (livox != nullptr) {
            livox_count++;
            auto iter = livox_proc_.find(m.getTopic());
            if (iter != livox_proc_.end()) {
                iter->second(livox);
            }
        }

        // driver_msgs LivoxPacket
        driver_msgs::LivoxPacketConstPtr livox_packet = m.instantiate<driver_msgs::LivoxPacket>();
        if (livox_packet != nullptr) {
            livox_packet_count++;
            auto iter = livox_packet_proc_.find(m.getTopic());
            if (iter != livox_packet_proc_.end()) {
                iter->second(livox_packet);
            }
        }

        // IMU
        sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg != nullptr) {
            imu_count++;
            auto iter = imu_proc_.find(m.getTopic());
            if (iter != imu_proc_.end()) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(imu_msg->header.stamp);
                imu->linear_acceleration = Vec3d(imu_msg->linear_acceleration.x, 
                                                  imu_msg->linear_acceleration.y, 
                                                  imu_msg->linear_acceleration.z);
                imu->angular_velocity = Vec3d(imu_msg->angular_velocity.x, 
                                               imu_msg->angular_velocity.y, 
                                               imu_msg->angular_velocity.z);
                iter->second(imu);
            }
        }

        // driver_msgs IMU
        driver_msgs::SensorImuInfoConstPtr driver_imu_msg = m.instantiate<driver_msgs::SensorImuInfo>();
        if (driver_imu_msg != nullptr) {
            driver_imu_count++;
            auto iter = driver_imu_proc_.find(m.getTopic());
            if (iter != driver_imu_proc_.end()) {
                iter->second(driver_imu_msg);
            }
        }

        if (sleep_usec > 0) {
            usleep(sleep_usec);
        }

        if (lightning::debug::flg_exit) {
            break;
        }
    }

    std::cout << "处理完成 - PointCloud2: " << cloud_count 
              << ", IMU: " << imu_count 
              << ", Livox: " << livox_count
              << ", LivoxPacket: " << livox_packet_count
              << ", DriverIMU: " << driver_imu_count << std::endl;

    bag.close();
    std::cout << "Bag 文件 " << bag_file_ << " 处理完成" << std::endl;
}

}  // namespace lightning