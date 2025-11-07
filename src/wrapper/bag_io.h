//
// Created by xiang on 23-12-14.
//

#ifndef LIGHTNING_BAG_IO_H
#define LIGHTNING_BAG_IO_H

#include <csignal>
#include <functional>
#include <map>
#include <string>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include "livox_ros_driver/CustomMsg.h"
#include "driver_msgs/SensorImuInfo.h"
#include "driver_msgs/LivoxPacket.h"

#include "common/imu.h"
#include "common/odom.h"
#include "common/point_def.h"
#include "core/lightning_math.hpp"
#include "io/dataset_type.h"
#include "wrapper/ros_utils.h"

namespace lightning {

/**
 * ROSBAG IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 * ROS1版本
 */
class RosbagIO {
   public:
    explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::NCLT)
        : bag_file_(std::move(bag_file)) {
        /// handle ctrl-c
        signal(SIGINT, lightning::debug::SigHandle);
    }

    /// 一些方便直接使用的topics, messages
    using Scan2DHandle = std::function<bool(sensor_msgs::LaserScanConstPtr)>;

    using PointCloud2Handle = std::function<bool(sensor_msgs::PointCloud2ConstPtr)>;
    using LivoxCloud2Handle = std::function<bool(livox_ros_driver::CustomMsgConstPtr)>;
    using LivoxPacketHandle = std::function<bool(driver_msgs::LivoxPacketConstPtr)>;
    using FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
    using ImuHandle = std::function<bool(IMUPtr)>;
    using DriverImuHandle = std::function<bool(driver_msgs::SensorImuInfoConstPtr)>;
    using OdomHandle = std::function<bool(const OdomPtr &)>;

    /**
     * 遍历文件内容，调用回调函数
     * @param sleep_usec 每调用一个回调后的等待时间
     */
    void Go(int sleep_usec = 0);

    /// point cloud 2 处理
    RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) {
        cloud2_proc_[topic_name] = f;
        return *this;
    }

    /// livox 处理
    RosbagIO &AddLivoxCloudHandle(const std::string &topic_name, LivoxCloud2Handle f) {
        livox_proc_[topic_name] = f;
        return *this;
    }

    /// driver_msgs LivoxPacket 处理
    RosbagIO &AddLivoxPacketHandle(const std::string &topic_name, LivoxPacketHandle f) {
        livox_packet_proc_[topic_name] = f;
        return *this;
    }

    RosbagIO &AddImuHandle(const std::string &topic_name, ImuHandle f) {
        imu_proc_[topic_name] = f;
        return *this;
    }

    /// driver_msgs IMU 处理
    RosbagIO &AddDriverImuHandle(const std::string &topic_name, DriverImuHandle f) {
        driver_imu_proc_[topic_name] = f;
        return *this;
    }

    /// 清除现有的处理函数
    void CleanProcessFunc() { 
        cloud2_proc_.clear();
        livox_proc_.clear();
        livox_packet_proc_.clear();
        imu_proc_.clear();
        driver_imu_proc_.clear();
    }

   private:
    std::map<std::string, PointCloud2Handle> cloud2_proc_;
    std::map<std::string, LivoxCloud2Handle> livox_proc_;
    std::map<std::string, LivoxPacketHandle> livox_packet_proc_;
    std::map<std::string, ImuHandle> imu_proc_;
    std::map<std::string, DriverImuHandle> driver_imu_proc_;

    std::string bag_file_;
    DatasetType dataset_type_ = DatasetType::NCLT;
};
}  // namespace lightning

#endif  // SLAM_ROS_BAG_IO_H
