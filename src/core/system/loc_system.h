//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "livox_ros_driver/CustomMsg.h"
#include "driver_msgs/SensorImuInfo.h"
#include "driver_msgs/LivoxPacket.h"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

namespace loc {
class Localization;
}

class LocSystem {
   public:
    struct Options {
        bool pub_tf_ = true;  // 是否发布tf
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    /// 初始化，地图路径在yaml里配置
    bool Init(const std::string& yaml_path);

    /// 设置初始化位姿
    void SetInitPose(const SE3& pose);

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);
    void ProcessIMU(const driver_msgs::SensorImuInfoConstPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void ProcessLidar(const livox_ros_driver::CustomMsgConstPtr& cloud);
    void ProcessLivoxPacket(const driver_msgs::LivoxPacketConstPtr& packet);

    /// 实时模式下的spin
    void Spin();

   private:
    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;  // 定位接口

    std::atomic_bool loc_started_ = false;  // 是否开启定位

    /// 实时模式下的ros node, subscribers
    ros::NodeHandle nh_;
    std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_ = nullptr;

    std::string imu_topic_;
    std::string driver_imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;
    std::string livox_packet_topic_;

    ros::Subscriber imu_sub_;
    ros::Subscriber driver_imu_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber livox_sub_;
    ros::Subscriber livox_packet_sub_;
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
