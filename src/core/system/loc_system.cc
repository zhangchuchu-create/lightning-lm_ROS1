//
// Created by xiang on 25-9-12.
//

#include <csignal>

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include "core/system/comm.h"
#include "core/system/livox_lidar_def.h"

using namespace livox_ros;
namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    //LOG(INFO) << "online mode, creating ros node ... ";

    /// subscribers
    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    driver_imu_topic_ = yaml.GetValue<std::string>("common", "driver_imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");
    livox_packet_topic_ = yaml.GetValue<std::string>("common", "driver_lidar_packet_topic");

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
        imu_topic_, 1000, [this](const sensor_msgs::ImuConstPtr& msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    driver_imu_sub_ = nh_.subscribe<driver_msgs::SensorImuInfo>(
        driver_imu_topic_, 1000, [this](const driver_msgs::SensorImuInfoConstPtr& msg) {
            ProcessIMU(msg);
        });

    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        cloud_topic_, 10, [this](const sensor_msgs::PointCloud2ConstPtr& cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = nh_.subscribe<livox_ros_driver::CustomMsg>(
        livox_topic_, 10, [this](const livox_ros_driver::CustomMsgConstPtr& cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_packet_sub_ = nh_.subscribe<driver_msgs::LivoxPacket>(
        livox_packet_topic_, 10, [this](const driver_msgs::LivoxPacketConstPtr& packet) {
            Timer::Evaluate([&]() { ProcessLivoxPacket(packet); }, "Proc LivoxPacket", true);
        });

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
        loc_->SetTFCallback(
            [this](const geometry_msgs::TransformStamped &pose) { 
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(pose.transform.translation.x, 
                                                 pose.transform.translation.y, 
                                                 pose.transform.translation.z));
                tf::Quaternion q(pose.transform.rotation.x, 
                                 pose.transform.rotation.y, 
                                 pose.transform.rotation.z, 
                                 pose.transform.rotation.w);
                transform.setRotation(q);
                tf_broadcaster_->sendTransform(tf::StampedTransform(transform, 
                                                                     ros::Time(pose.header.stamp.sec, pose.header.stamp.nsec),
                                                                     pose.header.frame_id, 
                                                                     pose.child_frame_id));
            });
    }

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        //LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    //LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
            //   << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessIMU(const driver_msgs::SensorImuInfoConstPtr &msg) {
    if (loc_started_) {
        // 将 driver_msgs::SensorImuInfo 转换为内部 IMU 格式
        IMUPtr imu = std::make_shared<IMU>();
        imu->timestamp = ToSec(msg->header.stamp);
        
        // 从 driver_msgs 中提取加速度和角速度数据
        // driver_msgs::SensorImuInfo 使用 acce_x/y/z 和 gyro_x/y/z 字段
        imu->linear_acceleration = Vec3d(msg->acce_x, msg->acce_y, msg->acce_z);
        imu->angular_velocity = Vec3d(msg->gyro_x, msg->gyro_y, msg->gyro_z);
        
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::PointCloud2ConstPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver::CustomMsgConstPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::ProcessLivoxPacket(const driver_msgs::LivoxPacketConstPtr &packet) {
    if (!loc_started_) {
        return;
    }
    // 解析 LivoxPacket 原始数据包
    uint64_t packet_time = packet->header.stamp.toNSec();
    int num_packets = packet->data.size() / SCAN_PACKET_SIZE;
    
    if (num_packets == 0) {
        return;
    }

    // 创建 PointCloud2 消息
    sensor_msgs::PointCloud2Ptr cloud = boost::make_shared<sensor_msgs::PointCloud2>();
    cloud->header = packet->header;
    
    // 设置点云字段：x, y, z, intensity, time, ring (匹配 velodyne_ros::Point 格式)
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2Fields(6,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "time", 1, sensor_msgs::PointField::FLOAT32,
        "ring", 1, sensor_msgs::PointField::UINT16);
    
    // 预分配空间
    int total_points = num_packets * POINT_NUMBER;
    modifier.resize(total_points);
    
    // 迭代器
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> iter_time(*cloud, "time");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(*cloud, "ring");
    
    // 解析每个数据包
    // PACKET_TIME_INTERVAL 是数据包间隔(ns)，需要除以点数得到点间隔
    uint64_t point_interval_ns = PACKET_TIME_INTERVAL / POINT_NUMBER;  // 每个点的时间间隔(纳秒)
    
    for (int pkt_idx = 0; pkt_idx < num_packets; ++pkt_idx) {
        int start_index = pkt_idx * SCAN_PACKET_SIZE;
        
        // 检查数据越界
        if (start_index + SCAN_PACKET_SIZE > static_cast<int>(packet->data.size())) {
            break;
        }
        
        // 获取原始点数据指针
        const LivoxLidarRawPoint* points = 
            reinterpret_cast<const LivoxLidarRawPoint*>(&packet->data[start_index]);
        
        // 计算此数据包的相对时间戳（相对于点云开始时间，单位：纳秒）
        uint64_t relative_timestamp_ns = pkt_idx * PACKET_TIME_INTERVAL;
        
        // 解析每个点
        for (int i = 0; i < POINT_NUMBER; ++i) {
            // 转换坐标：从 mm 到 m
            *iter_x = points[i].x / 1000.0f;
            *iter_y = points[i].y / 1000.0f;
            *iter_z = points[i].z / 1000.0f;
            *iter_intensity = static_cast<float>(points[i].reflectivity);
            // time 字段：相对于点云开始的时间偏移，单位：毫秒
            *iter_time = static_cast<float>((relative_timestamp_ns + i * point_interval_ns) / 1e6);
            *iter_ring = static_cast<uint16_t>(i % 4); // Livox Mid-360 有4线
            
            ++iter_x; ++iter_y; ++iter_z; ++iter_intensity; ++iter_time; ++iter_ring;
        }
    }
/*
    pcl::PointCloud<livox_ros::PointXYZRTLT>::Ptr cloud(new pcl::PointCloud<livox_ros::PointXYZRTLT>);
    uint64_t packet_time = packet->header.stamp.toNSec();  // 给的时间戳度是头时间戳
    int num_packets = packet->data.size() / SCAN_PACKET_SIZE;
    int point_count = num_packets * 96;
    std::vector<livox_ros::PointXyzlt> points_clouds_;
    for (int i = 0; i < num_packets; ++i) {
        int start_index = i * SCAN_PACKET_SIZE;
        int end_index = start_index + SCAN_PACKET_SIZE;

        std::vector<uint8_t> packet_vec(packet->data.begin() + start_index, packet->data.begin() + end_index);
        // ProcessPacket(packet, point_count, pcl_cloud, packet_time, i);
        LivoxLidarRawPoint* points = (LivoxLidarRawPoint*)packet_vec.data();
        uint64_t point_interval_ns = PACKET_TIME_INTERVAL / POINT_NUMBER;  // 每个点的时间间隔（纳秒）
        uint64_t relative_timestamp_ns = i * PACKET_TIME_INTERVAL;  // 此数据包相对于点云开始的时间（纳秒）
        uint8_t line_num = 4; // Livox Mid-360 有4线
        PointXyzlt point = {};
        for (uint32_t j = 0; j < POINT_NUMBER; j++) {
            point.x = points[j].x / 1000.0;
            point.y = points[j].y / 1000.0;
            point.z = points[j].z / 1000.0;
            point.intensity = points[j].reflectivity;
            point.line = j % line_num;
            point.tag = 0;
            // offset_time 应该是相对时间（毫秒），不是绝对时间戳
            point.offset_time = static_cast<uint64_t>((relative_timestamp_ns + j * point_interval_ns) / 1e6);
            points_clouds_.push_back(point);
        }
        if (point_count == static_cast<int>(points_clouds_.size())) {
            std::vector<PointXyzlt> point_cloud;
            point_cloud.swap(points_clouds_);
            uint64_t time = 0;
            // InitPointcloud2Msg(point_cloud, pcl_cloud, time);
            cloud->clear();
            cloud->is_dense = true;
            for (size_t i = 0; i < point_cloud.size(); ++i) {
                livox_ros::PointXYZRTLT point;
                point.x = point_cloud[i].x;
                point.y = point_cloud[i].y;
                point.z = point_cloud[i].z;
                // std::cout << "点Z坐标: " << point.z << std::endl;
                point.reflectivity = point_cloud[i].intensity;
                point.tag = point_cloud[i].tag;
                point.line = point_cloud[i].line;
                point.timestamp = static_cast<double>(point_cloud[i].offset_time);
                cloud->push_back(point);
            }
            cloud->height = 1;
            cloud->width = point_cloud.size();
        }
    }

    //pcl::PointCloud<livox_ros::PointXYZRTLT>格式转sensor_msgs::PointCloud2Ptr
    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);
    cloud_msg->header = packet->header;

    */
    // 处理解析后的点云
    loc_->ProcessLidarMsg(cloud);
}

void LocSystem::Spin() {
    ros::spin();
}

}  // namespace lightning