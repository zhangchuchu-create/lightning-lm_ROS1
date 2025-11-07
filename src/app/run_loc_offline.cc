//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>

//using namespace google::logging::internal;

#include "core/localization/localization.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"
#include "driver_msgs/SensorImuInfo.h"
#include "driver_msgs/LivoxPacket.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include "core/system/livox_lidar_def.h"
#include "core/system/comm.h"

using namespace livox_ros;
// 解析 LivoxPacket 转换为 PointCloud2
sensor_msgs::PointCloud2Ptr ParseLivoxPacket(const driver_msgs::LivoxPacketConstPtr& packet) {
    // static bool first_call = true;
    // uint64_t packet_time = packet->header.stamp.toNSec();
    // int num_packets = packet->data.size() / SCAN_PACKET_SIZE;
    
    // if (first_call) {
    //     std::cout << "LivoxPacket 数据大小: " << packet->data.size() << " 字节" << std::endl;
    //     std::cout << "包含子数据包数量: " << num_packets << std::endl;
    //     first_call = false;
    // }
    
    // if (num_packets == 0) {
    //     std::cerr << "警告: LivoxPacket 数据大小为 " << packet->data.size() 
    //               << " 字节，无法解析" << std::endl;
    //     return nullptr;
    // }

    // // 创建 PointCloud2 消息
    // sensor_msgs::PointCloud2Ptr cloud = boost::make_shared<sensor_msgs::PointCloud2>();
    // cloud->header = packet->header;
    
    // // 设置点云字段：x, y, z, intensity, time, ring (匹配 velodyne_ros::Point 格式)
    // sensor_msgs::PointCloud2Modifier modifier(*cloud);
    // modifier.setPointCloud2Fields(6,
    //     "x", 1, sensor_msgs::PointField::FLOAT32,
    //     "y", 1, sensor_msgs::PointField::FLOAT32,
    //     "z", 1, sensor_msgs::PointField::FLOAT32,
    //     "intensity", 1, sensor_msgs::PointField::FLOAT32,
    //     "time", 1, sensor_msgs::PointField::FLOAT32,
    //     "ring", 1, sensor_msgs::PointField::UINT16);
    
    // // 预分配空间
    // int total_points = num_packets * POINT_NUMBER;
    // modifier.resize(total_points);
    
    // // 迭代器
    // sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    // sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    // sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
    // sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cloud, "intensity");
    // sensor_msgs::PointCloud2Iterator<float> iter_time(*cloud, "time");
    // sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(*cloud, "ring");
    
    // // 解析每个数据包
    // // PACKET_TIME_INTERVAL 是数据包间隔(ns)，需要除以点数得到点间隔
    // uint64_t point_interval_ns = PACKET_TIME_INTERVAL / POINT_NUMBER;  // 每个点的时间间隔(纳秒)
    
    // for (int pkt_idx = 0; pkt_idx < num_packets; ++pkt_idx) {
    //     int start_index = pkt_idx * SCAN_PACKET_SIZE;
        
    //     if (start_index + SCAN_PACKET_SIZE > static_cast<int>(packet->data.size())) {
    //         break;
    //     }
        
    //     const LivoxLidarRawPoint* points = 
    //         reinterpret_cast<const LivoxLidarRawPoint*>(&packet->data[start_index]);
        
    //     // 计算相对时间戳（相对于点云开始时间，单位：纳秒）
    //     uint64_t relative_timestamp_ns = pkt_idx * PACKET_TIME_INTERVAL;
        
    //     for (int i = 0; i < POINT_NUMBER; ++i) {
    //         *iter_x = points[i].x / 1000.0f;
    //         *iter_y = points[i].y / 1000.0f;
    //         *iter_z = points[i].z / 1000.0f;
    //         *iter_intensity = static_cast<float>(points[i].reflectivity);
    //         // time 字段：相对于点云开始的时间偏移，单位：毫秒
    //         *iter_time = static_cast<float>((relative_timestamp_ns + i * point_interval_ns) / 1e6);
    //         *iter_ring = static_cast<uint16_t>(i % 4); // Livox Mid-360 有4线
            
    //         ++iter_x; ++iter_y; ++iter_z; ++iter_intensity; ++iter_time; ++iter_ring;
    //     }
    // }

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
        uint64_t point_interval = PACKET_TIME_INTERVAL * 100 / POINT_NUMBER;
        uint64_t timestamp = packet_time + i * point_interval * POINT_NUMBER;
        uint8_t line_num = 4; // Livox Mid-360 有4线
        PointXyzlt point = {};
        for (uint32_t i = 0; i < POINT_NUMBER; i++) {
            point.x = points[i].x / 1000.0;
            point.y = points[i].y / 1000.0;
            point.z = points[i].z / 1000.0;
            point.intensity = points[i].reflectivity;
            point.line = i % line_num;
            point.tag = 0;
            point.offset_time = timestamp + i * point_interval;
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
    return cloud_msg;

} // namespace

DEFINE_string(input_bag, "/work/share/lightning_ws/record_20241202_1.bag", "输入数据包");
DEFINE_string(config, "./config/default_nclt.yaml", "配置文件");
DEFINE_string(map_path, "/work/share/lightning_ws/new_map/", "地图路径");

/// 运行定位的测试
int main(int argc, char** argv) {
    std::cout << "------------------" << std::endl;
    //google::InitGoogleLogging(argv[0]);
    //FLAGS_colorlogtostderr = true;
    //    FLAGS_stderrthreshold = google::INFO; = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        std::cerr << "错误：未指定输入数据" << std::endl;
        return -1;
    }
    std::cout << "loc start" << std::endl;
    std::cout << "输入 bag: " << FLAGS_input_bag << std::endl;
    std::cout << "配置文件: " << FLAGS_config << std::endl;
    std::cout << "地图路径: " << FLAGS_map_path << std::endl;
    using namespace lightning;

    std::cout << "正在打开 bag 文件..." << std::endl;
    RosbagIO rosbag(FLAGS_input_bag);
    std::cout << "bag 文件打开成功" << std::endl;

    loc::Localization::Options options;
    options.online_mode_ = false;

    std::cout << "正在初始化定位系统..." << std::endl;
    loc::Localization loc(options);
    if (!loc.Init(FLAGS_config, FLAGS_map_path)) {
        std::cerr << "定位系统初始化失败！" << std::endl;
        return -1;
    }
    std::cout << "定位系统初始化成功" << std::endl;

    std::cout << "正在读取配置文件..." << std::endl;
    lightning::YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");
    std::string driver_lidar_packet_topic = yaml.GetValue<std::string>("common", "driver_lidar_packet_topic");
    std::string driver_imu_topic = yaml.GetValue<std::string>("common", "driver_imu_topic");
    std::cout << "Lidar topic: " << lidar_topic << std::endl;
    std::cout << "IMU topic: " << imu_topic << std::endl;
    std::cout << "Driver Lidar Packet topic: " << driver_lidar_packet_topic << std::endl;
    std::cout << "Driver IMU topic: " << driver_imu_topic << std::endl;

    // 添加消息缓冲，确保有足够的 IMU 数据在处理点云之前
    int imu_init_count = 0;
    const int MIN_IMU_INIT = 200;  // 至少处理200个IMU消息后再处理点云（约2秒数据）
    bool imu_initialized = false;
    std::cout << "需要先接收 " << MIN_IMU_INIT << " 个 IMU 消息后再处理点云数据" << std::endl;

    rosbag
        .AddImuHandle(imu_topic,
                      [&loc, &imu_init_count, &imu_initialized](IMUPtr imu) {
                          loc.ProcessIMUMsg(imu);
                          imu_init_count++;
                          if (imu_init_count == MIN_IMU_INIT) {
                              imu_initialized = true;
                              std::cout << "IMU 初始化完成，开始处理点云数据" << std::endl;
                          }
                          usleep(1000);
                          return true;
                      })
        .AddPointCloud2Handle(lidar_topic,
                              [&loc](sensor_msgs::PointCloud2ConstPtr cloud) {
                                  loc.ProcessLidarMsg(cloud);
                                  usleep(1000);
                                  return true;
                              })
        .AddLivoxCloudHandle("/livox/lidar",
                             [&loc](livox_ros_driver::CustomMsgConstPtr cloud) {
                                 loc.ProcessLivoxLidarMsg(cloud);
                                 usleep(1000);
                                 return true;
                             })
        .AddLivoxPacketHandle(driver_lidar_packet_topic,
                             [&loc, &imu_initialized](driver_msgs::LivoxPacketConstPtr packet) {
                                 if (imu_initialized) {
                                     // 解析 LivoxPacket 转换为 PointCloud2
                                     auto cloud = ParseLivoxPacket(packet);
                                     if (cloud) {
                                        //  std::cout << "处理 LivoxPacket: " << cloud->width << " 个点" << std::endl;
                                         loc.ProcessLidarMsg(cloud);
                                     }
                                 }
                                 usleep(1000);
                                 return true;
                             })
        .AddDriverImuHandle(driver_imu_topic,
                            [&loc, &imu_init_count, &imu_initialized](driver_msgs::SensorImuInfoConstPtr imu_msg) {
                                static int imu_count = 0;
                                IMUPtr imu = std::make_shared<IMU>();
                                imu->timestamp = ToSec(imu_msg->header.stamp);
                                imu->linear_acceleration = Vec3d(imu_msg->acce_x, imu_msg->acce_y, imu_msg->acce_z);
                                imu->angular_velocity = Vec3d(imu_msg->gyro_x, imu_msg->gyro_y, imu_msg->gyro_z);
                                loc.ProcessIMUMsg(imu);
                                imu_init_count++;
                                if (imu_init_count == MIN_IMU_INIT) {
                                    imu_initialized = true;
                                    std::cout << "IMU 初始化完成，开始处理点云数据" << std::endl;
                                }
                                if (++imu_count % 500 == 0) {
                                    // std::cout << "已处理 " << imu_count << " 条 IMU 消息" << std::endl;
                                }
                                usleep(1000);
                                return true;
                            })
        .Go();

    std::cout << "Bag 文件处理完成" << std::endl;
    Timer::PrintAll();
    std::cout << "正在结束定位系统..." << std::endl;
    loc.Finish();

    std::cout << "程序执行完成！" << std::endl;

    return 0;
}