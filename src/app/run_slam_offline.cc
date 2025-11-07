//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <iomanip>

// using namespace google::logging::internal;

#include "core/system/slam.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

// using namespace google::logging::internal;

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    // google::InitGoogleLogging(argv[0]);
    // FLAGS_colorlogtostderr = true;
    //     FLAGS_stderrthreshold = google::INFO; = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);

    // 初始化 ROS
    ros::init(argc, argv, "run_slam_offline");

    if (FLAGS_input_bag.empty()) {
        // LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    SlamSystem::Options options;
    options.online_mode_ = false;

    SlamSystem slam(options);

    /// 实时模式好像掉帧掉的比较厉害？

    if (!slam.Init(FLAGS_config)) {
        // LOG(ERROR) << "failed to init slam";
        return -1;
    }
    // 输出配置文件地址
    std::cout << "配置文件: " << FLAGS_config << std::endl;
    // 输出bag文件地址
    std::cout << "输入 bag: " << FLAGS_input_bag << std::endl;

    slam.StartSLAM("new_map");

    lightning::YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");
    std::string driver_imu_topic = yaml.GetValue<std::string>("common", "driver_imu_topic");
    std::string driver_lidar_packet_topic = yaml.GetValue<std::string>("common", "driver_lidar_packet_topic");

    // 添加消息缓冲，确保有足够的 IMU 数据在处理点云之前
    int imu_init_count = 0;
    const int MIN_IMU_INIT = 300;  // 至少处理300个IMU消息后再处理点云（约3秒数据）
    bool imu_initialized = false;
    double last_imu_time = 0;
    std::cout << "需要先接收 " << MIN_IMU_INIT << " 个 IMU 消息后再处理点云数据" << std::endl;

    rosbag
        /// IMU 的处理
        .AddImuHandle(imu_topic,
                      [&slam, &imu_init_count, &imu_initialized, &last_imu_time](IMUPtr imu) {
                          slam.ProcessIMU(imu);
                          last_imu_time = imu->timestamp;
                          imu_init_count++;
                          if (imu_init_count == MIN_IMU_INIT) {
                              imu_initialized = true;
                              std::cout << "IMU 初始化完成（最后 IMU 时间: " << std::fixed << std::setprecision(6)
                                        << last_imu_time << "），开始处理点云数据" << std::endl;
                          }
                          return true;
                      })

        /// driver_msgs IMU 的处理
        .AddDriverImuHandle(
            driver_imu_topic,
            [&slam, &imu_init_count, &imu_initialized, &last_imu_time](driver_msgs::SensorImuInfoConstPtr msg) {
                slam.ProcessIMU(msg);
                last_imu_time = lightning::ToSec(msg->header.stamp);
                imu_init_count++;
                if (imu_init_count == MIN_IMU_INIT) {
                    imu_initialized = true;
                    std::cout << "IMU 初始化完成（最后 IMU 时间: " << std::fixed << std::setprecision(6)
                              << last_imu_time << "），开始处理点云数据" << std::endl;
                }
                return true;
            })

        /// lidar 的处理
        .AddPointCloud2Handle(lidar_topic,
                              [&slam](sensor_msgs::PointCloud2ConstPtr msg) {
                                  slam.ProcessLidar(msg);
                                  return true;
                              })
        /// livox 的处理
        .AddLivoxCloudHandle("/livox/lidar",
                             [&slam](livox_ros_driver::CustomMsgConstPtr cloud) {
                                 slam.ProcessLidar(cloud);
                                 return true;
                             })
        /// livox 的处理
        .AddLivoxCloudHandle("/livox_lidar_custom",
                             [&slam](livox_ros_driver::CustomMsgConstPtr cloud) {
                                 slam.ProcessLidar(cloud);
                                 return true;
                             })
        /// LivoxPacket 的处理
        .AddLivoxPacketHandle(driver_lidar_packet_topic,
                              [&slam, &imu_initialized](driver_msgs::LivoxPacketConstPtr packet) {
                                  if (imu_initialized) {
                                      slam.ProcessLivoxPacket(packet);
                                  }
                                  return true;
                              })
        .Go();

    std::cout << "Bag 处理完成，准备保存地图..." << std::endl;

    try {
        slam.SaveMap("");
        std::cout << "地图保存完成" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "保存地图时出错: " << e.what() << std::endl;
    }

    Timer::PrintAll();

    std::cout << "程序正常结束" << std::endl;
    // LOG(INFO) << "done";

    return 0;
}