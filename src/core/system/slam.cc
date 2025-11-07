//
// Created by xiang on 25-5-6.
//

#include <csignal>

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "core/system/comm.h"
#include "core/system/livox_lidar_def.h"

using namespace livox_ros;

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        //LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    if (options_.with_loop_closing_) {
        //LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
        //LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次重绘
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        //LOG(INFO) << "online mode, creating ros node ... ";

        /// subscribers
        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        driver_imu_topic_ = yaml["common"]["driver_imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();
        livox_packet_topic_ = yaml["common"]["driver_lidar_packet_topic"].as<std::string>();

        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
            imu_topic_, 1000, [this](const sensor_msgs::ImuConstPtr& msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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

        savemap_service_ = nh_.advertiseService(
            "lightning/save_map", &SlamSystem::SaveMap, this);

        //LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

bool SlamSystem::SaveMap(SaveMapService::Request& request,
                         SaveMapService::Response& response) {
    map_name_ = request.map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response.response = 0;
    return true;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    //LOG(INFO) << "slam map saving to " << save_path;

    // 检查是否有关键帧
    auto all_keyframes = lio_->GetAllKeyframes();
    if (all_keyframes.empty()) {
        std::cout << "警告：没有生成任何关键帧，无法保存地图" << std::endl;
        return;
    }

    std::cout << "共有 " << all_keyframes.size() << " 个关键帧" << std::endl;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = all_keyframes.front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            //LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    //LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessIMU(const driver_msgs::SensorImuInfoConstPtr& msg) {
    if (running_ == false) {
        return;
    }

    // 将 driver_msgs::SensorImuInfo 转换为内部 IMU 格式
    IMUPtr imu = std::make_shared<IMU>();
    imu->timestamp = ToSec(msg->header.stamp);
    
    // 从 driver_msgs 中提取加速度和角速度数据
    // driver_msgs::SensorImuInfo 使用 acce_x/y/z 和 gyro_x/y/z 字段
    imu->linear_acceleration = Vec3d(msg->acce_x, msg->acce_y, msg->acce_z);
    imu->angular_velocity = Vec3d(msg->gyro_x, msg->gyro_y, msg->gyro_z);
    
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::ProcessLidar(const livox_ros_driver::CustomMsgConstPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

// void SlamSystem::ProcessLidar(const driver_msgs::CustomMsgConstPtr& cloud) {
//     if (running_ == false) {
//         return;
//     }

//     lio_->ProcessPointCloud2(cloud);
//     lio_->Run();

//     auto kf = lio_->GetKeyframe();
//     if (kf != cur_kf_) {
//         cur_kf_ = kf;
//     } else {
//         return;
//     }

//     if (cur_kf_ == nullptr) {
//         return;
//     }

//     if (options_.with_loop_closing_) {
//         lc_->AddKF(cur_kf_);
//     }

//     if (options_.with_gridmap_) {
//         g2p5_->PushKeyframe(cur_kf_);
//     }

//     if (ui_) {
//         ui_->UpdateKF(cur_kf_);
//     }
// }

void SlamSystem::ProcessLivoxPacket(const driver_msgs::LivoxPacketConstPtr& packet) {
    if (running_ == false) {
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
    
    static bool first_cloud_check = true;
    if (first_cloud_check) {
        printf("生成点云：%d 个点\n", total_points);
        first_cloud_check = false;
    }
    
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
    
    // 检查最后一个点的时间
    static int cloud_count = 0;
    cloud_count++;
    if (cloud_count <= 3) {
        // 获取最后一个点的时间
        sensor_msgs::PointCloud2ConstIterator<float> check_time(*cloud, "time");
        check_time += (total_points - 1);  // 移动到最后一个点
        printf("点云 #%d: 总点数=%d, 最后一个点的time=%.6f ms\n", 
               cloud_count, total_points, *check_time);
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
    
    // 处理解析后的点云
    lio_->ProcessPointCloud2(cloud_msg);
    */
   lio_->ProcessPointCloud2(cloud);
    bool lio_success = lio_->Run();
    
    static int process_count = 0;
    static int kf_count = 0;
    process_count++;
    
    if (!lio_success && process_count == 1) {
        std::cout << "警告：LIO Run() 返回 false" << std::endl;
    }

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_) {
        ros::spin();
    }
}

}  // namespace lightning
