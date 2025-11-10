//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>

//using namespace google::logging::internal;

#include "core/system/slam.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"


DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    //google::InitGoogleLogging(argv[0]);
    //FLAGS_colorlogtostderr = true;
    //    FLAGS_stderrthreshold = google::INFO; = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    using namespace lightning;

    /// 需要ros::init
    ros::init(argc, argv, "lightning_slam");

    SlamSystem::Options options;
    options.online_mode_ = true;

    std::cout << "=== Lightning SLAM Online 启动 ===" << std::endl;
    std::cout << "配置文件: " << FLAGS_config << std::endl;

    SlamSystem slam(options);
    if (!slam.Init(FLAGS_config)) {
        std::cout << "错误: 初始化 SLAM 失败" << std::endl;
        //LOG(ERROR) << "failed to init slam";
        return -1;
    }

    std::cout << "SLAM 初始化成功，等待传感器数据..." << std::endl;

    slam.StartSLAM("new_map");
    slam.Spin();

    std::cout << "SLAM 运行结束" << std::endl;

    Timer::PrintAll();

    ros::shutdown();

    std::cout << "程序正常退出" << std::endl;
    //LOG(INFO) << "done";

    return 0;
}