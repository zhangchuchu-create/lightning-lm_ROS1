cmake_minimum_required(VERSION 3.16)

project(lightning)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# set to release by default
set(CMAKE_BUILD_TYPE Release)

# -------------------------------------------------------------
# catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_srvs
    tf
    tf_conversions
    pcl_conversions
    pcl_ros
    message_filters
    visualization_msgs
    message_generation
    common_msgs
    driver_msgs
)

# msg 和 srv
file(GLOB_RECURSE MY_SRV_FILES srv/*.srv)

add_service_files(
    FILES
    LocCmd.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_srvs
    tf
    tf_conversions
    pcl_conversions
    pcl_ros
    message_filters
    visualization_msgs
    message_runtime
    livox_ros_driver
    
    common_msgs
    driver_msgs
)

# -------------------------------------------------------------
# 导入第三方库，同时做一些全局编译选项配置
add_definitions("-DPCL_NO_PRECOMPILE")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp -fPIC -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O2 -fopenmp -fPIC -g -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -ggdb -g -fopenmp -fPIC -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -g")


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Use catkin's default output directories for ROS compatibility
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/packages.cmake)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#add_subdirectory(thirdparty/livox_ros_driver)

add_subdirectory(src)
