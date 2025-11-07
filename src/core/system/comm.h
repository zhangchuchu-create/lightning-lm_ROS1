#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace livox_ros {

/** Max lidar data source num */
const uint8_t kMaxSourceLidar = 32;

/** Eth packet relative info parama */
const uint32_t kMaxPointPerEthPacket = 100;
const uint32_t kMinEthPacketQueueSize = 32;     /**< must be 2^n */
const uint32_t kMaxEthPacketQueueSize = 131072; /**< must be 2^n */
const uint32_t kImuEthPacketQueueSize = 256;

/** Max packet length according to Ethernet MTU */
const uint32_t KEthPacketMaxLength = 1500;
const uint32_t KEthPacketHeaderLength = 18; /**< (sizeof(LivoxEthPacket) - 1) */
const uint32_t KCartesianPointSize = 13;
const uint32_t KSphericalPointSzie = 9;

const uint64_t kRosTimeMax = 4294967296000000000; /**< 2^32 * 1000000000ns */
const int64_t kPacketTimeGap = 1000000;           /**< 1ms = 1000000ns */
/**< the threshold of packet continuous */
const int64_t kMaxPacketTimeGap = 1700000;
/**< the threshold of device disconect */
const int64_t kDeviceDisconnectThreshold = 1000000000;
const uint32_t kNsPerSecond = 1000000000;               /**< 1s  = 1000000000ns */
const uint32_t kNsTolerantFrameTimeDeviation = 1000000; /**< 1ms  = 1000000ns */
const uint32_t kRatioOfMsToNs = 1000000;                /**< 1ms  = 1000000ns */

const int kPathStrMinSize = 4;   /**< Must more than 4 char */
const int kPathStrMaxSize = 256; /**< Must less than 256 char */
const int kBdCodeSize = 15;

const uint32_t kPointXYZRSize = 16;
const uint32_t kPointXYZRTRSize = 18;

const double PI = 3.14159265358979323846;

constexpr uint32_t kMaxBufferSize = 0x8000;  // 32k bytes

/** Device Line Number **/
const uint8_t kLineNumberDefault = 1;
const uint8_t kLineNumberMid360 = 4;
const uint8_t kLineNumberHAP = 6;

// SDK related
typedef enum {
    kIndustryLidarType = 1,
    kVehicleLidarType = 2,
    kDirectLidarType = 4,
    kLivoxLidarType = 8
} LidarProtoType;

// SDK related
/** Timestamp sync mode define. */
typedef enum {
    kTimestampTypeNoSync = 0,    /**< No sync signal mode. */
    kTimestampTypeGptpOrPtp = 1, /**< gPTP or PTP sync mode */
    kTimestampTypeGps = 2        /**< GPS sync mode. */
} TimestampType;

/** Lidar connect state */
typedef enum {
    kConnectStateOff = 0,
    kConnectStateOn = 1,
    kConnectStateConfig = 2,
    kConnectStateSampling = 3,
} LidarConnectState;

/** Device data source type */
typedef enum {
    kSourceRawLidar = 0, /**< Data from raw lidar. */
    kSourceRawHub = 1,   /**< Data from lidar hub. */
    kSourceLvxFile,      /**< Data from parse lvx file. */
    kSourceUndef,
} LidarDataSourceType;

typedef enum { kCoordinateCartesian = 0, kCoordinateSpherical } CoordinateType;

typedef enum {
    kConfigDataType = 1 << 0,
    kConfigScanPattern = 1 << 1,
    kConfigBlindSpot = 1 << 2,
    kConfigDualEmit = 1 << 3,
    kConfigUnknown
} LivoxLidarConfigCodeBit;

typedef enum {
    kNoneExtrinsicParameter,
    kExtrinsicParameterFromLidar,
    kExtrinsicParameterFromXml
} ExtrinsicParameterType;

typedef struct {
    uint8_t lidar_type{};
} LidarSummaryInfo;

/** 8bytes stamp to uint64_t stamp */
typedef union {
    struct {
        uint32_t low;
        uint32_t high;
    } stamp_word;

    uint8_t stamp_bytes[8];
    int64_t stamp;
} LdsStamp;

#pragma pack(1)

typedef struct {
    float x;            /**< X axis, Unit:m */
    float y;            /**< Y axis, Unit:m */
    float z;            /**< Z axis, Unit:m */
    float reflectivity; /**< Reflectivity   */
    uint8_t tag;        /**< Livox point tag   */
    uint8_t line;       /**< Laser line id     */
    double timestamp;   /**< Timestamp of point*/
} LivoxPointXyzrtlt;

typedef struct {
    float x;
    float y;
    float z;
    float intensity;
    uint8_t tag;
    uint8_t line;
    uint64_t offset_time;
} PointXyzlt;

typedef struct {
    uint32_t handle;
    uint8_t lidar_type;  ////refer to LivoxLidarType
    uint32_t points_num;
    PointXyzlt *points;
} PointPacket;

typedef struct {
    uint64_t base_time[kMaxSourceLidar]{};
    uint8_t lidar_num{};
    PointPacket lidar_point[kMaxSourceLidar]{};
} PointFrame;

#pragma pack()

typedef struct {
    LidarProtoType lidar_type;
    uint32_t handle;
    uint64_t base_time;
    uint32_t points_num;
    std::vector<PointXyzlt> points;
} StoragePacket;

typedef struct {
    LidarProtoType lidar_type;
    uint32_t handle;
    bool extrinsic_enable;
    uint32_t point_num;
    uint8_t data_type;
    uint8_t line_num;
    uint64_t time_stamp;
    uint64_t point_interval;
    std::vector<uint8_t> raw_data;
} RawPacket;

typedef struct {
    StoragePacket *storage_packet;
    volatile uint32_t rd_idx;
    volatile uint32_t wr_idx;
    uint32_t mask;
    uint32_t size; /**< must be power of 2. */
} LidarDataQueue;

/*****************************/
/* About Extrinsic Parameter */
typedef struct {
    float roll;  /**< Roll angle, unit: degree. */
    float pitch; /**< Pitch angle, unit: degree. */
    float yaw;   /**< Yaw angle, unit: degree. */
    int32_t x;   /**< X translation, unit: mm. */
    int32_t y;   /**< Y translation, unit: mm. */
    int32_t z;   /**< Z translation, unit: mm. */
} ExtParameter;

typedef float TranslationVector[3]; /**< x, y, z translation, unit: mm. */
typedef float RotationMatrix[3][3];

typedef struct {
    TranslationVector trans;
    RotationMatrix rotation;
} ExtParameterDetailed;

typedef struct {
    LidarProtoType lidar_type;
    uint32_t handle;
    ExtParameter param;
} LidarExtParameter;

/** Configuration in json config file for livox lidar */
typedef struct {
    char broadcast_code[16];
    bool enable_connect;
    bool enable_fan;
    uint32_t return_mode;
    uint32_t coordinate;
    uint32_t imu_rate;
    uint32_t extrinsic_parameter_source;
    bool enable_high_sensitivity;
} UserRawConfig;

typedef struct {
    bool enable_fan;
    uint32_t return_mode;
    uint32_t coordinate; /**< 0 for CartesianCoordinate; others for SphericalCoordinate. */
    uint32_t imu_rate;
    uint32_t extrinsic_parameter_source;
    bool enable_high_sensitivity;
    volatile uint32_t set_bits;
    volatile uint32_t get_bits;
} UserConfig;

typedef struct {
    uint32_t handle;
    int8_t pcl_data_type;
    int8_t pattern_mode;
    int32_t blind_spot_set;
    int8_t dual_emit_en;
    ExtParameter extrinsic_param;
    volatile uint32_t set_bits;
    volatile uint32_t get_bits;
} UserLivoxLidarConfig;

constexpr uint32_t kMaxProductType = 10;
constexpr uint32_t kDeviceTypeLidarMid70 = 6;

/***********************************/
/* Global function for general use */
bool IsFilePathValid(const char *path_str);
uint32_t CalculatePacketQueueSize(const double publish_freq);
std::string IpNumToString(uint32_t ip_num);
uint32_t IpStringToNum(std::string ip_string);
std::string ReplacePeriodByUnderline(std::string str);

/// 创建livox的pcl形式数据格式
struct PointXYZRTLT {
    PCL_ADD_POINT4D;                 // quad-word XYZ
    float reflectivity;              // reflectivity
    uint8_t tag;                     // tag
    uint8_t line;                    // laser line id
    double timestamp;                // 每个点的时间戳，定位运动补偿需要
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;
}  // namespace livox_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::PointXYZRTLT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, reflectivity, reflectivity)(
                                      uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))
