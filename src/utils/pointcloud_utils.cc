//
// Created by xiang on 25-4-22.
//

#include "utils/pointcloud_utils.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace lightning {

/// 体素滤波
CloudPtr VoxelGrid(CloudPtr cloud, float voxel_size) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    return output;
}


}  // namespace lightning