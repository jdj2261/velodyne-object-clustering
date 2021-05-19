/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR transform classes
*/
#pragma once

#include <vector>
#include <memory>
#include "velodyne_pcl/viewer.h"
#include "velodyne_pointcloud/pointcloudXYZIR.h"
#include "velodyne_pointcloud/rawdata.h"
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>

namespace velodyne_pointcloud
{
  class Transfrom
  {
  public:
    Transfrom();
    ~Transfrom();
    void processScan(const std::vector<velodyne_driver::VelodynePacket> &scan_packets,
                     velodyne_pcl::pointcloud::Ptr &cloud);
    std::shared_ptr<velodyne_rawdata::RawData> data_;
  };
}
