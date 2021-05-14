/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR transform classes
*/
#pragma once

#include "velodyne_pointcloud/rawdata.h"
#include <vector>
#include <memory>

namespace velodyne_pointcloud
{
  class Transfrom
  {
  public:
    Transfrom();
    ~Transfrom();
    void processScan(const std::vector<velodyne_driver::VelodynePacket> &scan_packets);
    std::shared_ptr<velodyne_rawdata::RawData> data_;
  };
}
