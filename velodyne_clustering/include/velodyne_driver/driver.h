/** @author Dae Jong Jin
 ** @date   2021. 05. 14
 ** @file   Velodyne 3D LIDAR driver classes
*/
#pragma once

#include <string>
#include "input.h"
#include "velodyne_pointcloud/transform.h"
#include <memory>

namespace velodyne_driver
{
  class VelodyneDriver
  {
  public:

    VelodyneDriver();
    ~VelodyneDriver();
    bool poll();

  private:

    struct
    {
      std::string model;               ///< device model name
      int    npackets;                 ///< number of packets to collect
      double rpm;                      ///< device rotation rate (RPMs)
      int cut_angle;                   ///< cutting angle in 1/100°
      double time_offset;              ///< time in seconds added to each velodyne time stamp
    } config_;

    int last_azimuth_;
    std::shared_ptr<Input> input_;
    std::shared_ptr<velodyne_pointcloud::Transfrom> trans_;
  };
}
