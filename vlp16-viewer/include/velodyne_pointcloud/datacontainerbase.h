/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data containerbase classes
*/
#pragma once

#include <stdint.h>

namespace velodyne_rawdata
{
  class DataContainerBase
  {
  public:
    virtual void addPoint(
        const float& x,
        const float& y,
        const float& z,
        const uint16_t& ring,
        const float& azimuth,
        const float& distance,
        const float& intensity) = 0;
    virtual void newLine() = 0;
  };
}

