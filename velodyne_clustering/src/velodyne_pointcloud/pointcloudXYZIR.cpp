/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data pointcloudXYZIR classes
*/
#pragma once
#include "velodyne_pointcloud/pointcloudXYZIR.h"

namespace velodyne_pointcloud
{
  void PointcloudXYZIR::addPoint(const float& x,
                                 const float& y,
                                 const float& z,
                                 const u_int16_t& ring,
                                 const u_int16_t& azimuth,
                                 const float& distance,
                                 const float& intensity)
  {
    // convert polar coordinates to Euclidean XYZ
    velodyne_rawdata::VPoint point;

    point.ring = ring;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    point.azimuth = azimuth;
    // append this point to the cloud
    pc.push_back(point);
  }
}
