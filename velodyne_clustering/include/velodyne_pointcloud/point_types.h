/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data point_types classes
*/
#pragma once

#include <unistd.h>
#include <stdint.h>
namespace velodyne_pointcloud
{
/** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
      //PCL_ADD_POINT4D;                    // quad-word XYZ
      float data[4];
      float x;
      float y;
      float z;
      float intensity;                 ///< laser intensity reading
      float distance;
      float azimuth;
      uint16_t ring;                      ///< laser ring number
  } ;
}
