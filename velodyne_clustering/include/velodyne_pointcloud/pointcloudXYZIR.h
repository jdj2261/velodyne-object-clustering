/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data pointcloudXYZIR classes
*/
#pragma once

#include "velodyne_pointcloud/rawdata.h"

namespace velodyne_pointcloud
{
    class PointcloudXYZIR : public velodyne_rawdata::DataContainerBase
    {
    public:

        velodyne_rawdata::VPointCloud pc;
        time_t t_stamp;

        //PointcloudXYZIR(){}

        virtual void addPoint(
                const float& x,
                const float& y,
                const float& z,
                const u_int16_t& ring,
                const float& azimuth,
                const float& distance,
                const float& intensity);
    };

}  // namespace velodyne_pointcloud
