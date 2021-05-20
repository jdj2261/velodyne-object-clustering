/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data calibration classes
*/

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <limits>
#include "velodyne_pointcloud/calibration.h"

namespace velodyne_pointcloud
{
    void Calibration::initParams(Calibration &calibration)
    {
        const int num_lasers = 16;
        float distance_resolution_m = 0.002;
        calibration.laser_corrections.clear();
        calibration.num_lasers = num_lasers;
        calibration.distance_resolution_m = distance_resolution_m;
        calibration.laser_corrections.resize(num_lasers);

        calibration.laser_corrections[0].vert_correction = -0.2617993877991494;
        calibration.laser_corrections[1].vert_correction = 0.017453292519943295;
        calibration.laser_corrections[2].vert_correction = -0.22689280275926285;
        calibration.laser_corrections[3].vert_correction = 0.05235987755982989;
        calibration.laser_corrections[4].vert_correction = -0.19198621771937624;
        calibration.laser_corrections[5].vert_correction = 0.08726646259971647;
        calibration.laser_corrections[6].vert_correction = -0.15707963267948966;
        calibration.laser_corrections[7].vert_correction = 0.12217304763960307;
        calibration.laser_corrections[8].vert_correction = -0.12217304763960307;
        calibration.laser_corrections[9].vert_correction = 0.15707963267948966;
        calibration.laser_corrections[10].vert_correction = -0.08726646259971647;
        calibration.laser_corrections[11].vert_correction = 0.19198621771937624;
        calibration.laser_corrections[12].vert_correction = -0.05235987755982989;
        calibration.laser_corrections[13].vert_correction = 0.2268928027592628;
        calibration.laser_corrections[14].vert_correction = -0.017453292519943295;
        calibration.laser_corrections[15].vert_correction = 0.2617993877991494;

        for(int i=0; i < num_lasers; i++)
        {
            calibration.laser_corrections[i].dist_correction = 0.0;
            calibration.laser_corrections[i].dist_correction_x = 0.0;
            calibration.laser_corrections[i].dist_correction_y = 0.0;
            calibration.laser_corrections[i].focal_distance = 0.0;
            calibration.laser_corrections[i].focal_slope = 0.0;
            calibration.laser_corrections[i].horiz_offset_correction = 0.0;
            calibration.laser_corrections[i].vert_offset_correction = 0.0;
            calibration.laser_corrections[i].rot_correction = 0.0;
            calibration.laser_corrections[i].cos_rot_correction = cosf(0.0);
            calibration.laser_corrections[i].sin_rot_correction = sinf(0.0);
            calibration.laser_corrections[i].cos_vert_correction = cosf(calibration.laser_corrections[i].vert_correction);
            calibration.laser_corrections[i].sin_vert_correction = sinf(calibration.laser_corrections[i].vert_correction);
            calibration.laser_corrections[i].min_intensity = 0;
            calibration.laser_corrections[i].max_intensity = 255;

        }

        double next_angle = -std::numeric_limits<double>::infinity();
        for (int ring = 0; ring < num_lasers; ++ring)
        {
            // find minimum remaining vertical offset correction
            double min_seen = std::numeric_limits<double>::infinity();
            int next_index = num_lasers;
            for (int j = 0; j < num_lasers; ++j)
            {
                double angle = calibration.laser_corrections[j].vert_correction;
                if (next_angle < angle && angle < min_seen)
                {
                    min_seen = angle;
                    next_index = j;
                }
            }

            if (next_index < num_lasers) {    // anything found in this ring?

                // store this ring number with its corresponding laser number
                calibration.laser_corrections[next_index].laser_ring = ring;
                next_angle = min_seen;
//                std::cout<<"laser_ring = " <<ring<< ", angle = " << next_angle<<std::endl;
            }
        }
    }
} // velodyne_pointcloud namespace
