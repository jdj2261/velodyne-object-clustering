/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data transfrom classes
*/
#include <iostream>
#include <fstream>
#include <time.h>
#include "velodyne_pointcloud/transform.h"

namespace velodyne_pointcloud
{
    Transfrom::Transfrom()
    {
        //std::shared_ptr<velodyne_rawdata::RawData> data_temp (new velodyne_rawdata::RawData);
        data_.reset(new velodyne_rawdata::RawData);
        data_->setup();
        data_->setParameters(0.9,200,0,0);
    }

    Transfrom::~Transfrom()
    {
        data_.reset();
    }

    void Transfrom::processScan(const std::vector<velodyne_driver::VelodynePacket> &scanMsg,
                                velodyne_pcl::pointcloud::Ptr &cloud)
    {
        // allocate a point cloud with same time and frame ID as raw data
        PointcloudXYZI outMsg;
        // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
        time(&outMsg.t_stamp);
        velodyne_pcl::PointXYZI point;
        outMsg.pc.reserve(scanMsg.size() * data_->scansPerPacket());

        // process each packet provided by the driver
        for (size_t i = 0; i < scanMsg.size(); ++i)
        {
            data_->unpack(scanMsg[i], outMsg);
        }

        //for (size_t i = 0; i < 10; ++i)
        //    for (size_t i = 0; i < outMsg.pc.size(); ++i)
        for (const auto& laser : outMsg.pc)
        {
            if (laser.x != 0 && laser.y != 0 && laser.z != 0)
            {
//                std::cout << "the x is " << laser.x << std::endl;
//                std::cout << "the y is " << laser.y << std::endl;
//                std::cout << "the z is " << laser.z << std::endl;
//                std::cout << "the ring is " << laser.ring << std::endl;
//                std::cout << "the intensity is " << laser.intensity << std::endl;
//                std::cout << "the distance is " << laser.distance << std::endl;
//                std::cout << "the azimuth is " << laser.azimuth << std::endl;
//                std::cout << std::endl;

                point.x = laser.x;
                point.y = laser.y;
                point.z = laser.z;
                point.intensity = laser.intensity;

                cloud->push_back(point);
            }
        }
        cloud->width = static_cast<int>(cloud->points.size());
        //        cout << "Number of lasers:" << lasers.size() << endl;
        cloud->height = 1;
        cloud->points.resize (cloud->width * cloud->height);
        cloud->is_dense = true;
        //    std::cout<< "Publishing " << scanMsg.size() * data_->scansPerPacket() << " Velodyne points, time: " << asctime(gmtime(&outMsg.t_stamp))<<std::endl;
    }

}
