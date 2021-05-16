/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data transfrom classes
*/
#include <iostream>
#include <fstream>
#include "velodyne_pointcloud/transform.h"
#include "velodyne_pointcloud/pointcloudXYZIR.h"
#include <time.h>

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

  void Transfrom::processScan(const std::vector<velodyne_driver::VelodynePacket> &scanMsg)
  {
    // allocate a point cloud with same time and frame ID as raw data
    PointcloudXYZIR outMsg;
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    time(&outMsg.t_stamp);
    // outMsg.pc->header.frame_id = scanMsg->header.frame_id;
    // outMsg.pc->height = 1;

    outMsg.pc.reserve(scanMsg.size() * data_->scansPerPacket());

    //        std::ofstream fd_t;
    //        fd_t.open("test.txt");

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg.size(); ++i)
    {
      data_->unpack(scanMsg[i], outMsg);
    }

    //for (size_t i = 0; i < 10; ++i)
//    for (size_t i = 0; i < outMsg.pc.size(); ++i)
    for (const auto& laser : outMsg.pc)
    {
      if (laser.x  != 0 && laser.y  != 0 && laser.z  != 0)
        {
          std::cout << "the x is " << laser.x << std::endl;
          std::cout << "the y is " << laser.y << std::endl;
          std::cout << "the z is " << laser.z << std::endl;
          std::cout << "the intensity is " << laser.intensity << std::endl;
          std::cout << "the distance is " << laser.distance << std::endl;
          std::cout << "the azimuth is " << laser.azimuth << std::endl;
          std::cout << std::endl;
        }
    }
    //        fd_t.close();

    std::cout<< "Publishing " << scanMsg.size() * data_->scansPerPacket() << " Velodyne points, time: " << asctime(gmtime(&outMsg.t_stamp))<<std::endl;
  }

}
