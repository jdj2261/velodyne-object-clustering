/** @author Dae Jong Jin
 ** @date   2021. 05. 14
 ** @file   Velodyne 3D LIDAR driver classes
*/
#pragma once

#include <string>
#include <memory>
#include "input.h"
#include "velodyne_pointcloud/transform.h"

namespace velodyne_driver
{
    class VelodyneDriver
    {
    public:
        VelodyneDriver() = default;
        VelodyneDriver(const std::string &address,
                       const std::string &port,
                       const std::string &pcap,
                       bool is_saved)
            : address_(address),
              port_(static_cast<u_int16_t>(std::stoi(port))),
              pcap_file_(pcap),
              is_saved_(is_saved),
              packet_rate_(754.0),
              cut_angle_(-0.01),
              last_azimuth_(-1)
        {
            init_driver();
        }

        ~VelodyneDriver() noexcept
        {
            input_.reset();
            trans_.reset();
        }
        void init_driver();
        void print_driver_info(const std::string &name);
        bool poll(velodyne_pcl::pointcloud::Ptr &cloud);

    private:
        struct Config
        {
            std::string   model;
            u_int16_t      rpm;
            u_int16_t     npackets;
            double        cut_angle;
            double        time_offset;
        } config_;

        std::string address_;
        u_int16_t port_;
        std::string pcap_file_;
        bool is_saved_;
        double packet_rate_;
        double cut_angle_;
        int last_azimuth_;
        std::shared_ptr<Input> input_;
        std::shared_ptr<velodyne_pointcloud::Transfrom> trans_;
    };
}
