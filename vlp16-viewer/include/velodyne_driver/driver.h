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
        VelodyneDriver(const std::string& address,
                       const std::string& port,
                       const std::string& pcap,
                       bool is_saved):
            address_(address),
            port_(static_cast<u_int16_t>(std::stoi(port))),
            pcap_file_(pcap),
            is_saved_(is_saved)
        {
            init();
        }

        ~VelodyneDriver() noexcept
        {
            input_.reset();
            trans_.reset();
        }
        void init();
        bool poll(velodyne_pcl::pointcloud::Ptr &cloud);

    private:
        struct
        {
            std::string   model;
            int           npackets;
            double        rpm;
            int           cut_angle;
            double        time_offset;
        } config_;

        std::string address_;
        u_int16_t port_;
        std::string pcap_file_;
        bool is_saved_;

        int last_azimuth_;
        std::shared_ptr<Input> input_;
        std::shared_ptr<velodyne_pointcloud::Transfrom> trans_;
    };
}
