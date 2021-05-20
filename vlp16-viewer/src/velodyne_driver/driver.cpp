/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data driver classes
*/

#include "velodyne_driver/driver.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <fcntl.h>

namespace velodyne_driver
{
    void VelodyneDriver::init_driver()
    {
        config_.model = "VLP16";
        std::string model_full_name("VLP-16");
        std::string deviceName(std::string("Velodyne ") + model_full_name);
        config_.rpm = 600;
        double frequency(config_.rpm / 60.0);
        config_.npackets = static_cast<u_int16_t>(ceil(packet_rate_ / frequency)) ;
        // Convert cut_angle from radian to one-hundredth degree,
        config_.cut_angle = int((cut_angle_*360/(2*M_PI))*100);
        print_driver_info(deviceName);
        config_.time_offset = 0.0;

        if (pcap_file_ != "")
            input_.reset(new velodyne_driver::InputPCAP(port_, packet_rate_, pcap_file_));
        else
            input_.reset(new velodyne_driver::InputSocket(port_));
        trans_.reset(new velodyne_pointcloud::Transfrom);
    }

    void VelodyneDriver::print_driver_info(const std::string &name)
    {
        std::cout << "******** velodyne driver Info *********" << std::endl;
        std::cout << "Name : " << name << std::endl;
        std::cout <<"publishing " << config_.npackets << " packets per scan"<<std::endl;
        if (cut_angle_ < 0.0)
            std::cout<<"Cut at specific angle feature deactivated."<< std::endl;
        else if (cut_angle_ < (2 * std::asin(1)))
        {
            std::cout<<"Cut at specific angle feature activated. "<<std::endl;
            std::cout<<"Cutting velodyne points always at %f " << cut_angle_ << " rad."<<std::endl;
        }
        else
        {
            std::cout<<"cut_angle parameter is out of range. Allowed range is "<<std::endl;
            std::cout<<"between 0.0 and 2*PI or negative values to deactivate this feature."<<std::endl;
            cut_angle_ = -0.01;
        }
    }

    bool VelodyneDriver::poll(velodyne_pcl::pointcloud::Ptr &cloud)
    {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        std::vector<VelodynePacket> scan_packets;
        //InputSocket* input_ = new InputSocket(DATA_PORT_NUMBER);
        //velodyne_pointcloud::Convert conv;

        if( config_.cut_angle >= 0) //Cut at specific angle feature enabled
        {
            scan_packets.reserve(config_.npackets);
            VelodynePacket tmp_packet;
            while(true)
            {
                while(true)
                {
                    int rc = input_->getPacket(&tmp_packet, config_.time_offset);
                    if (rc == 0) break;       // got a full packet?
                    if (rc < 0) return false; // end of file reached?
                }
                scan_packets.emplace_back(tmp_packet);

                // Extract base rotation of first block in packet
                std::size_t azimuth_data_pos = 100*0+2;
                u_int8_t azimuth = *(static_cast<u_int8_t*>(&tmp_packet.data[azimuth_data_pos]));

                //if first packet in scan, there is no "valid" last_azimuth_
                if (last_azimuth_ == -1)
                {
                    last_azimuth_ = azimuth;
                    continue;
                }
                if((last_azimuth_ < config_.cut_angle && config_.cut_angle <= azimuth)
                        || ( config_.cut_angle <= azimuth && azimuth < last_azimuth_)
                        || (azimuth < last_azimuth_ && last_azimuth_ < config_.cut_angle))
                {
                    last_azimuth_ = azimuth;
                    break; // Cut angle passed, one full revolution collected
                }
                last_azimuth_ = azimuth;
            }
        }
        else // standard behaviour
        {
            // Since the velodyne delivers data at a very high rate, keep
            // reading and publishing scans as fast as possible.
            scan_packets.resize(config_.npackets);
            for (int i = 0; i < config_.npackets; ++i)
            {
                while (true)
                {
                    // keep reading until full packet received
                    int rc = input_->getPacket(&scan_packets[i], config_.time_offset);
                    if (rc == 0) break;       // got a full packet?
                    if (rc < 0) return false; // end of file reached?

                }
            }
        }
        trans_->processScan(scan_packets, cloud);
        return true;
    }


} // namespace velodyne_driver
