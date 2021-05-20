/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data main
*/
#pragma once
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include <string>
#include "velodyne_driver/driver.h"
#include "velodyne_pcl/viewer.h"

class Info
{
public:
    Info() = default;
    Info(const std::string& address,
         const std::string& port,
         const std::string& pcap,
         bool is_saved)
        : address_{address},
          port_{port},
          pcap_{pcap},
          is_saved_{is_saved},
          rate_{0} {}
    ~Info() noexcept {}

    bool select_info(const int& argc, char** argv)
    {
        if (pcl::console::find_argument(argc, argv, "-h") == 1)
        {
            this->printUsage(argv[0]);
            return false;
        }

        if (pcl::console::find_argument(argc, argv, "-save") == 1)
        {
            is_saved_ = true;
            std::cout << pcl::console::find_argument(argc, argv, "-save") << std::endl;
        }

        if(pcl::console::find_argument(argc, argv, "-port") == 1)
            pcl::console::parse_argument(argc, argv, "-port", port_);

        if (pcl::console::find_argument(argc, argv, "-ip") == 1)
            pcl::console::parse_argument(argc, argv, "-ip", address_);
        else if (pcl::console::find_argument(argc, argv, "-pcap") == 1)
        {
            pcl::console::parse_argument(argc, argv, "-pcap", pcap_);
            this->rate_ = 100;
        }

        return true;
    }

    void printUsage(const char* progName)
    {
        std::cout << "\n\nUsage: " << progName <<" [options]\n\n"
                  << "Options:\n"
                  << "You shoud input ip or pcap:\n"
                  << "-------------------------------------------\n"
                  << "-h           this help\n"
                  << "-ip          ip address\n"
                  << "-port        port\n"
                  << "-pcap        pcap filename\n"
                  << "-save        Save to pcd files?(bool)"
                  << "\n\n";
    }

    std::string get_address() const
    {
        return address_;
    }

    std::string get_port() const
    {
        return port_;
    }

    std::string get_pcap() const
    {
        return pcap_;
    }

    bool get_is_saved() const
    {
        return is_saved_;
    }

    int get_rate() const
    {
        return rate_;
    }

    void print_info()
    {
        std::cout << "*****************************" << std::endl;
        std::string result_is_saved =  (is_saved_)? "true" : "false";
        std::cout << "-ipadress : " << address_ << std::endl;
        std::cout << "-port : " << port_ << std::endl;
        std::cout << "-pcap : " << pcap_ << std::endl;
        std::cout << "-saveframes : " << result_is_saved << std::endl;
        std::cout << "*****************************" << std::endl;
    }

private:
    std::string address_;
    std::string port_;
    std::string pcap_;
    bool is_saved_;
    int rate_;
};

