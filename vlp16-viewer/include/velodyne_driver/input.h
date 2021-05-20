/** @author Dae Jong Jin
 ** @date   2021. 05. 14
 ** @file   Velodyne 3D LIDAR data input classes
*/
#pragma once

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <string>

namespace velodyne_driver
{
    struct VelodynePacket
    {
        uint8_t data[1206];
        time_t stamp;
    };

    /** @brief Velodyne input base class */
    class Input
    {
    public:
        explicit Input() = default;
        explicit Input(const uint16_t &port):port_(port) {}
        virtual ~Input() noexcept {}
        virtual int getPacket(VelodynePacket *pkt,
                              const double &time_offset) = 0;

    protected:
        uint16_t port_;
    };

    class InputSocket: public Input
    {
    public:
        explicit InputSocket() = default;
        explicit InputSocket(const uint16_t &port);
        virtual ~InputSocket() noexcept override;
        virtual int getPacket(VelodynePacket *pkt,
                              const double &time_offset) override;

    private:
        int sockfd_;
        in_addr devip_;
    };

    class InputPCAP: public Input
    {
    public:
        InputPCAP() = default;
        InputPCAP(const uint16_t &port,
                  const double &packet_rate,
                  const std::string &filename);
        virtual ~InputPCAP() noexcept override;
        virtual int getPacket(VelodynePacket *pkt,
                              const double &time_offset) override;

    private:
        double packet_rate_;
        std::string filename_;
        pcap_t *pcap_;
        bpf_program pcap_packet_filter_;
        char errbuf_[PCAP_ERRBUF_SIZE];
        bool empty_;
        bool pcap_time_;
        bool read_once_;
        bool read_fast_;
        double repeat_delay;
    };

} // namespace velodyne_driver
