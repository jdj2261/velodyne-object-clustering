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

    static uint16_t DATA_PORT_NUMBER = 2368;  // default data port

    /** @brief Velodyne input base class */
    class Input
    {
    public:
        Input(uint16_t port):port_(port) {}
        virtual ~Input() {}

        virtual int getPacket(VelodynePacket *pkt, const double time_offset) = 0;

    protected:
        uint16_t port_;
    };

    class InputSocket: public Input
    {
    public:
        InputSocket(uint16_t port);
        virtual ~InputSocket();

        virtual int getPacket(VelodynePacket *pkt,
                              const double time_offset);
        void setDeviceIP(const std::string& ip);

    private:
        int sockfd_;
        in_addr devip_;
    };

    class InputPCAP: public Input
    {
    public:
        InputPCAP(uint16_t port = DATA_PORT_NUMBER,
                  double packet_rate = 0.0,
                  std::string filename = "");
        virtual ~InputPCAP();
        virtual int getPacket(VelodynePacket *pkt,
                              const double time_offset);
        void setDeviceIP(const std::string& ip);

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
