/** @author Dae Jong Jin
 ** @date   2021. 05. 14
 ** @file   Velodyne 3D LIDAR data input classes
*/

#include <iostream>
#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <cstring>
#include <sys/file.h>
#include <time.h>
#include "velodyne_driver/input.h"

namespace velodyne_driver
{
  VelodynePacket velodynePacket;
  static const size_t packet_size = sizeof(velodynePacket.data);

  ////////////////////////////////////////////////////////////////////////
  // InputSocket class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param port UDP port number
   */
  InputSocket::InputSocket(uint16_t port):
    Input (port)
  {
    sockfd_ = -1;

    // connect to Velodyne UDP port
    std::cout << "Opening UDP socket: port " << port << std::endl;
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_ == -1)
    {
      perror("socket");
      return;
    }

    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(port);          // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

    if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
      {
        perror("bind");                 // TODO: ROS_ERROR errno
        return;
      }

    if (fcntl(sockfd_,F_SETFL, O_NONBLOCK|FASYNC) < 0)
      {
        perror("non-block");
        return;
      }
  }

  /** @brief destructor */
  InputSocket::~InputSocket()
  {
    (void) close(sockfd_);
  }

  /** @brief Get one velodyne packet. */
  int InputSocket::getPacket(VelodynePacket *pkt, const double time_offset)
  {
    time_t time1 = time(nullptr);

    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
    {
      do
      {
        int retval = poll(fds, 1, POLL_TIMEOUT);
        if (retval < 0)             // poll() error?
        {
          if (errno != EINTR)
            std::cout<<"poll() error: %s" <<strerror(errno)<<std::endl;
          return -1;
        }
        if (retval == 0)            // poll() timeout?
        {
          std::cout<<"Velodyne poll() timeout"<<std::endl;
          return -1;
        }
        if ((fds[0].revents & POLLERR)
            || (fds[0].revents & POLLHUP)
            || (fds[0].revents & POLLNVAL)) // device error?
        {
          std::cout<<"poll() reports Velodyne error"<<std::endl;
          return -1;
        }
      } while ((fds[0].revents & POLLIN) == 0);
      // Receive packets that should now be available from the
      // socket using a blocking read.
      ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0],
                                packet_size,  0,
                                (sockaddr*) &sender_address,
                                &sender_address_len);
//      std::cout<<"number of bytes received"<<nbytes<<std::endl;

      if (nbytes < 0)
      {
        if (errno != EWOULDBLOCK)
        {
          perror("recvfail");
          std::cout<<"recvfail"<<std::endl;
          return -1;
        }
      }
      else if ((size_t) nbytes == packet_size)
      {
        break; //done
      }

      std::cout << "incomplete Velodyne packet read: "
                << nbytes << " bytes" << std::endl;
    }

    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred. Add the time offset.
    time_t time2 = time(nullptr);
    pkt->stamp = (time1 + time2) / 2 + time_offset;

    return 0;
  }

  ////////////////////////////////////////////////////////////////////////
  // InputPCAP class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */

  InputPCAP::InputPCAP(uint16_t port,
                       double packet_rate,
                       std::string filename):
    Input(port),
    packet_rate_(packet_rate),
    filename_(filename),
    read_once_(false),
    read_fast_(false),
    repeat_delay(0.0)
  {
    pcap_ = nullptr;
    empty_ = true;

    std::cout << "Opening PCAP file " << filename_.c_str() << std::endl;
    if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == nullptr)
    {
      std::cout << "Error opening Velodyne socket dump file." << std::endl;
      return;
    }

    std::stringstream filter;
    filter << "udp dst port " << port;
    pcap_compile(pcap_, &pcap_packet_filter_,
                 filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
  }
  /** destructor */
  InputPCAP::~InputPCAP(void)
  {
    pcap_close(pcap_);
  }

  /** @brief Get one velodyne packet. */
  int InputPCAP::getPacket(VelodynePacket *pkt, const double time_offset)
  {
    struct pcap_pkthdr *header;
    const u_char *pkt_data;

    while (true)
    {
      int res;
      if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
      {
        // Skip packets not for the correct port and from the
        // selected IP address.
        if (0 == pcap_offline_filter(&pcap_packet_filter_,
                                     header, pkt_data))
          continue;

        memcpy(&pkt->data[0], pkt_data+42, packet_size);
        pkt->stamp = time(nullptr);
        empty_ = false;
        return 0;                   // success
      }

      if (empty_)
      {
        std::cout << "Error "
                  << res
                  << " reading Velodyne packet: "
                  << pcap_geterr(pcap_);
        return -1;
      }
      pcap_close(pcap_);
      pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
      empty_ = true;
    }
  }

} // namespace velodyne_driver
