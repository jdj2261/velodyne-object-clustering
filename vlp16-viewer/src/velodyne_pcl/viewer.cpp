#include "velodyne_pcl/viewer.h"

namespace velodyne_pcl
{
    void Viewer::printUsage(const char* progName)
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
}
