/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data main
*/

#include "main.h"
#include "velodyne_pcl/viewer.h"

using namespace velodyne_pcl;
using namespace velodyne_driver;

int main(int argc, char** argv)
{
    std::shared_ptr<Viewer> viewer = std::make_shared<Viewer>();
    std::shared_ptr<VelodyneDriver> dvr = std::make_shared<VelodyneDriver>();

    if (pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        viewer->printUsage(argv[0]);
        return 0;
    }

    std::string address;
    std::string port("2368");
    std::string pcap;
    bool is_saved(false);

    if (pcl::console::find_argument(argc, argv, "-save") >= 0)
    {
        is_saved = true;
        std::cout << "Save to PCD files" << std::endl;
    }

    if (pcl::console::find_argument(argc, argv, "-ip") >= 0 ||
        pcl::console::find_argument(argc, argv, "-port") >= 0)
    {
        pcl::console::parse_argument(argc, argv, "-ip", address);
        pcl::console::parse_argument(argc, argv, "-pcap", port);
        std::cout << "VLP-16 Socket" << std::endl;
    }
    else if (pcl::console::find_argument(argc, argv, "-pcap") >= 0)
    {
        pcl::console::parse_argument(argc, argv, "-pcap", pcap);
        std::cout << "VLP-16 PCAP" << std::endl;
    }
    else
    {
        viewer->printUsage(argv[0]);
        return 0;
    }

    std::string result_is_saved =  (is_saved)? "true" : "false";
    std::cout << "-ipadress : " << address << std::endl;
    std::cout << "-port : " << port << std::endl;
    std::cout << "-pcap : " << pcap << std::endl;
    std::cout << "-saveframes : " << result_is_saved << std::endl;

    ptr_visualizer pcl_viewer(new visualizer(viewer->viewer_name));
//    ptr_visualizer pcl_viewer;

    if (!pcap.empty())
    {
        std::cout << "Capture from PCAP..." << std::endl;
        // TODO
        // grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( pcap ) );
    }
    else if (!address.empty())
    {
        std::cout << "Capture from Sensor..." << std::endl;
        // TODO
        // grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );
    }


    while(!pcl_viewer->wasStopped())
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl_viewer->removeAllPointClouds();
        pcl_viewer->removeAllShapes();
        pcl_viewer->removeText3D();
        auto startTime = std::chrono::high_resolution_clock::now();
        dvr->poll(cloud);
        pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud, "test");
        pcl_viewer->spinOnce(100);
        auto endTime = std::chrono::high_resolution_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "Main " << elapsedTime.count() << " milliseconds" << std::endl;
    }

    return 0;
}
