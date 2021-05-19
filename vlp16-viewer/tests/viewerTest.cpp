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
    std::shared_ptr<Viewer> viewer = std::make_shared<Viewer>(CameraAngle::TopDown, 30);

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

    if(pcl::console::find_argument(argc, argv, "-port") >= 0)
    {
        pcl::console::parse_argument(argc, argv, "-port", port);
        std::cout << "Port Changed" << std::endl;
    }

    if (pcl::console::find_argument(argc, argv, "-ip") >= 0)
    {
        pcl::console::parse_argument(argc, argv, "-ip", address);

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

    std::shared_ptr<VelodyneDriver> dvr =
            std::make_shared<VelodyneDriver>(address, port, pcap, is_saved);
    ptr_visualizer pcl_viewer;
    pcl_viewer = viewer->setCameraAngle();

    while(!pcl_viewer->wasStopped())
    {
        pointcloud::Ptr cloud(new pointcloud);
//        auto startTime = std::chrono::high_resolution_clock::now();
        dvr->poll(cloud);
        pcl_viewer->addPointCloud<PointXYZI>(cloud, "test");
        pcl_viewer->spinOnce(100);
//        auto endTime = std::chrono::high_resolution_clock::now();
//        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//        std::cout << "Main " << elapsedTime.count() << " milliseconds" << std::endl;
        pcl_viewer->removeAllPointClouds();
        pcl_viewer->removeAllShapes();
        pcl_viewer->removeText3D();
    }

    return 0;
}
