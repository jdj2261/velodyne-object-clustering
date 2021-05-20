/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data main
*/

#include "main.h"

using namespace velodyne_pcl;
using namespace velodyne_driver;

int main(int argc, char** argv)
{
    std::shared_ptr<Info> info = std::make_shared<Info>("", "2368", "", false);
    if (info->select_info(argc, argv) != true)
        return 0;
    info->print_info();
    std::shared_ptr<VelodyneDriver> dvr =
            std::make_shared<VelodyneDriver>(info->get_address(),
                                             info->get_port(),
                                             info->get_pcap(),
                                             info->get_is_saved());

    std::shared_ptr<Viewer> viewer = std::make_shared<Viewer>(CameraAngle::TopDown, 30);
    ptr_visualizer pcl_viewer;
    pcl_viewer = viewer->setCameraAngle();

    while(!pcl_viewer->wasStopped())
    {
        pointcloud::Ptr cloud(new pointcloud);
//        auto startTime = std::chrono::high_resolution_clock::now();
        dvr->poll(cloud);
        pcl_viewer->addPointCloud<PointXYZI>(cloud, "test");
        pcl_viewer->spinOnce(info->get_rate());
//        auto endTime = std::chrono::high_resolution_clock::now();
//        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//        std::cout << "Main " << elapsedTime.count() << " milliseconds" << std::endl;
        pcl_viewer->removeAllPointClouds();
        pcl_viewer->removeAllShapes();
        pcl_viewer->removeText3D();
    }

    return 0;
}
