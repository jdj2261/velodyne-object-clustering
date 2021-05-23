/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data main
*/

#include "main.h"

using namespace velodyne_pcl;
using namespace velodyne_driver;

void onInit(std::shared_ptr<VelodyneDriver> dvr,
            const int &rate,
            pointcloud::Ptr &out_cloud)
{
    volatile bool running_;
    while (true)
    {
        pointcloud::Ptr cloud(new pointcloud);
        running_ = dvr->poll(cloud);
        if (!running_)
            std::cout << "fail" << std::endl;
        running_ = false;
        out_cloud = cloud;
        std::this_thread::sleep_for(std::chrono::milliseconds(rate));
    }
}

int main(int argc, char** argv)
{
    std::shared_ptr<Info> info = std::make_shared<Info>("", "2368", "", false);
    if (info->selectInfo(argc, argv) != true)
        return 0;
    info->printInfo();
    std::shared_ptr<VelodyneDriver> dvr =
            std::make_shared<VelodyneDriver>(info->get_address(),
                                             info->get_port(),
                                             info->get_pcap(),
                                             info->get_is_saved());

    std::shared_ptr<Viewer> viewer = std::make_shared<Viewer>(CameraAngle::TopDown, 30);
    visualizer::Ptr pcl_viewer;
    pcl_viewer = viewer->setCameraAngle();

    pointcloud::Ptr cloud(new pointcloud);
    std::thread t1(onInit, dvr, info->get_rate(), std::ref(cloud));

    while (true)
    {
//        auto startTime = std::chrono::high_resolution_clock::now();
        pcl_viewer->addPointCloud<PointXYZI>(cloud, "test");
        pcl_viewer->spinOnce();
//        auto endTime = std::chrono::high_resolution_clock::now();
//        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//        std::cout << "Main " << elapsedTime.count() << " milliseconds" << std::endl;
        pcl_viewer->removeAllPointClouds();
        pcl_viewer->removeAllShapes();
        pcl_viewer->removeText3D();
    }
    return 0;
}
