/** @author Dae Jong Jin
 ** @date   2021. 05. 15
 ** @file   Velodyne 3D LIDAR data main
*/

#include "main.h"
#include "velodyne_cluster/vector.h"
#include "velodyne_cluster/process.h"

using namespace velodyne_pcl;
using namespace velodyne_driver;
using namespace velodyne_cluster;

constexpr Box host_box {-1.0f, -1.7f, -1.5f, 1.0f, 1.7f, -0.5f};
const Vect3D MinPoint(-20, -20, -5.0);
const Vect3D MaxPoint(20, 20, 5);

constexpr float kFilterResolution = 0.1f;
constexpr int kMaxIterations = 1000;
constexpr double kDistanceThreshold = 0.1;

constexpr double kClusterTolerance = 0.9;
constexpr int kMinSize = 3;
constexpr int kMaxSize = 1000;

void do_filter(std::shared_ptr<Processor> processor,
               const pointcloud::Ptr &input_cloud,
               pointcloud::Ptr &output_cloud)
{
    if (input_cloud->empty())
        return;
    std::unique_ptr<Timer> timer = std::make_unique<Timer>("filter_cloud");
    auto filter_cloud = processor->filterCloud(input_cloud,
                                               host_box,
                                               kFilterResolution,
                                               MinPoint, MaxPoint);
    output_cloud = filter_cloud;
}

void do_segmentation(std::shared_ptr<Processor> processor,
                     const pointcloud::Ptr &input_cloud,
                     std::pair<pointcloud::Ptr, pointcloud::Ptr> &output_cloud)
{
    if (input_cloud->empty())
        return;
    auto segment_cloud = processor->segmentPlane(input_cloud,
                                                 kMaxIterations,
                                                 kDistanceThreshold);
    output_cloud = segment_cloud;
}

void makeBox(std::shared_ptr<Viewer> viewer,
             visualizer::Ptr pcl_viewer,
             std::shared_ptr<Processor> processor,
             const pointcloud::Ptr &input_cloud)
{
    int cluster_ID = 0;
    Color color(0.5, 0, 1);
    double opacity(0.8);

    if (input_cloud->empty())
        return;

    auto clustered_cloud = processor->clusterCloud(
                input_cloud,
                kClusterTolerance,
                kMinSize, kMaxSize);
    for(const auto &cluster : clustered_cloud)
    {
        PointXYZI minPoint, maxPoint;
        pcl::getMinMax3D(*cluster, minPoint, maxPoint);
        Box box = processor->boundBox(minPoint, maxPoint);
        if (cluster->points.size() >= kMinSize && cluster->points.size() <= kMaxSize )
        {
            viewer->viewBox(pcl_viewer, box, cluster_ID, color, opacity);
        }
//        std::cout << cluster_ID << std::endl;
        cluster_ID++;
    }
}

int main(int argc, char** argv)
{
    std::shared_ptr<Info> info = std::make_shared<Info>("", "", "", false);
    if (info->selectInfo(argc, argv) != true)
        return 0;
    info->printInfo();
    std::shared_ptr<VelodyneDriver> dvr =
            std::make_shared<VelodyneDriver>(info->get_address(),
                                             info->get_port(),
                                             info->get_pcap(),
                                             info->get_is_saved());

    std::shared_ptr<Processor> processor = std::make_shared<Processor>();

    std::shared_ptr<Viewer> viewer = std::make_shared<Viewer>(CameraAngle::TopDown, 30);
    visualizer::Ptr pcl_viewer;
    pcl_viewer = viewer->setCameraAngle();

    pointcloud::Ptr filter_cloud(new pointcloud);
    std::pair<pointcloud::Ptr, pointcloud::Ptr> segment_cloud(new pointcloud, new pointcloud);

    while(!pcl_viewer->wasStopped())
    {
        pointcloud::Ptr cloud(new pointcloud);
//        std::unique_ptr<Timer> timer = std::make_unique<Timer>("filter_cloud");
        bool running_ = dvr->poll(cloud);
        do_filter(processor, cloud, filter_cloud);
        do_segmentation(processor, filter_cloud, segment_cloud);

//        viewer->viewPointCloud(pcl_viewer, cloud, "cloud", Color(1.0, 1.0, 1.0));
        viewer->viewPointCloud(pcl_viewer, segment_cloud.first, "object", Color(1.0, 0.0, 0.0));
        viewer->viewPointCloud(pcl_viewer, segment_cloud.second, "plane", Color(0.0, 1.0, 0.0));

        // TODO -> Thread do_filter
        makeBox(viewer, pcl_viewer, processor, segment_cloud.first);
        if (!running_)
            std::cout << "failed" << std::endl;

//        timer->elapsed();

        pcl_viewer->spinOnce(info->get_rate());
        pcl_viewer->removeAllPointClouds();
        pcl_viewer->removeAllShapes();
        pcl_viewer->removeText3D();
    }    
    return 0;
}
