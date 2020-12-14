/* \author Aaron Brown */
// Velodyne Object Tracking using PCL
// for exploring self-driving car sensors

/******************************************************************************
 *
 * Developer: Daejong Jin (wlseoeo@gmail.com)
 * Date: 12/03/2020
 *
 */
#include "main.hpp"
// Include VelodyneCapture Header
#include "velodyne/velodyneCapture.hpp"
#include "render/render.hpp"
#include "process/processPointClouds.cpp"


// To make 3d Boxing for clustered points
void makeBox(pcl::visualization::PCLVisualizer::Ptr& viewer,  std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pcd_processor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
{
    constexpr float kFilterResolution = 10;
    const Vect3 MinPoint(-10.0f, -20.0f, -1.5f);
    const Vect3 MaxPoint(10.0f, 2.0f, 4.0f);

    //    renderPointCloud(viewer, input_cloud, "test", Color(1,1,1));
    auto filter_cloud = pcd_processor->FilterCloud(input_cloud, kFilterResolution, MinPoint, MaxPoint);

    renderPointCloud(viewer, filter_cloud, "FilteredCloud", Color(1,1,1));

    constexpr int kMaxIterations = 500;
    constexpr float kDistanceThreshold = 0.01;
    auto segment_cloud = pcd_processor->SegmentPlane(filter_cloud, kMaxIterations, kDistanceThreshold);

    // render obstacles point cloud with red
    //    renderPointCloud(viewer, segment_cloud.first, "ObstacleCloud", Color(1, 0, 0));

    constexpr float kClusterTolerance = 1.0;
    constexpr int kMinSize = 5;
    constexpr int kMaxSize = 5000;
    auto cloud_clusters = pcd_processor->Clustering(segment_cloud.first, kClusterTolerance, kMinSize, kMaxSize);

    int cluster_ID = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(0.5, 0, 1)};

    int num_of_colors = colors.size();

    Box host_box = {-1.0, -1.7, -1.5, 1.0, 1.7, 0.2};
    renderBox(viewer, host_box, -1, Color(0.5, 0, 1), 0.8);

    constexpr float kBBoxMinHeight = 0.75;
    constexpr float kBBoxBound = 0.75;

    for(const auto& cluster : cloud_clusters) {
        //        std::cout << "cluster size ";
        //        pcd_processor->numPoints(cluster);

//            renderPointCloud(viewer, cluster, "ObstacleCloud" + std::to_string(cluster_ID), colors[cluster_ID % num_of_colors]);

        pcl::PointXYZI minPoint, maxPoint;
        pcl::getMinMax3D(*cluster, minPoint, maxPoint);

        Box box = pcd_processor->BoundingBox(minPoint, maxPoint);
        pcl::PointXYZ test_point;
        test_point.x = box.x_mid;
        test_point.y = box.y_mid;
        test_point.z = 1.0;
        // Filter out some cluster with little points and shorter in height
        if (cluster->points.size() >= kMinSize )
        {
            if (maxPoint.x - minPoint.x < host_box.x_max- host_box.x_min &&
                    maxPoint.y - minPoint.y < host_box.y_max- host_box.y_min &&
                    maxPoint.z - minPoint.z < host_box.z_max- host_box.z_min)
            {
                continue;
            }
            else
            {
                renderBox(viewer, box, cluster_ID);
                viewer->addText3D(std::to_string((int)abs(test_point.y)), test_point, 1.0, 1.0, 1.0, 1.0, std::to_string(cluster_ID));
            }
        }

        //        cout << cluster_ID << endl;
        cluster_ID++;
    }
    viewer->addText(" Cluster: " + std::to_string(cluster_ID), 5, 5, 20, 1, 1, 1, std::to_string(cluster_ID));

    //render ground plane with green
//    renderPointCloud(viewer, segment_cloud.second, "GroundCloud", Color(0, 1, 0));

    // if (!viewer->updatePointCloud<pcl::PointXYZI>(input_cloud,"raw_cloud"))
    // {
    //     viewer->addPointCloud<pcl::PointXYZI>(input_cloud,"raw_cloud");
    //     //            pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered,"cloud");
    // }
}

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 100;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 25, 25, 0, 100, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(0, 3, 1, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}


int main(int argc, char *argv[])
{
    // Command - Line Argument Parsing if (pcl::console::find_switch(argc, argv, "-help"))
    if (pcl::console::find_switch( argc, argv, "-help" ))
    {
        std::cout << "usage: " << argv[0]
                  << " [-ipaddress <192.168.1.70>]"
                  << " [-port <2368>]"
                  << " [-pcap <*.pcap>]"
                  << " [-saveframes]"
                  << " [-help]"
                  << std::endl;
        return 0;
    }

    std::string ipaddress("192.168.1.70");
    std::string port("2368");
    std::string pcap;
    bool is_saved(false);

    pcl::console::parse_argument(argc, argv, "-ipaddress", ipaddress);
    pcl::console::parse_argument(argc, argv, "-port", port);
    pcl::console::parse_argument(argc, argv, "-pcap", pcap);
    pcl::console::parse_argument(argc, argv, "-saveframes", is_saved);

    std::cout << "-ipadress : " << ipaddress << std::endl;
    std::cout << "-port : " << port << std::endl;
    std::cout << "-pcap : " << pcap << std::endl;
    std::cout << "-saveframes : " << is_saved << std::endl;

    // Open VelodyneCapture that retrieve from Sensor
    const boost::asio::ip::address address = boost::asio::ip::address::from_string(ipaddress);
    const unsigned short ip_port = std::stoi(port);

    std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI = std::make_shared<ProcessPointClouds<pcl::PointXYZI>>();
    //velodyne::VLP16Capture capture( address, ip_port);
    //velodyne::HDL32ECapture capture( address, port );

    // Open VelodyneCapture that retrieve from PCAP
    // velodyne::VLP16Capture capture(pcap);
    velodyne::HDL32ECapture capture( pcap) ;

    if (!capture.isOpen())
    {
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    pcl::visualization::PCLVisualizer::Ptr pcl_viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    CameraAngle setAngle = FPS;
    initCamera(setAngle, pcl_viewer);

    while (!pcl_viewer->wasStopped())
    {
        pcl_viewer->removeAllPointClouds();
        pcl_viewer->removeAllShapes();
        pcl_viewer->removeText3D();

        // Capture One Rotation Data
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        if (lasers.empty())
            continue;
        else
            pointProcessorI->laser2pcd(lasers, cloud);

        std::cout << "cloud size ";
        std::cout << cloud->points.size() << std::endl;
        makeBox(pcl_viewer, pointProcessorI, cloud);


        pcl_viewer->spinOnce();
    }

    cout << "finished.." << endl;
    pcl_viewer->close();

    return 0;
}
