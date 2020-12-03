/* \author Aaron Brown */
// Velodyne Object Tracking using PCL
// for exploring self-driving car sensors

/**
 * Developer: Daejong Jin
 * Date: 12/03/2020
 */

#include <main.hpp>
// #include <process/processPointClouds.hpp>
#include <process/processPointClouds.cpp>


void makeBox(pcl::visualization::PCLVisualizer::Ptr& viewer,  std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pcd_processor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud) 
{
    constexpr float kFilterResolution = 10;
    const Eigen::Vector4f kMinPoint(-10000, -10000, -10000, 1);
    const Eigen::Vector4f kMaxPoint(60000, 65000, 40000, 1);
    
    // renderPointCloud(viewer, input_cloud, "test", Color(1,1,1));
    auto filter_cloud = pcd_processor->FilterCloud(input_cloud, kFilterResolution, kMinPoint, kMaxPoint);
    
    renderPointCloud(viewer, filter_cloud, "FilteredCloud", Color(1,1,1));

    constexpr int kMaxIterations = 100;
    constexpr float kDistanceThreshold = 0.2;
    auto segment_cloud = pcd_processor->SegmentPlane(filter_cloud, kMaxIterations, kDistanceThreshold);

        // render obstacles point cloud with red
    // renderPointCloud(viewer, segment_cloud.first, "ObstacleCloud", Color(1, 0, 0));

    constexpr float kClusterTolerance = 50;
    constexpr int kMinSize = 25;
    constexpr int kMaxSize = 5000;
    auto cloud_clusters = pcd_processor->Clustering(segment_cloud.first, kClusterTolerance, kMinSize, kMaxSize);

    int cluster_ID = 1;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(0.5, 0, 1)};
    int num_of_colors = colors.size();

    Box host_box = {-1.5, -1.7, -1, 2.6, 1.7, -0.4};
    renderBox(viewer, host_box, 0, Color(0.5, 0, 1), 0.8);

    constexpr float kBBoxMinHeight = 0.75;
    for(const auto& cluster : cloud_clusters) {
        std::cout << "cluster size ";
        pcd_processor->numPoints(cluster);

    //    renderPointCloud(viewer, cluster, "ObstacleCloud" + std::to_string(cluster_ID), colors[cluster_ID % num_of_colors]);

        Box box = pcd_processor->BoundingBox(cluster);
        // Filter out some cluster with little points and shorter in height
        if (box.z_max - box.z_min >= kBBoxMinHeight || cluster->points.size() >= kMinSize * 2) {
            renderBox(viewer, box, cluster_ID);
        }

        cluster_ID++;
    }

        //render ground plane with green
    // renderPointCloud(viewer, segment_cloud.second, "GroundCloud", Color(0, 1, 0));
    
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
    int distance = 1000;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-1, 1, 0, 0, 0, 10);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(100.0);
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

    CameraAngle setAngle = XY;
    initCamera(setAngle, pcl_viewer);

    while (!pcl_viewer->wasStopped())
    {
        pcl_viewer->removeAllPointClouds();
        pcl_viewer->removeAllShapes();

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

        //        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        //        pcl::PointXYZI point;

        //        if( lasers.empty() ){
        //            continue;
        //        }

        //        for( const velodyne::Laser& laser : lasers ){
        //            const double distance = static_cast<double>( laser.distance );
        //            const double azimuth  = laser.azimuth  * CV_PI / 180.0;
        //            const double vertical = laser.vertical * CV_PI / 180.0;

        //            float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
        //            float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
        //            float z = static_cast<float>( ( distance * std::sin( vertical ) ) );
        //            float i = static_cast<unsigned int>(laser.intensity);

        //            if( x == 0.0f && y == 0.0f && z == 0.0f ){
        //                x = std::numeric_limits<float>::quiet_NaN();
        //                y = std::numeric_limits<float>::quiet_NaN();
        //                z = std::numeric_limits<float>::quiet_NaN();
        //            }

        //            point.x = x;
        //            point.y = y;
        //            point.z = z;
        //            point.intensity = i;

        //            cloud->push_back(point);
        //        }

        //        cloud->width = cloud->points.size();
        //        //        cout << "Number of lasers:" << lasers.size() << endl;
        //        cloud->height = 1;
        //        cloud->points.resize (cloud->width * cloud->height);
        //        cloud->is_dense = true;

        //        pcl::VoxelGrid<pcl::PointXYZI> vg;
        //        vg.setInputCloud (cloud);
        //        vg.setLeafSize (10, 10, 10);
        //        vg.setDownsampleAllData (true);
        //        vg.filter (*cloud_filtered);

        //        pcl::PassThrough<pcl::PointXYZI> pass;
        //        pass.setInputCloud (cloud);
        //        pass.setFilterFieldName ("x");
        //        pass.setFilterLimits (-1000, 1000);   // -2m ~ 2m
        //        pass.setFilterLimitsNegative (false);
        //        pass.filter (*cloud_filtered);

        //        pass.setInputCloud (cloud_filtered);
        //        pass.setFilterFieldName ("y");
        //        pass.setFilterLimits (-5000, 5000);  // 0 ~ 10m
        //        pass.setFilterLimitsNegative (false);
        //        pass.filter (*cloud_filtered);

        //        pass.setInputCloud (cloud_filtered);
        //        pass.setFilterFieldName ("z");
        //        pass.setFilterLimits (-120, 100);  // 0 ~ 1m
        //        pass.setFilterLimitsNegative (false);
        //        pass.filter (*cloud_filtered);

        //        auto startTime = std::chrono::steady_clock::now();
        //        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        //        tree->setInputCloud(cloud_filtered);

        //        std::vector<pcl::PointIndices> cluster_indices;
        //        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        //        ec.setClusterTolerance (50); // default : 10 25
        //        ec.setMinClusterSize (25); // default : 1000
        //        ec.setMaxClusterSize (5000); // default : 1500
        //        ec.setSearchMethod (tree);
        //        ec.setInputCloud (cloud_filtered);
        //        ec.extract(cluster_indices);

        //        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

        //        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        //        {
        //            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

        //            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        //                cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
        //            cloud_cluster->width = cloud_cluster->points.size ();
        //            cloud_cluster->height = 1;
        //            cloud_cluster->is_dense = true;

        //            clusters.push_back(cloud_cluster);

        //        }

        //        auto endTime = std::chrono::steady_clock::now();
        //        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        //        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size ()<< " clusters" << std::endl;

        //        int clusterId = 0;

        // if (!pcl_viewer->updatePointCloud<pcl::PointXYZI>(cloud,"raw_cloud"))
        // {
        //     pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud,"raw_cloud");
        //     //            pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered,"cloud");
        // }
        //        for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
        //        {
        //            std::cout << "cluster size ";
        //            std::cout << cluster->points.size() << std::endl;
        //            //            pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered,"raw_cloud"+std::to_string(clusterId));
        //            pcl_viewer->addPointCloud<pcl::PointXYZI>(cluster,"cloud"+std::to_string(clusterId));
        //            pcl_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud"+std::to_string(clusterId));

        //            pcl::PointXYZI minPoint, maxPoint;
        //            pcl::getMinMax3D(*cluster, minPoint, maxPoint);

        //            Box box;
        //            box.x_min = minPoint.x;
        //            box.y_min = minPoint.y;
        //            box.z_min = minPoint.z;
        //            box.x_max = maxPoint.x;
        //            box.y_max = maxPoint.y;
        //            box.z_max = maxPoint.z;

        //            std::string cube = "box"+std::to_string(clusterId);
        //            pcl_viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, 0, 1, 0, cube);
        //            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
        //            //            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, cube);
        //            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);

        //            ++clusterId;
        //        }

        pcl_viewer->spinOnce();
    }

    cout << "finished.." << endl;
    pcl_viewer->close();

    return 0;
}
