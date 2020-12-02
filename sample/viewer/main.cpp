#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
//#include <opencv2/viz.hpp>

// Include VelodyneCapture Header
#include "VelodyneCapture.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>


struct Color
{
    float r, g, b;

    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB)
    {}
};

struct Box
{
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
};

enum CameraAngle
{
    XY, TopDown, Side, FPS
};

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 1000;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (100.0);
}

int main( int argc, char* argv[] )
{
    // Open VelodyneCapture that retrieve from Sensor
    const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.70" );
    const unsigned short port = 2368;
//    velodyne::VLP16Capture capture( address, port );
    //velodyne::HDL32ECapture capture( address, port );

    // Open VelodyneCapture that retrieve from PCAP
    const std::string filename = "test.pcap";
    velodyne::VLP16Capture capture( filename );
//    velodyne::HDL32ECapture capture( filename );
    bool close_window = false;

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

//    pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0); // black background
//    pcl_viewer->setRepresentationToSurfaceForAllActors();
//    pcl_viewer->addCoordinateSystem ();

    CameraAngle setAngle = XY;
    initCamera(setAngle, pcl_viewer);

    while(!pcl_viewer->wasStopped() && !close_window ){

        pcl_viewer->removeAllPointClouds();
        pcl_viewer->removeAllShapes();
        // Capture One Rotation Data
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI point;

        if( lasers.empty() ){
            continue;
        }

        for( const velodyne::Laser& laser : lasers ){
            const double distance = static_cast<double>( laser.distance );
            const double azimuth  = laser.azimuth  * CV_PI / 180.0;
            const double vertical = laser.vertical * CV_PI / 180.0;

            float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
            float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
            float z = static_cast<float>( ( distance * std::sin( vertical ) ) );
            float i = static_cast<unsigned int>(laser.intensity);

            if( x == 0.0f && y == 0.0f && z == 0.0f ){
                x = std::numeric_limits<float>::quiet_NaN();
                y = std::numeric_limits<float>::quiet_NaN();
                z = std::numeric_limits<float>::quiet_NaN();
            }

            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = i;

            cloud->push_back(point);
        }

        cloud->width = cloud->points.size();
        //        cout << "Number of lasers:" << lasers.size() << endl;
        cloud->height = 1;
        cloud->points.resize (cloud->width * cloud->height);
        cloud->is_dense = true;

        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud (cloud);
        vg.setLeafSize (10, 10, 10);
        vg.setDownsampleAllData (true);
        vg.filter (*cloud_filtered);

        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-2000, 2000);   // -2m ~ 2m
        pass.setFilterLimitsNegative (false);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-5000, 5000);  // 0 ~ 10m
        pass.setFilterLimitsNegative (false);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-150, 100);  // 0 ~ 1m
        pass.setFilterLimitsNegative (false);
        pass.filter (*cloud_filtered);

        auto startTime = std::chrono::steady_clock::now();
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance (25); // default : 10
        ec.setMinClusterSize (50); // default : 1000
        ec.setMaxClusterSize (1000); // default : 1500
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract(cluster_indices);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);

        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size ()<< " clusters" << std::endl;

        int clusterId = 0;

        if (!pcl_viewer->updatePointCloud<pcl::PointXYZI>(cloud,"raw_cloud"))
        {
            pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud,"raw_cloud");
//            pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered,"cloud");
        }
        for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
        {
            std::cout << "cluster size ";
            std::cout << cluster->points.size() << std::endl;
//            pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered,"raw_cloud"+std::to_string(clusterId));
            pcl_viewer->addPointCloud<pcl::PointXYZI>(cluster,"cloud"+std::to_string(clusterId));
            pcl_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud"+std::to_string(clusterId));

            pcl::PointXYZI minPoint, maxPoint;
            pcl::getMinMax3D(*cluster, minPoint, maxPoint);

            Box box;
            box.x_min = minPoint.x;
            box.y_min = minPoint.y;
            box.z_min = minPoint.z;
            box.x_max = maxPoint.x;
            box.y_max = maxPoint.y;
            box.z_max = maxPoint.z;

            std::string cube = "box"+std::to_string(clusterId);
            pcl_viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, 0, 1, 0, cube);
            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
//            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, cube);
            pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);

            ++clusterId;
        }


        pcl_viewer->spinOnce();
    }

    close_window = false;
    cout << "finished.." << endl;
    pcl_viewer->close();

    return 0;
}
