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


int main( int argc, char* argv[] )
{
    // Open VelodyneCapture that retrieve from Sensor
    const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.70" );
    const unsigned short port = 2368;
    velodyne::VLP16Capture capture( address, port );
    //velodyne::HDL32ECapture capture( address, port );

    // Open VelodyneCapture that retrieve from PCAP
    const std::string filename = "test.pcap";
//    velodyne::VLP16Capture capture( filename );
//    velodyne::HDL32ECapture capture( filename );
    bool close_window = false;

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0); // black background
    pcl_viewer->setRepresentationToSurfaceForAllActors();
    pcl_viewer->addCoordinateSystem ();

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

        //        pcl::VoxelGrid<pcl::PointXYZI> vg;
        //        vg.setInputCloud (cloud);
        //        vg.setLeafSize (0.01, 0.01, 0.01);
        //        vg.setDownsampleAllData (true);
        //        vg.filter (*cloud_filtered);

        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-200, 200);   // -2m ~ 2m
        pass.setFilterLimitsNegative (false);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (0, 200);  // 0 ~ 10m
        pass.setFilterLimitsNegative (false);
        pass.filter (*cloud_filtered);

        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-10, 200);  // 0 ~ 1m
        pass.setFilterLimitsNegative (false);
        pass.filter (*cloud_filtered);

//        cout << "Number of filter:" << cloud_filtered->size() << endl;

//        pcl::VoxelGrid<pcl::PointXYZI> vg;
//        vg.setInputCloud (cloud_filtered);
//        vg.setLeafSize (10, 10, 10);
//        vg.setDownsampleAllData (true);
//        vg.filter (*cloud_filtered);

        auto startTime = std::chrono::steady_clock::now();
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance (10); // 2cm
        ec.setMinClusterSize (50);
        ec.setMaxClusterSize (1000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract(cluster_indices);

        int j = 0;

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {

            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);

            j++;
        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size ()<< " clusters" << std::endl;

//        for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
//        {
//           std::cout << "cluster size ";
//           std::cout << cluster->points.size() << std::endl;
//        }
        cout << "Number of lasers:" << cluster_indices.size() << endl;
        if (!pcl_viewer->updatePointCloud<pcl::PointXYZI>(cloud_cluster,"cloud"))
        {
            pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud_cluster,"cloud");
//            pcl_viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered,"cloud");
        }

        pcl_viewer->spinOnce();
    }

    close_window = false;
    cout << "finished.." << endl;
    pcl_viewer->close();

    return 0;
}
