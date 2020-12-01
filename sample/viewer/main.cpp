#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

// Include VelodyneCapture Header
#include "VelodyneCapture.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>


int main( int argc, char* argv[] )
{
    // Open VelodyneCapture that retrieve from Sensor
    const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.21" );
    const unsigned short port = 2368;
    // velodyne::VLP16Capture capture( address, port );
    //velodyne::HDL32ECapture capture( address, port );

    // Open VelodyneCapture that retrieve from PCAP
    const std::string filename = "test.pcap";
    velodyne::VLP16Capture capture( filename );
    //velodyne::HDL32ECapture capture( filename );
    bool close_window = false;

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    // Create Viewer
//    cv::viz::Viz3d viewer( "Velodyne" );

    boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    pcl_viewer->setPosition(0,0);
//    pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0); // Setting background to a dark grey
//    pcl_viewer->setShowFPS(false);
//    pcl_viewer->addCoordinateSystem();
//    pcl_viewer->setPosition(0,0);
    pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0); // black background
    pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud"); // size of point clouds
    pcl_viewer->setRepresentationToSurfaceForAllActors();
    pcl_viewer->addCoordinateSystem ();
//    pcl_viewer->initCameraParameters ();

    // Register Keyboard Callback
//    viewer.registerKeyboardCallback(
//        []( const cv::viz::KeyboardEvent& event, void* cookie ){
//            // Close Viewer
//            if( event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
//                static_cast<cv::viz::Viz3d*>( cookie )->close();
//            }
//        }
//        , &viewer
//    );

    while(!pcl_viewer->wasStopped() && !close_window ){

        pcl_viewer->removeAllPointClouds();
        pcl_viewer->removeAllShapes();
        // Capture One Rotation Data
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;
        if( lasers.empty() ){
            break;
        }

        // Convert to 3-dimention Coordinates
        std::vector<cv::Vec3f> buffer( lasers.size() );
//        pcl::PointCloud<pcl::PointXYZRGB> pcl_buffer;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_buffer(new pcl::PointCloud<pcl::PointXYZI>);
//        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointXYZI p;

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

            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = i;

            buffer.push_back( cv::Vec3f( x, y, z ) );
            pcl_buffer->push_back(p);
        }

        pcl_buffer->width = pcl_buffer->points.size();
        cout << "Number of lasers:" << lasers.size() << endl;
        pcl_buffer->height = 1;
        pcl_buffer->points.resize (pcl_buffer->width * pcl_buffer->height);
        pcl_buffer->is_dense = true;
        if (!pcl_viewer->updatePointCloud<pcl::PointXYZI>(pcl_buffer,"cloud"))
        {
//            pcl_viewer->removePointCloud("POINTCLOUD");
            pcl_viewer->addPointCloud<pcl::PointXYZI>(pcl_buffer,"cloud");
        }
        // Create Widget
//        cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_32FC3, &buffer[0] );
//        cv::viz::WCloud cloud( cloudMat, cv::viz::Color::white()  );

//        // Show Point Cloud
//        viewer.showWidget( "Cloud", cloud );
//        viewer.spinOnce(20);
        pcl_viewer->spinOnce(1);
    }

    close_window = false;
    cout << "finished.." << endl;
    pcl_viewer->close();


    // Close All Viewers
//    cv::viz::unregisterAllWindows();

    return 0;
}
