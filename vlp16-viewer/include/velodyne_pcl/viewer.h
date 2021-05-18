#pragma once

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


namespace velodyne_pcl
{
    using ptr_visualizer = pcl::visualization::PCLVisualizer::Ptr;
    using visualizer = pcl::visualization::PCLVisualizer;
    class Viewer
    {
    public:
        // TODO
        /*
        ptr_visualizer simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
        {
          // --------------------------------------------
          // -----Open 3D viewer and add point cloud-----
          // --------------------------------------------
          ptr_visualizer viewer (new visualizer("3D Viewer"));
          viewer->setBackgroundColor (0, 0, 0);
          viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
          viewer->addCoordinateSystem (1.0);
          viewer->initCameraParameters ();
          return viewer;
        }
        */
        void printUsage(const char* progName);
        void set_param(int argc, char **argv,
                          std::string& ip,
                          std::string& port,
                          std::string& pcap,
                          bool& is_saved);
        const std::string viewer_name = "VLP-16 Viewer";
    };


}
