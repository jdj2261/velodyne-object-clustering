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
    using visualizer = pcl::visualization::PCLVisualizer;
    using ptr_visualizer = visualizer::Ptr;

    using PointXYZI = pcl::PointXYZI;
    using pointcloud = pcl::PointCloud<PointXYZI>;

    enum class CameraAngle
    {
        XY,
        TopDown,
        Side,
        FPS
    };

    class Viewer
    {
    public:
        // TODO
        Viewer() = default;
        Viewer(const CameraAngle& setAngle, const int& distance):
            setAngle_(setAngle),
            distance_(distance){}
        ~Viewer() noexcept{}

        ptr_visualizer setCameraAngle();
        void printUsage(const char* progName);

    private:
        CameraAngle setAngle_;
        int distance_;
        const std::string viewer_name_{"VLP-16 Viewer"};
    };
}
