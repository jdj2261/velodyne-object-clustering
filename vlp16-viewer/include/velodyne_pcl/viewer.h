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
    using PointXYZI = pcl::PointXYZI;
    using pointcloud = pcl::PointCloud<PointXYZI>;
    using visualizer_with_intensity = pcl::visualization::PointCloudColorHandlerGenericField<PointXYZI>;


    struct BoxQ
    {
        Eigen::Vector3f bboxTransform;
        Eigen::Quaternionf bboxQuaternion;
        float cube_length;
        float cube_width;
        float cube_height;
    };

    struct Box
    {
        float x_min;
        float y_min;
        float z_min;
        float x_max;
        float y_max;
        float z_max;
        float x_mid = x_min + (x_max - x_min)/2 ;
        float y_mid = y_min + (y_max - y_min)/2 ;
    };

    enum class CameraAngle
    {
        XY,
        TopDown,
        Side,
        FPS
    };

    class Color
    {
        double r_, g_, b_;

    public:
        Color() = default;
        Color(const double &r, const double &g, const double &b)
            : r_(r), g_(g), b_(b) {}
        ~Color() noexcept {}

        double getR() const { return r_; }
        double getG() const { return g_; }
        double getB() const { return b_; }

        void setR(const double &r) { r_ = r; }
        void setG(const double &g) { g_ = g; }
        void setB(const double &b) { b_ = b; }
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

        visualizer::Ptr setCameraAngle();
        void printUsage(const char* progName);

        void viewPointCloud(visualizer::Ptr &viewer,
                            const pointcloud::Ptr &input_cloud,
                            const std::string &cloud_name,
                            Color color);
        void viewBox(visualizer::Ptr &viewer, const Box &box,
                     const int &id, const Color &color,
                     double &opacity);
    private:
        CameraAngle setAngle_;
        int distance_;
        const std::string viewer_name_{"VLP-16 Viewer"};
    };

}
