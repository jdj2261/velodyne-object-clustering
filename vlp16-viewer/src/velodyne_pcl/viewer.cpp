#include "velodyne_pcl/viewer.h"

namespace velodyne_pcl
{
    bool cmpf(double A, double B, double epsilon = 0.005)
    {
        return (fabs(A - B) < epsilon);
    }

    visualizer::Ptr Viewer::setCameraAngle()
    {
        visualizer::Ptr viewer(new visualizer(viewer_name_));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->initCameraParameters ();
        switch(setAngle_)
        {
            case CameraAngle::XY : viewer->setCameraPosition(-distance_, -distance_, distance_, 1, 1, 0); break;
            case CameraAngle::TopDown : viewer->setCameraPosition(0, -distance_, distance_, 0, 2, 1); break;
            case CameraAngle::Side : viewer->setCameraPosition(0, -distance_, 0, 0, 0, 1); break;
            case CameraAngle::FPS: viewer->setCameraPosition(-1, 1, 0, 0, 0, 10);
        }

        if(setAngle_!=CameraAngle::FPS)  viewer->addCoordinateSystem (1.0);
        return viewer;
    }

    void Viewer::viewPointCloud(visualizer::Ptr &viewer,
                        const pointcloud::Ptr &input_cloud,
                        const std::string &cloud_name,
                        Color color)
    {

        if(cmpf(color.getR(), -1.0))
            visualizer_with_intensity intensity_distribution(input_cloud, "intensity");
        else
        {
            viewer->addPointCloud<PointXYZI>(input_cloud, cloud_name);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                     color.getR(), color.getG(), color.getB(),
                                                     cloud_name);
        }
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  2,
                                                  cloud_name);
    }

    void Viewer::viewBox(visualizer::Ptr &viewer, const Box &box,
                         const int &id,
                         const Color &color, double &opacity)
    {
        if(opacity > 1.0)
            opacity = 1.0;
        if(opacity < 0.0)
            opacity = 0.0;


        std::string cube = "box"+std::to_string(id);

        //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
        viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.getR(), color.getG(), color.getB(), cube);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.getR(), color.getG(), color.getB(), cube);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

        std::string cubeFill = "boxFill"+std::to_string(id);
        //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
        viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.getR(), color.getG(), color.getB(), cubeFill);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.getR(), color.getG(), color.getB(), cubeFill);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
    }
}
