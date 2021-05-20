#include "velodyne_pcl/viewer.h"

namespace velodyne_pcl
{
    ptr_visualizer Viewer::setCameraAngle()
    {
        ptr_visualizer viewer(new visualizer(viewer_name_));
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
}