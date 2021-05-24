#pragma once
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "velodyne_cluster/vector.h"
#include "velodyne_pcl/viewer.h"

using namespace velodyne_pcl;

namespace velodyne_cluster
{
    class Processor
    {
    public:
        void rotate(const pointcloud::Ptr &input_cloud,
                      const float &theta,
                      pointcloud::Ptr &output_cloud);

        void makeBox();

        pointcloud::Ptr filterCloud(const pointcloud::Ptr &input_cloud,
                                    const Box &host_box, const float &filterRes,
                                    const Vect3D &minPoint, const Vect3D &maxPoint);

        std::pair<pointcloud::Ptr, pointcloud::Ptr> segmentPlane(const pointcloud::Ptr &input_cloud,
                                                                 const int &maxIterations,
                                                                 const double &distanceThreshold);
        std::pair<pointcloud::Ptr, pointcloud::Ptr> seperateCloud(const pcl::PointIndices::Ptr &inliers,
                                                                  const pointcloud::Ptr &input_cloud);

        std::vector<pointcloud::Ptr> clusterCloud(const pointcloud::Ptr &cloud,
                                                  const double &clusterTolerance,
                                                  const int &minSize,
                                                  const int &maxSize);
        Box boundBox(const PointXYZI &minPoint, const PointXYZI &maxPoint);
    };

}
