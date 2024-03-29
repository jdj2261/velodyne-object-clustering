// PCL lib Functions for processing point clouds
#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>

#include "velodyne/velodyneCapture.hpp"
#include "render/box.hpp"
#include "render/render.hpp"
#include <opencv2/opencv.hpp>


template <typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    virtual ~ProcessPointClouds(){}

    void laser2pcd (std::vector<velodyne::Laser> lasers, typename pcl::PointCloud<PointT>::Ptr& cloud);
    typename pcl::PointCloud<PointT>::Ptr FilterCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, Box host_box, float filterRes, const Vect3 minPoint, const Vect3 maxPoint);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(const pcl::PointIndices::Ptr& inliers, const typename pcl::PointCloud<PointT>::Ptr& cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, const int maxIterations, const float distanceThreshold);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(const typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize);
    void numPoints(const typename pcl::PointCloud<PointT>::Ptr& cloud);
    Box BoundingBox(const PointT& minPoint, const PointT& maxPoint);

};

