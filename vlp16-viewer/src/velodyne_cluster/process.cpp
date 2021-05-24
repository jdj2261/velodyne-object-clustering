#include "velodyne_cluster/process.h"

namespace velodyne_cluster
{
    void Processor::rotate(const pointcloud::Ptr &input_cloud,
                           const float &theta,
                           pointcloud::Ptr &output_cloud)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform (0,0) = std::cos (theta);
        transform (0,1) = -sin(theta);
        transform (1,0) = sin (theta);
        transform (1,1) = std::cos (theta);

        pcl::transformPointCloud (*input_cloud, *output_cloud, transform);
    }

    void Processor::makeBox() {}

    pointcloud::Ptr Processor::filterCloud(const pointcloud::Ptr &input_cloud,
                                           const Box &host_box, const float &filterRes,
                                           const Vect3D &minPoint, const Vect3D &maxPoint)
    {
        pointcloud::Ptr filtered_cloud(new pointcloud);
        pcl::PassThrough<PointXYZI> pass;
        pass.setInputCloud(input_cloud);

        pass.setFilterFieldName ("x");
        pass.setFilterLimits (minPoint.getX(), maxPoint.getX());   // -2m ~ 2m
        pass.setFilterLimitsNegative (false);
        pass.filter (*filtered_cloud);

        pass.setInputCloud (filtered_cloud);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (minPoint.getY(), maxPoint.getY());  // 0 ~ 10m
        pass.setFilterLimitsNegative (false);
        pass.filter (*filtered_cloud);

        pass.setInputCloud (filtered_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (minPoint.getZ(), maxPoint.getZ());  // 0 ~ 1m
        pass.setFilterLimitsNegative (false);
        pass.filter (*filtered_cloud);

//        std::cerr << "Pass Cloud " << filtered_cloud->points.size () << std::endl;

        // // Convert the points to voxel grid points
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud (filtered_cloud);
        vg.setLeafSize (filterRes, filterRes,filterRes);
        vg.setDownsampleAllData (true);
        vg.filter (*filtered_cloud);
//        std::cerr << "Voxeled " << filtered_cloud->points.size () << std::endl;

        return filtered_cloud;
    }

    std::pair<pointcloud::Ptr, pointcloud::Ptr> Processor::segmentPlane(const pointcloud::Ptr &input_cloud,
                                                                        const int &maxIterations,
                                                                        const double &distanceThreshold)
    {
        pcl::SACSegmentation<PointXYZI> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

        seg.setOptimizeCoefficients(true);
        seg.setInputCloud(input_cloud);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceThreshold);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
        }
        std::pair<pointcloud::Ptr, pointcloud::Ptr> segResult = seperateCloud(inliers, input_cloud);
        return segResult;
    }

    std::pair<pointcloud::Ptr, pointcloud::Ptr> Processor::seperateCloud(const pcl::PointIndices::Ptr &inliers,
                                                                          const pointcloud::Ptr &input_cloud)
    {
        pointcloud::Ptr obstacle_cloud(new pointcloud());
        pointcloud::Ptr plane_cloud(new pointcloud());

        for (const int &index : inliers->indices)
            plane_cloud->points.push_back(input_cloud->points[index]);

        pcl::ExtractIndices<PointXYZI> extract;
        extract.setInputCloud(input_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_cloud);
//        std::cerr << "the planar component: "
//                  << plane_cloud->width * plane_cloud->height
//                  << " data points."
//                  << std::endl;

        // Create the filtering object
        extract.setNegative (true);
        extract.filter(*obstacle_cloud);

//        std::cerr << "the Object component: "
//                  << obstacle_cloud->width * plane_cloud->height
//                  << " data points."
//                  << std::endl;
        std::pair<pointcloud::Ptr, pointcloud::Ptr> segResult(obstacle_cloud, plane_cloud);
        return segResult;
    }

    std::vector<pointcloud::Ptr> Processor::clusterCloud(const pointcloud::Ptr &cloud,
                                              const double &clusterTolerance,
                                              const int &minSize,
                                              const int &maxSize)
    {
        std::vector<pointcloud::Ptr> vec_cluster;
        vec_cluster.reserve(10000);
        pcl::search::KdTree<PointXYZI>::Ptr tree(new pcl::search::KdTree<PointXYZI>());
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        std::shared_ptr<pcl::EuclideanClusterExtraction<PointXYZI>> euclidean_cluster =
                std::make_shared<pcl::EuclideanClusterExtraction<PointXYZI>>();
        euclidean_cluster->setClusterTolerance(clusterTolerance);
        euclidean_cluster->setMinClusterSize(minSize);
        euclidean_cluster->setMaxClusterSize(maxSize);
        euclidean_cluster->setSearchMethod(tree);
        euclidean_cluster->setInputCloud(cloud);
        euclidean_cluster->extract(cluster_indices);

        for (const auto& get_indices : cluster_indices)
        {
            pointcloud::Ptr cluster_cloud(new pointcloud());
            for (const auto index : get_indices.indices)
            {
                cluster_cloud->points.push_back(cloud->points[index]);
            }

            cluster_cloud->width = cluster_cloud->points.size();
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = true;

            if (cluster_cloud->width >= minSize && cluster_cloud->width <= maxSize)
            {
                vec_cluster.emplace_back(cluster_cloud);
            }
        }
        return vec_cluster;
    }

    Box Processor::boundBox(const PointXYZI &minPoint, const PointXYZI &maxPoint)
    {
        Box box;
        box.x_min = minPoint.x;
        box.y_min = minPoint.y;
        box.z_min = minPoint.z;
        box.x_max = maxPoint.x;
        box.y_max = maxPoint.y;
        box.z_max = maxPoint.z;

//        box.x_mid = box.x_min + (box.x_max - box.x_min)/2 ;
//        box.y_mid = box.y_min + (box.y_max - box.y_min)/2 ;
        return box;
    }
}
