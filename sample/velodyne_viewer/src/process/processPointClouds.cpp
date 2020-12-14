#include "process/processPointClouds.hpp"
//constructor:

template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// //de-constructor:
// template<typename PointT>
// ProcessPointClouds<PointT>::~ProcessPointClouds() {}

// template<typename PointT>
// std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
// {

//     std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

//     // sort files in accending order so playback is chronological
//     sort(paths.begin(), paths.end());

//     return paths;

// }


template<typename PointT>
void ProcessPointClouds<PointT>::laser2pcd (std::vector<velodyne::Laser> lasers, typename pcl::PointCloud<PointT>::Ptr& cloud)
{
    PointT point;

    for( const velodyne::Laser& laser : lasers ){
        const double distance = static_cast<double>( laser.distance );
        const double azimuth  = laser.azimuth  * CV_PI / 180.0;
        const double vertical = laser.vertical * CV_PI / 180.0;

        float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) ) ;
        float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) ) ;
        float z = static_cast<float>( ( distance * std::sin( vertical ) ) ) ;
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
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, float filterRes, const Vect3 minPoint, const Vect3 maxPoint)
{

    // Time segmentation process
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // // Convert the points to voxel grid points

    //    pcl::VoxelGrid<pcl::PointXYZI> vg;
    //    vg.setInputCloud (cloud);
    //    vg.setLeafSize (0.005, 0.005, 0.005);
    //    vg.setDownsampleAllData (true);
    //    vg.filter (*cloud_filtered);
    //    std::cerr << "Voxeled " << cloud_filtered->points.size () << std::endl;

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (minPoint.x, maxPoint.x);   // -2m ~ 2m
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (minPoint.y, maxPoint.y);  // 0 ~ 10m
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (minPoint.z, maxPoint.z);  // 0 ~ 1m
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);

    return cloud_filtered;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
        const pcl::PointIndices::Ptr& inliers, const typename pcl::PointCloud<PointT>::Ptr& cloud) {
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());

    // Copy inliers point cloud as plane
    for (int index : inliers->indices) {
        plane_cloud->points.push_back(cloud->points[index]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers so that we can get obstacles point cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}

// Use the pair object to hold your segmented results for the obstacle point cloud and the road point cloud
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, const int maxIterations, const float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    /*** PCL IMPLEMENTATION START ***/
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    // Optional
    seg.setOptimizeCoefficients(true);
    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(const typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize) {
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    clusters.reserve(60000);
    /*** Perform euclidean clustering to group detected obstacles ***/

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto& get_indices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>());

        for (const auto index : get_indices.indices)
        {
            cloud_cluster->points.push_back(cloud->points[index]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        if (cloud_cluster->width >= minSize && cloud_cluster->width <= maxSize)
        {
            clusters.emplace_back(cloud_cluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(const typename pcl::PointCloud<PointT>::Ptr& cloud) {
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(const PointT& minPoint, const PointT& maxPoint) {
    // Find bounding box for one of the clusters
//    PointT minPoint, maxPoint;
//    pcl::getMinMax3D(*cluster, minPoint, maxPoint);
    Box box{};

    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;
    box.x_mid = box.x_min + (box.x_max - box.x_min)/2 ;
    box.y_mid = box.y_min + (box.y_max - box.y_min)/2 ;

    return box;
}
