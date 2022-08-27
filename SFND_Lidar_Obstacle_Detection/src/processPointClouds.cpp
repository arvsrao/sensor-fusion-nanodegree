// PCL lib Functions for processing point clouds
#define DEBUG false

#include <unordered_set>
#include "processPointClouds.h"
#include "kdtree3d.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());

    pcl::VoxelGrid<PointT> sor;
    pcl::CropBox<PointT> cropBox;
    pcl::CropBox<PointT> removeEgoCar(true);

    removeEgoCar.setMin(Eigen::Vector4f(-1.5f, -1.7f, -1.0f,1.0f));
    removeEgoCar.setMax(Eigen::Vector4f(2.6f, 1.7f, 4.0f,1.0f));
    removeEgoCar.setInputCloud(cloud);
    removeEgoCar.filter(*filteredCloud);

    pcl::PointIndices pi;
    removeEgoCar.getRemovedIndices(pi);
    filteredCloud->points.clear();

    for (auto idx : pi.indices) filteredCloud->push_back(cloud->at(idx));

    cloud->points.clear();

    // crop input point cloud
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(filteredCloud);
    cropBox.filter(*cloud);

    // clear output buffer
    filteredCloud->points.clear();

    // voxel filter cropped point cloud
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    if (DEBUG)
        std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacle(new pcl::PointCloud<PointT>);

    int idy = 0;
    for (int idx = 0; idx < cloud->size(); idx++) {
        if (idy < inliers->indices.size() && inliers->indices.at(idy) == idx) {
            road->push_back(cloud->at(idx));
            idy++;
        }
        else obstacle->push_back(cloud->points.at(idx));
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, road);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    while (maxIterations--) {

        std::unordered_set<int> tempInliers;

        // Choose three points to make a plane. make sure the points are all different.
        while (tempInliers.size() < 3)
            tempInliers.insert(rand() % cloud->size());

        auto iter = tempInliers.begin();
        auto a = cloud->at(*iter);
        auto b = cloud->at(*(++iter));
        auto c = cloud->at(*(++iter));

        // compute the normal based at vec{a} by computing cross product
        auto ab = pcl::PointXYZ(b.x - a.x, b.y - a.y, b.z - a.z);
        auto ac = pcl::PointXYZ(c.x - a.x, c.y - a.y, c.z - a.z);
        auto eta = pcl::PointXYZ(ab.y * ac.z - ac.y * ab.z, ac.x * ab.z - ab.x * ac.z, ab.x * ac.y - ac.x * ab.y);
        auto eta_norm = std::sqrt(eta.x * eta.x + eta.y * eta.y + eta.z * eta.z);

        // Randomly sample and fit plane.
        // Iterate through points in cloud computing distance from plane through a & b & c.
        // Those that are within distanceTol from the plane are considered inliers.
        for (int idx = 0; idx < cloud->size(); idx++) {

            // ignore three points which generate the plane
            if (tempInliers.count(idx) != 0) continue;

            auto q = cloud->at(idx);
            auto dist_to_plane =
                    std::abs(eta.x * (q.x - a.x) + eta.y * (q.y - a.y) + eta.z * (q.z - a.z)) / eta_norm;

            if (dist_to_plane <= distanceTol)
                tempInliers.insert(idx);
        }

        if (tempInliers.size() > inliersResult.size()) {
            inliersResult = tempInliers;
        }

    }

    return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // INFO:: PCL function to find inliers for the cloud.
   /* pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold (distanceThreshold) ;

    // segment the largest planar component in the point cloud
    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);
    */
    std::unordered_set<int> inliers = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);

    if (inliers.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    if (DEBUG)
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>);

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    /*
     // Fill in the function to perform euclidean clustering to group detected obstacles
     // Creating the KdTree object for the search method of the extraction
     typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
     tree->setInputCloud (cloud);

     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<PointT> ec;
     ec.setClusterTolerance (clusterTolerance); // 2cm
     ec.setMinClusterSize (minSize);
     ec.setMaxClusterSize (maxSize);
     ec.setSearchMethod (tree);
     ec.setInputCloud (cloud);
     ec.extract (cluster_indices);

     for (auto indices : cluster_indices) {
       typename  pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
       for (auto idx : indices.indices) {
         temp->push_back(cloud->at(idx));
       }

       temp->width = temp->size();
       temp->height = 1;
       temp->is_dense = true;
       clusters.push_back(temp);
     }
    */

    std::vector<std::vector<float>> points;
    for (auto &pt : cloud->points) points.push_back({pt.x, pt.y, pt.z});

    auto tree = KdTree<3>(points);
    auto cluster_indices = tree.euclideanCluster(clusterTolerance);

    for (auto indices : cluster_indices) {

        // filter out clusters that are either too big or too small.
        if (indices.size() <= minSize || indices.size() >= maxSize) continue;

        typename pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
        temp->width = indices.size();
        temp->height = 1;
        temp->is_dense = true;

        // fill temp point cloud with cluster points
        for (auto idx : indices) temp->push_back(cloud->at(idx));

        clusters.push_back(temp);
    }

    if (DEBUG) {
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    }

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
