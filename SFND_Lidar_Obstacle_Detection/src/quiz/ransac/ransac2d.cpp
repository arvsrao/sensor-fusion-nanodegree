/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("./src/sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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

        // Randomly sample subset and fit line
        // iterate through points in cloud computing distance from line through a & b
        // those that are within distanceTol are inliers
        for (int idx = 0; idx < cloud->size(); idx++) {

            // ignore three points of random plane
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

    while (maxIterations--) {

	    std::unordered_set<int> tempInliers;

        // make sure a != b
        while (tempInliers.size() < 2)
            tempInliers.insert(rand() % cloud-> size());

        auto iter = tempInliers.begin();
        auto a = cloud->at(*iter);
        auto b = cloud->at(*(++iter));

        auto ab = pcl::PointXYZ(b.x - a.x, b.y - a.y, 0);
        auto eta = pcl::PointXYZ(ab.y, -ab.x, 0);
        auto eta_norm = std::sqrt(eta.x * eta.x + eta.y * eta.y);

        // Randomly sample subset and fit line
        // iterate through points in cloud computing distance from line through a & b
        // those that are within distanceTol are inliers
        for (int idx = 0; idx < cloud->size(); idx++) {

            if (tempInliers.count(idx) != 0) continue;

            auto q = cloud->at(idx);
            auto dist_to_line = std::abs(eta.x * (q.x - a.x) + eta.y * (q.y - a.y)) / eta_norm;

//            auto t_perp = (-(p.x - b.x) * ab.x - (p.y - b.y) * ab.y) / length;
//            auto alpha_t_perp_to_p = pcl::PointXY(p.x + t_perp * ab.x - b.x, p.y + t_perp * ab.y - b.y);
//            auto dist_to_line = std::sqrt( alpha_t_perp_to_p.x * alpha_t_perp_to_p.x + alpha_t_perp_to_p.y * alpha_t_perp_to_p.y);

            if (dist_to_line <= distanceTol)
                tempInliers.insert(idx);
        }

        if (tempInliers.size() > inliersResult.size()) {
            inliersResult = tempInliers;
        }

    }
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 80, 0.25);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    /*pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    std::vector<int> inliers;
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (0.4);
    ransac.computeModel();
    ransac.setMaxIterations(80);
    ransac.getInliers(inliers);

    int idy = 0;
    for (int idx = 0; idx < cloud->size(); idx++) {
        if (idy < inliers.size() && inliers.at(idy) == idx) {
            cloudInliers->push_back(cloud->at(idx));
            idy++;
        }
        else cloudOutliers->push_back(cloud->points.at(idx));
    }*/

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
