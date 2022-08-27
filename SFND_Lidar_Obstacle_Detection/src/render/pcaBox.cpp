//
// Created by Arvind Rao on 30.03.21.
//
/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../render/render.h"
#include "../render/box.h"
#include "../processPointClouds.cpp"
#include <chrono>
#include <string>
#include <unordered_set>

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);

    viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
    return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for(int i = 0; i < points.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = points[i][0];
        point.y = points[i][1];
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

int main ()
{

    // render flags
    bool renderBoxes = true;

    // Create viewer
    Box window;
    window.x_min = -10;
    window.x_max =  10;
    window.y_min = -10;
    window.y_max =  10;
    window.z_min =   0;
    window.z_max =   0;
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

    // Create data
    std::vector<std::vector<float>> points;
    std::vector<std::vector<float>> generators = { {1,0}, {2,1}, {3,2}, {4,3}, {5,4}, {6,5}, {7,6}, {8,7} };

    for (float i = -3.0f; i < 3.0; i+=1.0f) {
        for (auto &p : generators)
            points.push_back({ p[0] + 0.5f * i, p[1] });
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    renderPointCloud(viewer,cloud,"data", colors[0]);

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    Eigen::Matrix3f covariance;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

    //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);

    Eigen::Vector3f a = eigenVectorsPCA.col(1);
    Eigen::Vector3f b = eigenVectorsPCA.col(2);

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    auto pointProcessor = ProcessPointClouds<pcl::PointXYZ>();
    if (renderBoxes) {
       // Box box = pointProcessor.BoundingBox(cloud);
       BoxQ box = BoxQ();
       box.bboxQuaternion = bboxQuaternion;
       box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
       box.cube_length = maxPoint.z - minPoint.z;
       box.cube_width  = maxPoint.y - minPoint.y;
       renderBox(viewer, box, 0, colors[1], 0.25);
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }

}
