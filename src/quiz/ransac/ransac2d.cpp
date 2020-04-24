/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
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

// std::unordered_set<int> ransacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
// {
//     std::unordered_set<int> inliersResult;
// 	std::unordered_set<int> inliers;
// 	srand(time(NULL));
// 	// TODO: Fill in this function
// 	// For max iterations 
// 	for (int i=0; i<maxIterations;i++)
// 	{
// 	// Randomly sample subset
// 	auto size = cloud->points.size();
// 	auto first = cloud->points[(rand() *size)/RAND_MAX];
// 	auto second = cloud->points[(rand()*(size-1))/RAND_MAX];
// 	// while (second == first)
// 	// {
// 	// 	second = cloud->points[(rand()*(size-1))/RAND_MAX];
// 	// }
//     auto third = cloud->points[(rand()*(size-2))/RAND_MAX];
// 	// while (third == first || third == second)
// 	// {
// 	// 	third = cloud->points[(rand()*(size-2))/RAND_MAX];
// 	// }
//     //fitting a plane
// 	std::vector<float> V1 {second.x-first.x, second.y-first.y, second.z-first.z};
//     std::vector<float> V2 {third.x-first.x, third.y-first.y, third.z-first.z};
//     std::vector<float> N {(V1[1]*V2[2]-V1[2]*V2[1]),(V1[2]*V2[0]-V1[0]*V2[2]),(V1[0]*V2[1]-V2[1]*V2[0])};

//     auto A = N[0];
// 	auto B = N[1];
// 	auto C = N[2];
//     auto D = -(N[0]*first.x+N[1]*first.y+N[2]*first.z);
// 	// Measure distance between every point and fitted line
// 	for (int j=0;j<size;j++)
// 	{
// 		auto dist = abs(A*cloud->points[j].x + B*cloud->points[j].y + C*cloud->points[j].z + D)/sqrt(A*A+B*B+C*C);
        
// 	// If distance is smaller than threshold count it as inlier
// 		if (dist<=distanceTol)
// 		{
// 			inliers.insert(j);
// 		} 
// 	}
// 	if (inliers.size()>inliersResult.size())
// 	{
		
// 		inliersResult=inliers;

// 	}
// 	inliers.clear();
// 	}
// 	// Return indicies of inliers from fitted line with most inliers

// 	return inliersResult;
// }

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = ransacPlane(cloud, 1000, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

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
