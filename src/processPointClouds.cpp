// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloud_voxelgrid( new typename pcl::PointCloud<PointT> ());
    std::vector<int> cloudIndices;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new typename pcl::PointCloud<PointT> ());

  sor.setInputCloud (cloud);
  sor.setLeafSize (filterRes, filterRes, filterRes);
  sor.filter (*cloud_voxelgrid);

 // std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
   //    << " data points (" << pcl::getFieldsList (*voxelgrid) << ").";

    pcl::CropBox<PointT> cb(true);
    

    cb.setInputCloud(cloud_voxelgrid);
    cb.setMax(maxPoint);
    cb.setMin(minPoint);
    cb.filter(*cloud_filtered);

    // for (const auto &it : cloudIndices)
    // {
    //     cloud_filtered->points.push_back (cloud->points[it]); 
    //     cloud_filtered->width = cloud_filtered->points.size ();
    //     cloud_filtered->height = 1;
    //     cloud_filtered->is_dense = true;
    // }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_o{new pcl::PointCloud<PointT> ()};
    typename pcl::PointCloud<PointT>::Ptr cloud_p{new pcl::PointCloud<PointT> ()};
    for (auto index:inliers->indices)
    {
        cloud_p->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_o);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_o, cloud_p);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices ()};
    // TODO:: Fill in this function to find inliers for the cloud.
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // pcl::SACSegmentation<PointT> seg;

    // seg.setOptimizeCoefficients(true);
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (maxIterations);
    // seg.setDistanceThreshold (distanceThreshold);

    // seg.setInputCloud (cloud);
    // seg.segment (*inliers, *coefficients);
    inliers->indices = ransacPlane(cloud, maxIterations, distanceThreshold); //Akash Changes - using self-created Ransac model
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud,cloud);
    return segResult;
}
template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(std::vector<std::vector<float>>& points, int id, std::vector<int>* cluster, KdTree* tree, float distanceTol, std::vector<int> &processedPoints)
{
	processedPoints.push_back(id);
	cluster->push_back(id);
	std::vector<int> nearby_ids=tree->search(points[id],distanceTol);
	for(int nearby_id:nearby_ids)
	{
		if (std::count(processedPoints.begin(),processedPoints.end(),nearby_id)==0)
		Proximity(points, nearby_id, cluster, tree, distanceTol, processedPoints);
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<int> processedPoints;
	for (int id =0; id<points.size();id++)
	{
		if (std::count(processedPoints.begin(),processedPoints.end(),id)==0)
		{
		std::vector<int>* cluster = new std::vector<int>;
		Proximity(points, id, cluster, tree, distanceTol, processedPoints);
		clusters.push_back(*cluster);
		}
	}
    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<int>> cluster_indices;
    //std::vector<pcl::PointIndices> cluster_indices;
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     
    KdTree* tree = new KdTree;
    
    std::vector<std::vector<float>> points_vec;

        for (const auto point : cloud->points) {

            points_vec.push_back({point.x, point.y, point.z});
        }
    
    for (int i=0; i<points_vec.size(); i++) 
    {      
    	tree->insert(points_vec[i],i);
    }
    // tree->setInputCloud (cloud);

    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ec.setClusterTolerance(clusterTolerance);
    // ec.setMinClusterSize(minSize);
    // ec.setMaxClusterSize(maxSize);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud);
    // ec.extract(cluster_indices);

    // int j=0;
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    // {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    //   cloud_cluster->points.push_back (cloud->points[*pit]); //*
    // cloud_cluster->width = cloud_cluster->points.size ();
    // cloud_cluster->height = 1;
    // cloud_cluster->is_dense = true;

    // clusters.push_back(cloud_cluster);
    // }
    // auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    
    cluster_indices=euclideanCluster(points_vec, tree, clusterTolerance);
//     for ( const std::vector<int> &v : cluster_indices )
//     {
//    for ( int x : v ) std::cout << x << ' ';
//    std::cout << std::endl;
//     }

    for (const auto &it : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto &pit : it)
        cloud_cluster->points.push_back (cloud->points[pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        if(cloud_cluster->width > minSize && cloud_cluster->width < maxSize)
            clusters.push_back(cloud_cluster);
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
template<typename PointT>
std::vector<int> ProcessPointClouds<PointT>::ransacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::vector<int> inliersResult;
	std::unordered_set<int> inliers;
	srand(time(NULL));
	// TODO: Fill in this function
	// For max iterations 
	for (int i=0; i<maxIterations;i++)
	{
	// Randomly sample subset
	auto size = cloud->points.size();
    while (inliers.size()<3)
    {
        inliers.insert((rand() *size)/RAND_MAX);
    }
    auto itr = inliers.begin();
	auto first = cloud->points[*itr];
    itr++;
	auto second = cloud->points[*(itr)];
    itr++;
    auto third = cloud->points[*(itr)];
	std::vector<float> V1 {second.x-first.x, second.y-first.y, second.z-first.z};
    std::vector<float> V2 {third.x-first.x, third.y-first.y, third.z-first.z};
    std::vector<float> N {(V1[1]*V2[2]-V1[2]*V2[1]),(V1[2]*V2[0]-V1[0]*V2[2]),(V1[0]*V2[1]-V2[1]*V2[0])};

    auto A = N[0];
	auto B = N[1];
	auto C = N[2];
    auto D = -(N[0]*first.x+N[1]*first.y+N[2]*first.z);
	// Measure distance between every point and fitted line
	for (int j=0;j<size;j++)
	{
		auto dist = abs(A*cloud->points[j].x + B*cloud->points[j].y + C*cloud->points[j].z + D)/sqrt(A*A+B*B+C*C);
        
	// If distance is smaller than threshold count it as inlier
		if (dist<=distanceTol)
		{
			inliers.insert(j);
		} 
	}
	if (inliers.size()>inliersResult.size())
	{
        inliersResult.clear();
        inliersResult.insert(inliersResult.end(), inliers.begin(), inliers.end());
	}
	inliers.clear();
	}
	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;
}
