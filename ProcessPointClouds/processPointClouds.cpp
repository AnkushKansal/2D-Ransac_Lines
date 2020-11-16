// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include<unordered_set>
#include<vector>


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
	typename pcl::PointCloud<PointT>::Ptr cloud_voxel_filtered(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloud_region_filtered(new pcl::PointCloud<PointT>());

	// Do voxel grid point reduction
	pcl::VoxelGrid<PointT> grid;
	grid.setInputCloud(cloud);
	grid.setLeafSize(filterRes, filterRes, filterRes);
	grid.filter(*cloud_voxel_filtered);

	// region based filtering
	pcl::CropBox<PointT> region(true);
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloud_voxel_filtered);
	region.filter(*cloud_region_filtered);	

	// remove roof points
	std::vector<int> indices;
	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
	roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
	roof.setInputCloud(cloud_region_filtered);
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	for (int i : indices)
		inliers->indices.push_back(i);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud_region_filtered);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_region_filtered); //putting back to filtered region

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>());

    //Simply fill the plane cloud as inliers contains only plane indices
    for(auto index : inliers->indices)
        plane->points.push_back(cloud->points[index]);

    //Now extract the rest of plane using inliers.
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true); //to negative or subtract
    extract.filter (*obstacles); //extract to obstacle variavle


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}

/* PCL version
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

       // Create the segmentation object
    pcl::SACSegmentation<PointT> varSegment;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); //sets coffiecients of the plane
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices ());    
     // Optional
    varSegment.setOptimizeCoefficients (true);
     // Mandatory
    varSegment.setModelType (pcl::SACMODEL_PLANE);
    varSegment.setMethodType (pcl::SAC_RANSAC);
    varSegment.setMaxIterations (maxIterations);
    varSegment.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    varSegment.setInputCloud (cloud);
    varSegment.segment (*inliers, *coefficients); //we are derencing the pointer
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}*/

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    std::unordered_set<int> bestinliersResult;
	srand(time(NULL));

	// For max iterations
	while (maxIterations--) {
		// Randomly sample subset of 3 and fit plane
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			inliers.insert(rand() % cloud->points.size());
		}

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float a, b, c, d, v3_i, v3_j, v3_k;
		// Let v1 be a vector from point 1 to point 2 in the plane
		// Let v2 be a vector from point 1 to point 3 in the plane
		// Let v3 equal the cross product v1 x v2
		v3_i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		v3_j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		v3_k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

		//plane is modeled by the equation Ax + By + Cz + D = 0
		a = v3_i;
		b = v3_j;
		c = v3_k;
		d = -(v3_i * x1 + v3_j * y1 + v3_k * z1);

		for (int index = 0; index < cloud->points.size(); index++) {
			// Skip if the considered point is already an inlier.
			if (inliers.count(index) > 0) continue;

			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			float distance = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

			if (distance <= distanceThreshold) {
				inliers.insert(index);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > bestinliersResult.size()) {
			bestinliersResult = inliers;
		}
	}

     // Extracting the clouds
	typename pcl::PointCloud<PointT>::Ptr  cloudPlane(new pcl::PointCloud<PointT>());  // The plane points
	typename pcl::PointCloud<PointT>::Ptr cloudObstacles(new pcl::PointCloud<PointT>());  // The not-plane points

	for (int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if (bestinliersResult.count(index))
			cloudPlane->points.push_back(point);
		else
			cloudObstacles->points.push_back(point);
	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	//returning pair of both clouds
	return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudObstacles, cloudPlane);
}

/*PCL clustering
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles    

    //euclidean clustering to group detected obstacles
	// Creating the KdTree object for the search method of the extraction
	typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> euclicd;
	euclicd.setClusterTolerance(clusterTolerance);
	euclicd.setMinClusterSize(minSize);
	euclicd.setMaxClusterSize(maxSize);
	euclicd.setSearchMethod(tree);
	euclicd.setInputCloud(cloud);
	euclicd.extract(cluster_indices);

	//Now iterate over each cluster indices and form set of clusters
	for (pcl::PointIndices getIndices : cluster_indices)
	{
		typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());

        //Adding all indexes of single cluster 
		for (auto index : getIndices.indices)
		{
			cluster->points.push_back(cloud->points[index]);
		}

		//cluster.width = cluster->points.size();
		///cluster.height = 1;
		//cluster.is_dense = true;

		clusters.push_back(cluster);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}*/

template<typename PointT>
void ProcessPointClouds<PointT>::nearbyPointsSearch(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int curr_index, std::vector<bool>& processed, std::vector<int>& cluster)
{
	// mark processed
	processed[curr_index] = true;
	cluster.push_back(curr_index);

	// search
	std::vector<int> nearby = tree->search(points[curr_index], distanceTol);

	// iterate other vector
	for (int id : nearby)
	{
		if (!processed[id])
			nearbyPointsSearch(points, tree, distanceTol, id, processed, cluster);
	}

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while (i < points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}

		// main cluster
		std::vector<int> cluster;
		nearbyPointsSearch(points, tree, distanceTol, i, processed, cluster);

		// push
		clusters.push_back(cluster);

		i++;
	}
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

	// Time clustering process
	auto startTime = std::chrono::steady_clock::now();

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	// TODO:: Fill in the function to perform euclidean clustering to group detected obstacles    

	// kd tree
	KdTree* tree = new KdTree();
	std::vector<std::vector<float>> points;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		PointT point = cloud->points[i];

		//as we cant insert the cloud point directly, we insert x, y, z into vector
		std::vector<float> point_vector;
		point_vector.push_back(point.x);
		point_vector.push_back(point.y);
		point_vector.push_back(point.z);

		tree->insert(point_vector, i);
		points.push_back(point_vector);
	}

	// cluster
	std::vector<std::vector<int>> clusters_ids = euclideanCluster(points, tree, clusterTolerance);

	for (std::vector<int> ids : clusters_ids)
	{
		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
		for (int indice : ids)
		{
			clusterCloud->points.push_back(cloud->points[indice]);
		}
		clusterCloud->width = clusterCloud->points.size();
		clusterCloud->height = 1;
		clusterCloud->is_dense = true;

		if ((clusterCloud->width >= minSize) && (clusterCloud->width <= maxSize))
			clusters.push_back(clusterCloud);
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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