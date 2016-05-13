#include "node.h"

pcl::PointCloud<PointType>::Ptr visu_pc (new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr world(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr last_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr last_keypoints(new pcl::PointCloud<PointType>());
pcl::PointCloud<DescriptorType>::Ptr last_descriptors(new pcl::PointCloud<DescriptorType>());

/*
* Cloud Visualizer
*/
void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while(!viewer.wasStopped())
	{
	  viewer.showCloud (visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}

}

void callback(const pcl::PointCloud<PointType>::ConstPtr& msg)
{
	pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>(*msg));

#if DEBUG_MSG
	cout << "Number of points captured: " << cloud->size() << "\n";
#endif

	//estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimate_normals(cloud, normals);
	//keypoints
	pcl::PointCloud<PointType>::Ptr keypoints(new pcl::PointCloud<PointType>());
	iss_keypoints(cloud, keypoints);
	//descriptors
	pcl::PointCloud<DescriptorType>::Ptr descriptors(new pcl::PointCloud<DescriptorType>());

#if Descriptor == 1
	SHOT352_descriptors(keypoints, normals, cloud, descriptors);
#endif

	if(!last_cloud->empty()) { 
		// hacemos matching
		find_correspondences(descriptors);
	}

	*last_cloud = *cloud;
	*last_keypoints = *keypoints;
	*last_descriptors = *descriptors;


	filter_voxel_grid(cloud, cloud_filtered);
	visu_pc = cloud_filtered;
}

double get_cloud_resolution(const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
	double res = 0.0;
  	int n_points = 0, n_res;
  	std::vector<int> indices (2);
  	std::vector<float> sqr_distances (2);
  	pcl::search::KdTree<PointType> tree;
  	tree.setInputCloud(cloud); 
	for(size_t i=0;i<cloud->size();++i) {
		if(!pcl_isfinite((*cloud)[i].x)) 
			continue;
		n_res = tree.nearestKSearch (i, 2, indices, sqr_distances); 
		if (n_res == 2) {
      		res += sqrt(sqr_distances[1]);
      		++n_points;
    	} 
	}
	if(n_points != 0)
		res /= n_points;
	return res;
}

void filter_voxel_grid(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						pcl::PointCloud<PointType>::Ptr cloud_filtered)
{
	pcl::VoxelGrid<PointType> v_grid;
	v_grid.setInputCloud(cloud);
	v_grid.setLeafSize(0.05f, 0.05f, 0.05f);
	v_grid.filter(*cloud_filtered);

#if DEBUG_MSG
	std::cout << "Number of points after VoxelGrid: " << cloud_filtered->size() << "\n";
#endif

}

void iss_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					pcl::PointCloud<PointType>::Ptr keypoints)
{

	pcl::ISSKeypoint3D<PointType, PointType> iss_detector;	
	//compute cloud model_resolution
	double model_resolution = get_cloud_resolution(cloud);
	//iss_detector
	iss_detector.setInputCloud(cloud);
	iss_detector.setSalientRadius(6*model_resolution);
	iss_detector.setNonMaxRadius(4*model_resolution);
	iss_detector.compute(*keypoints);

#if DEBUG_MSG
	std::cout << "Number of keypoints with IIS detector: " << keypoints->size() << "\n";
#endif

}

void SHOT352_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
							const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
							const pcl::PointCloud<PointType>::ConstPtr& cloud,
							pcl::PointCloud<pcl::SHOT352>::Ptr descriptors)
{
	pcl::SHOTEstimationOMP<PointType, pcl::Normal, pcl::SHOT352> shot_describer;
	shot_describer.setRadiusSearch(6.0f);
	shot_describer.setInputCloud(keypoints);
	shot_describer.setInputNormals(normals);
	shot_describer.setSearchSurface(cloud);
	shot_describer.compute(*descriptors);

#if DEBUG_MSG
	std::cout << "Number of descriptors with SHOT352: " << descriptors->size() << "\n";
#endif

}

void estimate_normals(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						pcl::PointCloud<pcl::Normal>::Ptr normals)
{
	pcl::NormalEstimationOMP<PointType, pcl::Normal> ne;
	//pcl::NormalEstimation<PointType, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
	ne.setSearchMethod(tree);
	//radio de vecinos
	ne.setRadiusSearch(0.01);
	ne.compute(*normals);

#if DEBUG_MSG
	std::cout << "Number of normal estimated: " << normals->size() << "\n";
#endif

}

void find_correspondences(const pcl::PointCloud<DescriptorType>::ConstPtr& descriptors)
{
	pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
	pcl::KdTreeFLANN<DescriptorType> match;
	match.setInputCloud(descriptors);
	for(size_t i=0;i<last_descriptors->size();++i) {
		std::vector<int> indices(1);
		std::vector<float> sqr(1);
		if(!pcl_isfinite(last_descriptors->at(i).descriptor[0])) // skip NaN
			continue; 
		// para SHOT252 hay correspondencia si el cuadrado de la distancia
		// del descritor es menor de 0.25
		int neighbours = match.nearestKSearch(last_descriptors->at(i), 1, indices, sqr);
		if(neighbours == 1 && sqr[0] < 0.25) {
			pcl::Correspondence c(indices[0], static_cast<int>(i), sqr[0]);
			correspondences->push_back(c);
		}
	}
#if DEBUG_MSG
	std::cout << "Number of correspondences found: " << correspondences->size() << "\n";
#endif
}	

bool ransac(const pcl::PointCloud<PointType>::Ptr &cloud, 
				pcl::PointCloud<PointType>::Ptr &finalCloud)
{
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (TYPE_RANSAC == 1 || TYPE_RANSAC == 2)
		{
			// Modifica puntos a random
			if (i % 5 == 0)
				cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
			else if(i % 2 == 0)
				cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
			                              - (cloud->points[i].y * cloud->points[i].y));
			else
				cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
			                                - (cloud->points[i].y * cloud->points[i].y));    
		}
		else
		{
			if( i % 2 == 0)
				cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
			else
				cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
		}
	}	

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelSphere<PointType>::Ptr
		model_s(new pcl::SampleConsensusModelSphere<PointType> (cloud));

	pcl::SampleConsensusModelPlane<PointType>::Ptr
		model_p (new pcl::SampleConsensusModelPlane<PointType> (cloud));

	bool isGood = false;

	if(TYPE_RANSAC == 1)
	{
		pcl::RandomSampleConsensus<PointType> ransac (model_p);
		ransac.setDistanceThreshold (DISTANCE_THRESHOLD);
		isGood = ransac.computeModel();
		ransac.getInliers(inliers);
	}
	else if (TYPE_RANSAC == 2)
	{
		pcl::RandomSampleConsensus<PointType> ransac (model_s);
		ransac.setDistanceThreshold (DISTANCE_THRESHOLD);
		isGood = ransac.computeModel();
		ransac.getInliers(inliers);
	}

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<PointType>(*cloud, inliers, *finalCloud);

#if DEBUG_MSG
	std::cout << "Number of inliers found: " << inliers.size() << "\n";
#endif

	return isGood;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<PointType> >("/camera/depth/points", 1, callback);

  boost::thread t(simpleVis);

  while(ros::ok())
  {
	ros::spinOnce();
  }

}
