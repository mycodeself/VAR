#include "node.h"

pcl::PointCloud<PointType>::Ptr visu_pc (new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr last_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr last_keypoints(new pcl::PointCloud<PointType>());
pcl::PointCloud<DescriptorType>::Ptr last_descriptors(new pcl::PointCloud<DescriptorType>());
pcl::PointCloud<pcl::Normal>::Ptr last_normals(new pcl::PointCloud<pcl::Normal>());

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_translations;

double actual_res = 0;

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

void cloud_visualizer(const std::string& name, 
						const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
	pcl::visualization::CloudViewer viewer(name);
	while(!viewer.wasStopped()) {
		viewer.showCloud(cloud);
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
	remove_nan(cloud);
	actual_res = get_cloud_resolution(cloud);
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
		pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
		find_correspondences(descriptors, correspondences);
		//agrupamos
		if(ransac_alignment(cloud, descriptors, cloud_filtered)) {
		//	*visu_pc += *cloud_filtered;
			iterative_closest_point(cloud_filtered);
		}
		//cluster_geometric_consistency(keypoints, correspondences);
		/*cluster_hough3d(keypoints, normals, cloud, correspondences);
		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
		for(size_t i=0;i<rot_translations.size();++i) {
			
			pcl::transformPointCloud (*cloud, *rotated_model, rot_translations[i]);
			filter_voxel_grid(rotated_model, cloud_filtered);	
			*visu_pc += *cloud_filtered;		
		}*/
		//filter_voxel_grid(rotated_model, cloud_filtered);
	}else{
		filter_voxel_grid(cloud, cloud_filtered);
		*visu_pc += *cloud_filtered;
	}

	*last_cloud = *cloud;
	*last_keypoints = *keypoints;
	*last_descriptors = *descriptors;
	*last_normals = *normals;

	*visu_pc += *cloud_filtered;

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

void remove_nan(pcl::PointCloud<PointType>::Ptr cloud)
{
	pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>());
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output, indices);
	*cloud = *output;

#if DEBUG_MSG
	std::cout << "Number of points after remove_nan: " << cloud->size() << "\n";
#endif

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
	//iss_detector
	iss_detector.setInputCloud(cloud);
	iss_detector.setSalientRadius(6*actual_res);
	iss_detector.setNonMaxRadius(4*actual_res);
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
	shot_describer.setRadiusSearch(0.05);
	shot_describer.setNumberOfThreads(4);
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
	ne.setNumberOfThreads(4);
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

void find_correspondences(const pcl::PointCloud<DescriptorType>::ConstPtr& descriptors,
							pcl::CorrespondencesPtr correspondences)
{
	//pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
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

void cluster_geometric_consistency(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
									const pcl::CorrespondencesConstPtr& correspondences)
{
  	//std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_translations;
  	std::vector<pcl::Correspondences> clustered_corrs;	
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    
    gc_clusterer.setGCSize(0.01*actual_res);
    gc_clusterer.setGCThreshold(2*actual_res);

    gc_clusterer.setInputCloud(keypoints);
    gc_clusterer.setSceneCloud(last_keypoints);
    gc_clusterer.setModelSceneCorrespondences(correspondences);

    //gc_clusterer.cluster(clustered_corrs);
    gc_clusterer.recognize(rot_translations, clustered_corrs);

#if DEBUG_MSG
	std::cout << "Number of instances: " << rot_translations.size() << "\n";
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

void cluster_hough3d(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
						const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
						const pcl::PointCloud<PointType>::ConstPtr& cloud,
						const pcl::CorrespondencesConstPtr& correspondences)
{
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr actual_cloud_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr last_cloud_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
    std::vector<pcl::Correspondences> clustered_corrs;
    pcl::BOARDLocalReferenceFrameEstimation<PointType, pcl::Normal, pcl::ReferenceFrame> rf_est;

    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (0.020);
    rf_est.setInputCloud (keypoints);
    rf_est.setInputNormals (normals);
    rf_est.setSearchSurface (cloud);
    rf_est.compute (*actual_cloud_rf);
    rf_est.setInputCloud (last_keypoints);
    rf_est.setInputNormals (last_normals);
    rf_est.setSearchSurface (last_cloud);
    rf_est.compute (*last_cloud_rf);

    pcl::Hough3DGrouping<PointType, PointType, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
    clusterer.setHoughBinSize (0.01);
    clusterer.setHoughThreshold (5.0);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);
    clusterer.setInputCloud (keypoints);
    clusterer.setInputRf (actual_cloud_rf);
    clusterer.setSceneCloud (last_keypoints);
    clusterer.setSceneRf (last_cloud_rf);
    clusterer.setModelSceneCorrespondences (correspondences);
    clusterer.recognize (rot_translations, clustered_corrs);	
#if DEBUG_MSG
	std::cout << "Number of instances: " << rot_translations.size() << "\n";
#endif      
}

bool ransac_alignment(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						const pcl::PointCloud<DescriptorType>::ConstPtr& descriptors,
						pcl::PointCloud<PointType>::Ptr cloud_aligned)
{
  pcl::SampleConsensusPrerejective<PointType,PointType,DescriptorType> align;
  align.setInputSource (cloud);
  align.setSourceFeatures (descriptors);
  align.setInputTarget (last_cloud);
  align.setTargetFeatures (last_descriptors);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * 0.005); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*cloud_aligned);
  }
  return align.hasConverged();
}

double get_cpu_time(void) 
{
    struct timeval tim;
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    tim=ru.ru_utime;
    double t=(double)tim.tv_sec + (double)tim.tv_usec / 1000000.0;
    tim=ru.ru_stime;
    t+=(double)tim.tv_sec + (double)tim.tv_usec / 1000000.0;
    return t;
}

void iterative_closest_point(const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setInputSource(cloud);
	icp.setInputTarget(last_cloud);
	pcl::PointCloud<PointType> final;
	icp.align(final);
	if(icp.hasConverged()) {
		std::cout << "ICP has converged\n";
		*visu_pc += final;
	}
#if DEBUG_MSG
	std::cout << "ICP Score: " << icp.getFitnessScore() << "\n";
#endif
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
