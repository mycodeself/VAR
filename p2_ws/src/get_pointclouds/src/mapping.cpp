#include "mapping.h"

/* VARIABLES GLOBALES */
pcl::PointCloud<PointType>::Ptr visu_pc (new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr last_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr last_keypoints(new pcl::PointCloud<PointType>());
pcl::PointCloud<DescriptorType>::Ptr last_descriptors(new pcl::PointCloud<DescriptorType>());
pcl::PointCloud<pcl::Normal>::Ptr last_normals(new pcl::PointCloud<pcl::Normal>());

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_translations;

double actual_res = 0;


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




void ransac_correspondences(const pcl::PointCloud<PointType>::ConstPtr &keypoints,
							pcl::CorrespondencesPtr bestCorrespondences)
{
	// Estimate correspondences
	pcl::CorrespondencesPtr estimateCorrespondences (new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<PointType, PointType> corr_est;
	corr_est.setInputSource(keypoints);
	corr_est.setInputTarget(last_keypoints);
	corr_est.determineCorrespondences(*estimateCorrespondences);

	// Apply RANSAC
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointType>::Ptr crsc(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointType>);
    crsc->setInputSource(keypoints);
    crsc->setInputTarget(last_keypoints); 
    crsc->setInlierThreshold(0.01); 
    crsc->setMaximumIterations(10000); 
    crsc->setInputCorrespondences(estimateCorrespondences);
	crsc->getCorrespondences(*bestCorrespondences);

#if DEBUG_MSG
	std::cout << "Number of estimation correspondences: " << estimateCorrespondences->size() << "\n";
	std::cout << "Number of remaining correspondences: " << bestCorrespondences->size() << "\n";
#endif

}