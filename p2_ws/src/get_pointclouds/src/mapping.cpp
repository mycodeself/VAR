/**
 * @file mapping.cpp
 * Funciones necesarias para el mapeado, que no son de obtencion de keypoints
 * ni de descriptores. Implementacion
 *
 * @author Ismael Piñeiro Ramos
 */
#include "mapping.h"

/* VARIABLES GLOBALES */
pcl::PointCloud<PointType>::Ptr final_cloud (new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr last_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr last_keypoints(new pcl::PointCloud<PointType>());
pcl::PointCloud<DescriptorType>::Ptr last_descriptors(new pcl::PointCloud<DescriptorType>());
pcl::PointCloud<pcl::Normal>::Ptr last_normals(new pcl::PointCloud<pcl::Normal>());
Eigen::Affine3f transform_total = Eigen::Affine3f::Identity();

Eigen::Matrix4f transformation;

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
	ne.setNumberOfThreads(4);
	ne.setInputCloud(cloud);
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
	ne.setSearchMethod(tree);
	//radio de vecinos
	ne.setRadiusSearch(NORMALS_RADIUS_SEARCH);
	ne.compute(*normals);

#if DEBUG_MSG
	std::cout << "Number of normal estimated: " << normals->size() << "\n";
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
	icp.setMaxCorrespondenceDistance(ICP_MAX_CORRESPONDENCE_DISTANCE);
	icp.setMaximumIterations(ICP_MAX_ITERATIONS);
	icp.setTransformationEpsilon(ICP_TRANSFORMATION_EPSILON);
	icp.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_FITNESS_EPSILON);
	pcl::PointCloud<PointType> aligned_cloud;
	icp.align(aligned_cloud, transformation);
	if(icp.hasConverged())
		transformation = icp.getFinalTransformation();
#if DEBUG_MSG
	std::cout << "ICP Score: " << icp.getFitnessScore() << "\n";
	std::cout << "ICP matrix transformation: \n" << transformation << "\n";
#endif
}

void ransac_correspondences(const pcl::PointCloud<PointType>::ConstPtr &cloud,
							pcl::CorrespondencesPtr bestCorrespondences)
{
	// Estimate correspondences
	pcl::CorrespondencesPtr estimateCorrespondences (new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<PointType, PointType> corr_est;
	corr_est.setInputSource(cloud);
	corr_est.setInputTarget(last_cloud);
	corr_est.determineCorrespondences(*estimateCorrespondences);

	// Apply RANSAC
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> crsc;
    crsc.setInputSource(cloud);
    crsc.setInputTarget(last_cloud); 
    crsc.setInlierThreshold(RANSAC_INLIER_THRESHOLD); 
    crsc.setMaximumIterations(RANSAC_MAX_ITERATIONS); 
    crsc.setInputCorrespondences(estimateCorrespondences);
	crsc.getCorrespondences(*bestCorrespondences);
	transformation = crsc.getBestTransformation();
	
#if DEBUG_MSG
	std::cout << "Number of estimation correspondences: " << estimateCorrespondences->size() << "\n";
	std::cout << "Number of remaining correspondences: " << bestCorrespondences->size() << "\n";
	std::cout << "Matrix transformation: \n" << transformation << "\n";
#endif

}


void ransac_transform(const pcl::PointCloud<PointType>::ConstPtr &cloud,
						pcl::PointCloud<PointType>::Ptr &transformedCloud)
{
	Eigen::Matrix4f transform;
	pcl::CorrespondencesPtr bestCorrespondences(new pcl::Correspondences);

	// Estimate correspondences
	pcl::CorrespondencesPtr estimateCorrespondences (new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<PointType, PointType> corr_est;
	corr_est.setInputSource(cloud);
	corr_est.setInputTarget(last_cloud);
	corr_est.determineCorrespondences(*estimateCorrespondences);

	// Apply RANSAC
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointType>::Ptr crsc(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointType>);
    crsc->setInputSource(cloud);
    crsc->setInputTarget(last_cloud); 
    crsc->setInlierThreshold(0.05); 
    crsc->setMaximumIterations(5000); 
    crsc->setInputCorrespondences(estimateCorrespondences);
	crsc->getCorrespondences(*bestCorrespondences);
    crsc->setInputCorrespondences(bestCorrespondences);
	transform = crsc->getBestTransformation();

#if DEBUG_MSG
	std::cout << "Best transform matrix: " << "\n" << transform << "\n";
#endif

	transform_cloud(transform, cloud, transformedCloud);
	//pcl::transformPointCloud(*keypoints, *transformedCloud, transformTotal);
#if DEBUG_MSG
	std::cout << "Size of transformed cloud: " << transformedCloud->size() << "\n";
#endif
}

void transform_cloud(const Eigen::Matrix4f &transform,
						const pcl::PointCloud<PointType>::ConstPtr &cloud,
						pcl::PointCloud<PointType>::Ptr &transformedCloud)
{
	transform_total *= transform;
	pcl::transformPointCloud(*cloud, *transformedCloud, transform);

#if DEBUG_MSG
	std::cout << "TransformTotal matrix: " << transform_total.matrix() << "\n";
#endif
}