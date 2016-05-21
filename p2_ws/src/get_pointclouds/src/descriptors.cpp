#include "descriptors.h"

void SHOT352_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
							const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
							const pcl::PointCloud<PointType>::ConstPtr& cloud,
							pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors)
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

void FPFH_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
						pcl::PointCloud<pcl::FPFHSignature33>::Ptr& descriptors)
{
	pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimate_normals(keypoints, normals);
	fpfh.setNumberOfThreads(4);
	fpfh.setInputCloud(keypoints);
	fpfh.setInputNormals(normals);
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
	fpfh.setSearchMethod(tree);
	// Radio de busqueda, tiene que ser mayor que el utilizado al calcular las normales
	fpfh.setRadiusSearch(0.05);
	fpfh.compute(*descriptors);

#if DEBUG_MSG
	std::cout << "Number of descriptors with FPFH: " << descriptors->size() << "\n";
#endif	
}