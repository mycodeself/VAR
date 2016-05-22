/**
 * @file descriptors.cpp
 * Algoritmos de obtenicion de descriptores mediante PCL
 * implementacion
 *
 * @Author Ismael Piñeiro Ramos
 */
#include "descriptors.h"

void SHOT352_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
							const pcl::PointCloud<PointType>::ConstPtr& cloud,
							pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors)
{
	pcl::SHOTEstimationOMP<PointType, pcl::Normal, pcl::SHOT352> shot;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimate_normals(keypoints, normals);
	shot.setNumberOfThreads(4);
	shot.setRadiusSearch(SHOT352_RADIUS_SEARCH);
	shot.setInputCloud(keypoints);
	shot.setInputNormals(normals);
	shot.setSearchSurface(cloud);
	shot.compute(*descriptors);

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
	fpfh.setRadiusSearch(FPFH_RADIUS_SEARCH);
	fpfh.compute(*descriptors);

#if DEBUG_MSG
	std::cout << "Number of descriptors with FPFH: " << descriptors->size() << "\n";
#endif	
}

void CVFH_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
						pcl::PointCloud<pcl::VFHSignature308>::Ptr& descriptors)
{
	pcl::CVFHEstimation<PointType, pcl::Normal, pcl::VFHSignature308> cvfh;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimate_normals(keypoints, normals);	
	cvfh.setInputCloud(keypoints);
	cvfh.setInputNormals(normals);
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
	cvfh.setSearchMethod(tree);
	// Set the maximum allowable deviation of the normals,
	// for the region segmentation step.
	cvfh.setEPSAngleThreshold(CVFH_EPS_ANGLE_THRESHOLD); // 5 degrees.
	// Set the curvature threshold (maximum disparity between curvatures),
	// for the region segmentation step.
	cvfh.setCurvatureThreshold(CVFH_CURVATURE_THRESHOLD);
	// Set to true to normalize the bins of the resulting histogram,
	// using the total number of points. Note: enabling it will make CVFH
	// invariant to scale just like VFH, but the authors encourage the opposite.
	cvfh.setNormalizeBins(CVFH_NORMALIZE_BINS);
 
	cvfh.compute(*descriptors);	

#if DEBUG_MSG
	std::cout << "Number of descriptors with CVFH: " << descriptors->size() << "\n";
#endif		
}