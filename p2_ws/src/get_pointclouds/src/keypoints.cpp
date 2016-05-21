#include "keypoints.h"

void iss_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					pcl::PointCloud<PointType>::Ptr& keypoints)
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

void sift_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					pcl::PointCloud<PointType>::Ptr& keypoints)
{
	pcl::SIFTKeypoint<PointType, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());
	sift.setSearchMethod(tree);
	// min_scale, n_octaves, n_scales_per_octave
	sift.setScales(0.001, 8, 8);
	sift.setMinimumContrast(0.0001);
	sift.setInputCloud(cloud);
	sift.compute(result);
	copyPointCloud(result, *keypoints);

#if DEBUG_MSG
	std::cout << "Number of keypoints with SIFT detector: " << keypoints->size() << "\n";
#endif	
}