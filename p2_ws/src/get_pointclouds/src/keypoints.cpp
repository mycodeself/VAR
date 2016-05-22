/**
 * @file keypoints.cpp
 * Algoritmos de obtenicion de keypoints mediante PCL
 * implementacion
 *
 * @Author Ismael Piñeiro Ramos
 */
#include "keypoints.h"

void iss_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					pcl::PointCloud<PointType>::Ptr& keypoints)
{
#if SHOW_TIME
	double before_time;
	before_time = get_cpu_time();
#endif
	pcl::ISSKeypoint3D<PointType, PointType> iss_detector;	
	//iss_detector
	iss_detector.setInputCloud(cloud);
	iss_detector.setSalientRadius(ISS_SALIENT_RADIUS*actual_res);
	iss_detector.setNonMaxRadius(ISS_NON_MAX_RADIUS*actual_res);
	iss_detector.compute(*keypoints);

#if DEBUG_MSG
	std::cout << "Number of keypoints with IIS detector: " << keypoints->size() << "\n";
#endif
#if SHOW_TIME
	std::cout << "ISS Time: " << get_cpu_time() - before_time << "sec\n";
#endif 

}

void sift_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					pcl::PointCloud<PointType>::Ptr& keypoints)
{
#if SHOW_TIME
	double before_time;
	before_time = get_cpu_time();
#endif
	pcl::SIFTKeypoint<PointType, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
	sift.setSearchMethod(tree);
	// min_scale, n_octaves, n_scales_per_octave
	sift.setScales(SIFT_MIN_SCALE, SIFT_N_OCTAVES, SIFT_N_SCALES_OCTAVE);
	sift.setMinimumContrast(SIFT_MINIMUM_CONTRAST);
	sift.setInputCloud(cloud);
	sift.compute(result);
	copyPointCloud(result, *keypoints);

#if DEBUG_MSG
	std::cout << "Number of keypoints with SIFT detector: " << keypoints->size() << "\n";
#endif	
#if SHOW_TIME
	std::cout << "SIFT Time: " << get_cpu_time() - before_time << "sec\n";
#endif
}

void harris_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						pcl::PointCloud<PointType>::Ptr& keypoints)
{
#if SHOW_TIME
	double before_time;
	before_time = get_cpu_time();
#endif
	pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
	detector.setNonMaxSupression(HARRIS_NON_MAX_SUPRESSION);
	detector.setInputCloud(cloud);
	detector.setThreshold(HARRIS_THRESHOLD);
	detector.compute(*result);
	copyPointCloud(*result, *keypoints);
#if DEBUG_MSG
	std::cout << "Number of keypoints with HARRIS detector: " << keypoints->size() << "\n";
#endif	
#if SHOW_TIME
	std::cout << "HARRIS Time: " << get_cpu_time() - before_time << "sec\n";
#endif
}