#ifndef _COMMON_H_
#define _COMMON_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <pcl/keypoints/iss_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/features/board.h>
#include <pcl/common/time.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <sys/resource.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/keypoints/sift_keypoint.h>


#define DEBUG_MSG 1

/**
 * 1 == ISSKeypoints3D
 * 2 == SIFTKeypoint
 */
#define KeypointsMethod 2
/**
 * 1 == SHOT352
 * 2 == FPFH
 * No olvidar ajustar DescriptorType
 */	
#define DescriptorMethod 2
// tipo de punto
#define PointType pcl::PointXYZRGB
// algoritmo descriptor a usar
//#define DescriptorType pcl::SHOT352 // 1
#define DescriptorType pcl::FPFHSignature33 // 2

extern pcl::PointCloud<PointType>::Ptr visu_pc;
extern pcl::PointCloud<PointType>::Ptr last_cloud;
extern pcl::PointCloud<PointType>::Ptr last_keypoints;
extern pcl::PointCloud<DescriptorType>::Ptr last_descriptors;
extern pcl::PointCloud<pcl::Normal>::Ptr last_normals;
extern std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_translations;
extern double actual_res;

#endif