#ifndef _NODE_H_
#define _NODE_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <pcl/keypoints/iss_3d.h> // get_iss_keypoints
#include <pcl/impl/point_types.hpp> // get_SHOT352_descriptors
#include <pcl/features/shot_omp.h> // get_SHOT352_descriptors
#include <pcl/features/normal_3d.h> // pcl::NormalEstimation
#include <pcl/features/normal_3d_omp.h> // pcl::NormalEstimationOMP
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>

#define DEBUG_MSG 1

#define Descriptor 1

// tipo de punto a usar
#define PointType pcl::PointXYZRGB
// algoritmo descriptor a usar
#define DescriptorType pcl::SHOT352 // 1




void callback(const pcl::PointCloud<PointType>::ConstPtr& msg);
void simpleVis();

double get_cloud_resolution(const pcl::PointCloud<PointType>::Ptr cloud);

//keypoints
void iss_keypoints(const pcl::PointCloud<PointType>::Ptr cloud,
					const double& model_resolution,
					pcl::PointCloud<PointType>::Ptr keypoints);

//descriptors
void SHOT352_descriptors(const pcl::PointCloud<PointType>::Ptr keypoints,
							const pcl::PointCloud<pcl::Normal>::Ptr normals,
							const pcl::PointCloud<PointType>::Ptr cloud,
							pcl::PointCloud<pcl::SHOT352>::Ptr descriptors);

//filter voxelGrid
void filter_voxel_grid(const pcl::PointCloud<PointType>::Ptr cloud,
						pcl::PointCloud<PointType>::Ptr cloud_filtered);

void estimate_normals(const pcl::PointCloud<PointType>::Ptr cloud,
						pcl::PointCloud<pcl::Normal>::Ptr normals);

void find_correspondences(const pcl::PointCloud<DescriptorType>::Ptr actual_descriptors,
							pcl::PointCloud<DescriptorType>::Ptr world_descriptors);

#endif
