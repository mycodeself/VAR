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
#include <boost/thread/thread.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <sys/resource.h>

#define DEBUG_MSG 1

#define Descriptor 1

// tipo de punto a usar
#define PointType pcl::PointXYZRGB
// algoritmo descriptor a usar
#define DescriptorType pcl::SHOT352 // 1

// Constantes para RANSAC
#define DISTANCE_THRESHOLD 0.01
#define TYPE_RANSAC 1 // 0 - NO RANSAC, 1 - PLANO, 2 - ESFERA


void callback(const pcl::PointCloud<PointType>::ConstPtr& msg);

void simpleVis();

void cloud_visualizer(const std::string& name, 
						const pcl::PointCloud<PointType>::ConstPtr& cloud);

double get_cloud_resolution(const pcl::PointCloud<PointType>::ConstPtr& cloud);

//keypoints
void iss_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
							pcl::PointCloud<PointType>::Ptr keypoints);

void remove_nan(pcl::PointCloud<PointType>::Ptr cloud);

//descriptors
void SHOT352_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
							const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
							const pcl::PointCloud<PointType>::ConstPtr& cloud,
							pcl::PointCloud<pcl::SHOT352>::Ptr descriptors);

//filter voxelGrid
void filter_voxel_grid(const pcl::PointCloud<PointType>::ConstPtr& cloud,
							pcl::PointCloud<PointType>::Ptr cloud_filtered);

void estimate_normals(const pcl::PointCloud<PointType>::ConstPtr& cloud,
							pcl::PointCloud<pcl::Normal>::Ptr normals);
	
void find_correspondences(const pcl::PointCloud<DescriptorType>::ConstPtr& descriptors,
							pcl::CorrespondencesPtr correspondences);
	
void cluster_geometric_consistency(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
									const pcl::CorrespondencesConstPtr& correspondences);


void cluster_hough3d(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
						const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
						const pcl::PointCloud<PointType>::ConstPtr& cloud,
						const pcl::CorrespondencesConstPtr& correspondences);

// Devuelve true si se ha encontrado correspondencias suficientes
bool ransac(const pcl::PointCloud<PointType>::Ptr &cloud, 
				pcl::PointCloud<PointType>::Ptr &cloudFinal);

bool ransac_alignment(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						const pcl::PointCloud<DescriptorType>::ConstPtr& descriptors,
						pcl::PointCloud<PointType>::Ptr cloud_aligned);


double get_cpu_time();

#endif
