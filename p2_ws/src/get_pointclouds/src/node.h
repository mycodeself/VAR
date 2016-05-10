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

#define DEBUG_MSG 1

#define PointType pcl::PointXYZRGB

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);
void simpleVis();

double get_cloud_resolution();

//keypoints
void iss_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints);

//descriptors
void SHOT352_descriptors(pcl::PointCloud<pcl::SHOT352>::Ptr descriptors, 
								const pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints);

void filter_voxel_grid();


#endif