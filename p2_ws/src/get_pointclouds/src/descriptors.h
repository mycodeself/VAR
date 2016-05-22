#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

#include "common.h"
#include "mapping.h"

// SHOT352 PARAMETERS
#define SHOT352_RADIUS_SEARCH 0.05

// FPFH PARAMETERS
#define FPFH_RADIUS_SEARCH 0.05

// CVFH PARAMETERS
#define CVFH_EPS_ANGLE_THRESHOLD 5.0 / 180.0 * M_PI
#define CVFH_CURVATURE_THRESHOLD 1.0
#define CVFH_NORMALIZE_BINS false

/**
 * Computa los descriptores mediante el algoritmo SHOT352 de manera
 * paralelizada mediante OpenMP
 * @param keypoints Keypoints de los que calcular los descriptores
 * @param normals Estimacion de normales de las que calcular los descriptores
 * @param cloud Nube de puntso de la que calcular los descriptores
 * @param descriptors Nube de puntos de salida obteniendo los descriptores
 */
void SHOT352_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
							const pcl::PointCloud<PointType>::ConstPtr& cloud,
							pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors);

/**
 * 
 *
 */
void FPFH_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
						pcl::PointCloud<pcl::FPFHSignature33>::Ptr& descriptors);

/**
 *
 *
 */
void CVFH_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
						pcl::PointCloud<pcl::VFHSignature308>::Ptr& descriptors);
#endif