#ifndef _KEYPOINTS_H_
#define _KEYPOINTS_H_

#include "common.h"

// ISS PARAMETERS
#define ISS_SALIENT_RADIUS 6
#define ISS_NON_MAX_RADIUS 4

//SIFT PARAMETERS
#define SIFT_MIN_SCALE 0.01
#define SIFT_N_OCTAVES 4
#define SIFT_N_SCALES_OCTAVE 5
#define SIFT_MINIMUM_CONTRAST 10

//HARRIS PARAMETERS
#define HARRIS_NON_MAX_SUPRESSION true
#define HARRIS_THRESHOLD 1e-9
/**
 * Calcula los keypoints de una nube de puntos, utilizando
 * el algoritmo de ISSKeypoint3D
 * @param cloud Nube de puntos de entrada de la que calcular los keypoints
 * @param keypoints Nube de puntos de salida con los keypoints calculados
 */
void iss_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					pcl::PointCloud<PointType>::Ptr& keypoints);

/**
 * Calcula los keypoints de una nube de puntos, utilizando
 * el alrogirtmo SIFT
 * @param cloud Nube de puntos de entrada de la que calcular los keypoints
 * @param keypoints Nube de puntos de salida con los keypoints calculados
 */					
void sift_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					pcl::PointCloud<PointType>::Ptr& keypoints);

/**
 * Calcula los keypoints de una nube de puntos, utilizando el algoritmo
 * HarrisKeypoint3D
 * @param cloud Nube de puntos de entrada de la que calcular los keypoints
 * @param keypoints Nube de puntos de salida con los keypoints calculados
 */
void harris_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						pcl::PointCloud<PointType>::Ptr& keypoints);

#endif

