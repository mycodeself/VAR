#ifndef _KEYPOINTS_H_
#define _KEYPOINTS_H_

#include "common.h"

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


#endif

