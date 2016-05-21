#ifndef _MAPPING_H_
#define _MAPPING_H_

#include "common.h"

/**
 * Calcula la resolución espacial de una nube de puntos dada,
 * mediante la media de la distancia entre cada punto y su
 * vecino mas cercano
 * @param cloud Nube de puntos de la que calcular la resolución
 */
double get_cloud_resolution(const pcl::PointCloud<PointType>::ConstPtr& cloud);

/**
 * Elimina los "Not a Number" de una nube de puntos
 * @param cloud Nube de puntos de la que eliminar los NaN
 */
void remove_nan(pcl::PointCloud<PointType>::Ptr cloud);

/**
 * Realiza un filtrado VoxelGrid sobre la nube de puntos de entrada,
 * aproximando los puntos mediante sus centroides.
 * @param cloud Nube de puntos de entrada a filtrar
 * @param cloud_filtered Nube de puntos de salida ya filtrada
 */
void filter_voxel_grid(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						pcl::PointCloud<PointType>::Ptr cloud_filtered);

/**
 * Estima las normales locales de una nube de puntos, utiliza paralelizacion
 * mediante OpenMP
 * @param cloud Nube de puntos de entrada de la que estimar las normales
 * @param normals Nube de puntos de salida de las normales estimadas
 */
void estimate_normals(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						pcl::PointCloud<pcl::Normal>::Ptr normals);

/**
 * Realiza el alineamiento de la nube de puntos actual con la anterior
 * utilizando el algoritmo SampleConsensusPrerejective
 * @param cloud Nube de puntos de entrada a alinear
 * @param descriptors Descriptores de la nube de puntos de entrada
 * @param cloud_aligned Nube de puntos de salida alineada
 */
bool ransac_alignment(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						const pcl::PointCloud<DescriptorType>::ConstPtr& descriptors,
						pcl::PointCloud<PointType>::Ptr cloud_aligned);

/**
 * Minimiza las distancias entre los puntos de dos nubes y las transforma
 * utilizando el algoritmo IterativeClosestPoint
 * @param cloud Nube de puntos de entrada
 */
void iterative_closest_point(const pcl::PointCloud<PointType>::ConstPtr& cloud);

/**
 *
 *
 */
void ransac_correspondences(const pcl::PointCloud<PointType>::ConstPtr &keypoints,
							pcl::CorrespondencesPtr bestCorrespondences);


#endif