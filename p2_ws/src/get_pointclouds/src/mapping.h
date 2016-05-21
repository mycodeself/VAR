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
 * Calcula los keypoints de una nube de puntos, utilizando
 * el algoritmo de ISSKeypoint3D
 * @param cloud Nube de puntos de entrada de la que calcular los keypoints
 * @param keypoints Nube de puntos de salida con los keypoints calculados
 */
void iss_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					pcl::PointCloud<PointType>::Ptr keypoints);

/**
 * Computa los descriptores mediante el algoritmo SHOT352 de manera
 * paralelizada mediante OpenMP
 * @param keypoints Keypoints de los que calcular los descriptores
 * @param normals Estimacion de normales de las que calcular los descriptores
 * @param cloud Nube de puntso de la que calcular los descriptores
 * @param descriptors Nube de puntos de salida obteniendo los descriptores
 */
void SHOT352_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
							const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
							const pcl::PointCloud<PointType>::ConstPtr& cloud,
							pcl::PointCloud<pcl::SHOT352>::Ptr descriptors);

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
void find_correspondences(const pcl::PointCloud<DescriptorType>::ConstPtr& descriptors,
							pcl::CorrespondencesPtr correspondences);

/**
 *
 *
 */
void ransac_correspondences(const pcl::PointCloud<PointType>::ConstPtr &keypoints,
							pcl::CorrespondencesPtr bestCorrespondences);


#endif