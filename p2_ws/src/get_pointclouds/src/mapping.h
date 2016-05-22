/**
 * @file mapping.h
 * Funciones necesarias para el mapeado, que no son de obtencion de keypoints
 * ni de descriptores. Cabecera
 *
 * @author Ismael Piñeiro Ramos
 * @author Gacel Ivorra Rodriguez
 */
#ifndef _MAPPING_H_
#define _MAPPING_H_

#include "common.h"

#define NORMALS_RADIUS_SEARCH 0.01f

#define ICP_MAX_ITERATIONS 50
#define ICP_MAX_CORRESPONDENCE_DISTANCE 0.05
#define ICP_TRANSFORMATION_EPSILON 1e-8
#define ICP_EUCLIDEAN_FITNESS_EPSILON 1

#define RANSAC_MAX_ITERATIONS 1000
#define RANSAC_INLIER_THRESHOLD 0.01
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
 * Obtiene las correspondencias y las filtra para quedarnos unicamente
 * con las mejores correspondencias mediante el uso de RANSAC
 * @param keypoints nubde de puntos de entrada de la que calcular correspondencias
 * @param bestCorrespondences correspondencias de salida calculadas
 */
void ransac_correspondences(const pcl::PointCloud<PointType>::ConstPtr &keypoints,
							pcl::CorrespondencesPtr bestCorrespondences);

/**
 * Transformacion haciendo uso de la clase CorrespondenceRejectionSampleConsensus
 * @param keypoints nube de puntos a transformar
 * @param transformedCloud nube transformada
*/
void ransac_transform(const pcl::PointCloud<PointType>::ConstPtr &keypoints,
						pcl::PointCloud<PointType>::Ptr &transformedCloud);
/*
 * Transforma la nube y acumula la transformacion global
 * @param transform matriz de transformacion calculada
 * @param cloud nube a transformar
 * @param transformedCloud nube resultante de la transformacion
 */
void transform_cloud(const Eigen::Matrix4f &transform,
						const pcl::PointCloud<PointType>::ConstPtr &cloud,
						pcl::PointCloud<PointType>::Ptr &transformedCloud);
#endif