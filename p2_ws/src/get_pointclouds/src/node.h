#ifndef _NODE_H_
#define _NODE_H_

#include "common.h"
#include "keypoints.h"
#include "descriptors.h"
#include "mapping.h"

/**
 *
 *
 */
void callback(const pcl::PointCloud<PointType>::ConstPtr& msg);

/**
 *
 *
 */
void simpleVis();

/**
 * Lanza un visualizador de PCL con la nube de puntos pasada por parametro
 * @param name Nombre de la ventana
 * @param cloud Nube de puntos a visualizar
 */
void cloud_visualizer(const std::string& name, 
						const pcl::PointCloud<PointType>::ConstPtr& cloud);


/**
 * Computa el tiempo de CPU actual
 * @return tiempo actual de CPU
 */
double get_cpu_time();

#endif