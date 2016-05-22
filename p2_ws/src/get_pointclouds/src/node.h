/**
 * @file node.h
 * Implementacion del nodo que se encarga de procesar
 * los mensajes recibidos de la kinect
 *
 * @author Ismael Piñeiro Ramos
 * @author Gacel Ivorra Rodriguez
 */
#ifndef _NODE_H_
#define _NODE_H_

#include "common.h"
#include "keypoints.h"
#include "descriptors.h"
#include "mapping.h"

/**
 * Callback donde realizaremos los calculos correspondientes
 * para obtener el mapeado en un visualizador
 * @param msg Nube de puntos recibida en el callback desde la kinect
 */
void callback(const pcl::PointCloud<PointType>::ConstPtr& msg);

/**
 * Lanza el visualizador por defecto donde se muestra el mapeado
 */
void simpleVis();

/**
 * Lanza un visualizador de PCL con la nube de puntos pasada por parametro
 * @param name Nombre de la ventana
 * @param cloud Nube de puntos a visualizar
 */
void cloud_visualizer(const std::string& name, 
						const pcl::PointCloud<PointType>::ConstPtr& cloud);

#endif