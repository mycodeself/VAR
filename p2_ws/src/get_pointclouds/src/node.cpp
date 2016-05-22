/**
 * @file node.cpp
 * Implementacion del nodo que se encarga de procesar
 * los mensajes recibidos de la kinect implementacion
 *
 * @author Ismael Piñeiro Ramos
 */
#include "node.h"

void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

	while(!viewer.wasStopped())
	{
	  viewer.showCloud (final_cloud);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

void cloud_visualizer(const std::string& name, 
						const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
	pcl::visualization::CloudViewer viewer(name);
	while(!viewer.wasStopped()) {
		viewer.showCloud(cloud);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

void callback(const pcl::PointCloud<PointType>::ConstPtr& msg)
{
	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_original(new pcl::PointCloud<PointType>(*msg));
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>(*msg));
#if DEBUG_MSG
	cout << "Number of points captured: " << cloud->size() << "\n";
#endif
	// Eliminamos los NaN de la nube de puntos
	remove_nan(cloud);
	// Obtenemos la resolucion de la nube de puntos actual
	actual_res = get_cloud_resolution(cloud);
	// Estimamos las normales
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimate_normals(cloud, normals);

	// Obtenemos keypoints
	pcl::PointCloud<PointType>::Ptr keypoints(new pcl::PointCloud<PointType>());

#if KeypointsMethod	== 1
	iss_keypoints(cloud, keypoints);
#elif KeypointsMethod == 2
	sift_keypoints(cloud, keypoints);
#elif KeypointsMethod == 3
	harris_keypoints(cloud, keypoints);
#endif

	// Obtenemos descriptores
	pcl::PointCloud<DescriptorType>::Ptr descriptors(new pcl::PointCloud<DescriptorType>());

#if DescriptorMethod == 1
	SHOT352_descriptors(keypoints, cloud, descriptors);
#elif DescriptorMethod == 2
	FPFH_descriptors(keypoints, descriptors);
#elif DescriptorMethod == 3
	CVFH_descriptors(keypoints, descriptors);	
#endif
	if(!last_cloud->empty()) { 
		// hacemos matching
		pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
		// obtenemos correspondencias
		ransac_correspondences(cloud, correspondences);
		// refinamos con ICP
		iterative_closest_point(cloud);
		pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*cloud_original, *transformed_cloud, transformation);
		filter_voxel_grid(transformed_cloud, cloud_filtered);
	}else{
		// Es la primera nube la metemos sin más
		filter_voxel_grid(cloud_original, cloud_filtered);
	}

	*last_cloud = *cloud_original;
	*last_keypoints = *keypoints;
	*last_descriptors = *descriptors;
	*last_normals = *normals;
	*final_cloud += *cloud_filtered;

	/*cloud_filtered->clear();
	filter_voxel_grid(final_cloud, cloud_filtered);
	final_cloud->clear();
  	std::swap(final_cloud, cloud_filtered); 
*/


}

/**
 * Computa el tiempo de CPU actual
 * @return tiempo actual de CPU
 */
double get_cpu_time(void) 
{
    struct timeval tim;
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    tim=ru.ru_utime;
    double t=(double)tim.tv_sec + (double)tim.tv_usec / 1000000.0;
    tim=ru.ru_stime;
    t+=(double)tim.tv_sec + (double)tim.tv_usec / 1000000.0;
    return t;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<PointType> >("/camera/depth/points", 1, callback);

  boost::thread t(simpleVis);
  transformation.setIdentity();
  while(ros::ok())
  {
	ros::spinOnce();
  }

}