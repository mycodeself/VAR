#include "node.h"


void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while(!viewer.wasStopped())
	{
	  viewer.showCloud (visu_pc);
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
	pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>(*msg));

#if DEBUG_MSG
	cout << "Number of points captured: " << cloud->size() << "\n";
#endif
	remove_nan(cloud);
	actual_res = get_cloud_resolution(cloud);
	//estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimate_normals(cloud, normals);
	//keypoints
	pcl::PointCloud<PointType>::Ptr keypoints(new pcl::PointCloud<PointType>());
	iss_keypoints(cloud, keypoints);
	//descriptors
	pcl::PointCloud<DescriptorType>::Ptr descriptors(new pcl::PointCloud<DescriptorType>());

#if Descriptor == 1
	SHOT352_descriptors(keypoints, normals, cloud, descriptors);
#endif

	if(!last_cloud->empty()) { 
		// hacemos matching
		pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
		//pcl::CorrespondencesPtr bestCorrespondences (new pcl::Correspondences ());

		//find_correspondences(descriptors, correspondences);

		ransac_correspondences(keypoints, correspondences);

		//agrupamos
		if(ransac_alignment(cloud, descriptors, cloud_filtered)) {
		//	*visu_pc += *cloud_filtered;
			iterative_closest_point(cloud_filtered);
		}
		//cluster_geometric_consistency(keypoints, correspondences);
		/*cluster_hough3d(keypoints, normals, cloud, correspondences);
		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
		for(size_t i=0;i<rot_translations.size();++i) {
			
			pcl::transformPointCloud (*cloud, *rotated_model, rot_translations[i]);
			filter_voxel_grid(rotated_model, cloud_filtered);	
			*visu_pc += *cloud_filtered;		
		}*/
		//filter_voxel_grid(rotated_model, cloud_filtered);
	}else{
		filter_voxel_grid(cloud, cloud_filtered);
		*visu_pc += *cloud_filtered;
	}

	*last_cloud = *cloud;
	*last_keypoints = *keypoints;
	*last_descriptors = *descriptors;
	*last_normals = *normals;

	*visu_pc += *cloud_filtered;

}

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

  while(ros::ok())
  {
	ros::spinOnce();
  }

}