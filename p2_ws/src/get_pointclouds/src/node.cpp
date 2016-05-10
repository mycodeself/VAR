#include "node.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);


void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while(!viewer.wasStopped())
	{
	  viewer.showCloud (visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}

}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
	*cloud = *msg;

#if DEBUG_MSG
	cout << "Number of points captured: " << cloud->size() << "\n";
#endif

	filter_voxel_grid();

	//keypoints
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	iss_keypoints(keypoints);

	//descriptors
	//pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>);
	//SHOT352_descriptors(descriptors, keypoints);


	visu_pc = cloud_filtered;
}

double get_cloud_resolution()
{
	double res = 0.0;
  	int n_points = 0, n_res;
  	std::vector<int> indices (2);
  	std::vector<float> sqr_distances (2);
  	pcl::search::KdTree<PointType> tree;
  	tree.setInputCloud (cloud); 
	for(size_t i=0;i<cloud->size();++i) {
		if(!pcl_isfinite((*cloud)[i].x)) 
			continue;
		n_res = tree.nearestKSearch (i, 2, indices, sqr_distances); 
		if (n_res == 2) {
      		res += sqrt (sqr_distances[1]);
      		++n_points;
    	} 
	}
	if(n_points != 0)
		res /= n_points;
	return res;
}

void filter_voxel_grid()
{
	pcl::VoxelGrid<pcl::PointXYZRGB> v_grid;
	v_grid.setInputCloud(cloud);
	v_grid.setLeafSize(0.01f, 0.01f, 0.01f);
	v_grid.filter(*cloud_filtered);

#if DEBUG_MSG
	std::cout << "Number of points after VoxelGrid: " << cloud_filtered->size() << "\n";
#endif

}

void iss_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints)
{
	pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
	iss_detector.setInputCloud(cloud_filtered);
	//iss_detector.setSalientRadius(1.0f);
	//iss_detector.setNonMaxRadius(1.0f);
	iss_detector.compute(*keypoints);

#if DEBUG_MSG
	std::cout << "Number of keypoints with IIS detector: " << keypoints->size() << "\n";
#endif

}

void SHOT352_descriptors(pcl::PointCloud<pcl::SHOT352>::Ptr descriptors, 
								const pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints)
{
	pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot_describer;
	shot_describer.setRadiusSearch(6.0f);
	shot_describer.setInputCloud(keypoints);
	//shot_describer.setInputNormals.(normals);
	shot_describer.setSearchSurface(cloud_filtered);
	shot_describer.compute(*descriptors);

#if DEBUG_MSG
	std::cout << "Number of descriptors with SHOT352: " << descriptors->size() << "\n";
#endif

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);

  boost::thread t(simpleVis);

  while(ros::ok())
  {
	ros::spinOnce();
  }

}
