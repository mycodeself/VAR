void cluster_geometric_consistency(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
									const pcl::CorrespondencesConstPtr& correspondences)
{
  	//std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_translations;
  	std::vector<pcl::Correspondences> clustered_corrs;	
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    
    gc_clusterer.setGCSize(0.01*actual_res);
    gc_clusterer.setGCThreshold(2*actual_res);

    gc_clusterer.setInputCloud(keypoints);
    gc_clusterer.setSceneCloud(last_keypoints);
    gc_clusterer.setModelSceneCorrespondences(correspondences);

    //gc_clusterer.cluster(clustered_corrs);
    gc_clusterer.recognize(rot_translations, clustered_corrs);

#if DEBUG_MSG
	std::cout << "Number of instances: " << rot_translations.size() << "\n";
#endif    
}

bool ransac(const pcl::PointCloud<PointType>::Ptr &cloud, 
                pcl::PointCloud<PointType>::Ptr &finalCloud)
{
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (TYPE_RANSAC == 1 || TYPE_RANSAC == 2)
        {
            // Modifica puntos a random
            if (i % 5 == 0)
                cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
            else if(i % 2 == 0)
                cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                          - (cloud->points[i].y * cloud->points[i].y));
            else
                cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                            - (cloud->points[i].y * cloud->points[i].y));    
        }
        else
        {
            if( i % 2 == 0)
                cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
            else
                cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
        }
    }   

    std::vector<int> inliers;

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelSphere<PointType>::Ptr
        model_s(new pcl::SampleConsensusModelSphere<PointType> (cloud));

    pcl::SampleConsensusModelPlane<PointType>::Ptr
        model_p (new pcl::SampleConsensusModelPlane<PointType> (cloud));

    bool isGood = false;

    if(TYPE_RANSAC == 1)
    {
        pcl::RandomSampleConsensus<PointType> ransac (model_p);
        ransac.setDistanceThreshold (DISTANCE_THRESHOLD);
        isGood = ransac.computeModel();
        ransac.getInliers(inliers);
    }
    else if (TYPE_RANSAC == 2)
    {
        pcl::RandomSampleConsensus<PointType> ransac (model_s);
        ransac.setDistanceThreshold (DISTANCE_THRESHOLD);
        isGood = ransac.computeModel();
        ransac.getInliers(inliers);
    }

    // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<PointType>(*cloud, inliers, *finalCloud);

#if DEBUG_MSG
    std::cout << "Number of inliers found: " << inliers.size() << "\n";
#endif

    return isGood;
}

void cluster_hough3d(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
                        const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
                        const pcl::PointCloud<PointType>::ConstPtr& cloud,
                        const pcl::CorrespondencesConstPtr& correspondences)
{
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr actual_cloud_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr last_cloud_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
    std::vector<pcl::Correspondences> clustered_corrs;
    pcl::BOARDLocalReferenceFrameEstimation<PointType, pcl::Normal, pcl::ReferenceFrame> rf_est;

    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (0.020);
    rf_est.setInputCloud (keypoints);
    rf_est.setInputNormals (normals);
    rf_est.setSearchSurface (cloud);
    rf_est.compute (*actual_cloud_rf);
    rf_est.setInputCloud (last_keypoints);
    rf_est.setInputNormals (last_normals);
    rf_est.setSearchSurface (last_cloud);
    rf_est.compute (*last_cloud_rf);

    pcl::Hough3DGrouping<PointType, PointType, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
    clusterer.setHoughBinSize (0.01);
    clusterer.setHoughThreshold (5.0);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);
    clusterer.setInputCloud (keypoints);
    clusterer.setInputRf (actual_cloud_rf);
    clusterer.setSceneCloud (last_keypoints);
    clusterer.setSceneRf (last_cloud_rf);
    clusterer.setModelSceneCorrespondences (correspondences);
    clusterer.recognize (rot_translations, clustered_corrs);    
#if DEBUG_MSG
    std::cout << "Number of instances: " << rot_translations.size() << "\n";
#endif      
}

void find_correspondences(const pcl::PointCloud<DescriptorType>::ConstPtr& descriptors,
                            pcl::CorrespondencesPtr correspondences)
{
    //pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());
    pcl::KdTreeFLANN<DescriptorType> match;
    match.setInputCloud(descriptors);
    for(size_t i=0;i<last_descriptors->size();++i) {
        std::vector<int> indices(1);
        std::vector<float> sqr(1);
        if(!pcl_isfinite(last_descriptors->at(i).descriptor[0])) // skip NaN
            continue; 
        // para SHOT252 hay correspondencia si el cuadrado de la distancia
        // del descritor es menor de 0.25
        int neighbours = match.nearestKSearch(last_descriptors->at(i), 1, indices, sqr);
        if(neighbours == 1 && sqr[0] < 0.25) {
            pcl::Correspondence c(indices[0], static_cast<int>(i), sqr[0]);
            correspondences->push_back(c);
        }
    }
#if DEBUG_MSG
    std::cout << "Number of correspondences found: " << correspondences->size() << "\n";
#endif

}   
