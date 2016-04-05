#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <message_filters/synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>

#define USE_STITCHER_CLASS 1 // macro que define si se usará la clase stitcher proporcionada por OpenCV
#define SHOW_CAPTURE_IMAGES 0 // macro que define si se mostrarán las imagenes capturadas por las cámaras para debug

class Panorama {
	private:
		ros::NodeHandle m_nh;
		image_transport::ImageTransport m_it;
		image_transport::SubscriberFilter m_leftCamSub;
		image_transport::SubscriberFilter m_rightCamSub;
		typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image, sensor_msgs::Image> t_SyncPolicy;	
		message_filters::Synchronizer<t_SyncPolicy> m_sync;
		cv::Mat m_leftImgMat;
		cv::Mat m_rightImgMat;	

		void StitchingImage()
		{
#if SHOW_CAPTURE_IMAGES
			cv::imshow("leftImage", m_leftImgMat);
			cv::imshow("rightImage", m_rightImgMat);
#endif
#if USE_STITCHER_CLASS
			std::vector<cv::Mat> images;
			cv::Mat resultImage;
			images.push_back(m_rightImgMat);
			images.push_back(m_leftImgMat);
			
			cv::Stitcher stitcher = cv::Stitcher::createDefault(true);
			stitcher.setPanoConfidenceThresh(0);
			cv::Stitcher::Status status = stitcher.stitch(images, resultImage);
			if(cv::Stitcher::OK == status)
			{
				cv::imshow("view", resultImage);
				cv::waitKey(30);
			}else{
				std::cout << "STATUS: " << status << "\n";
				std::cout << "ERROR\n";
			}
#else
			cv::Mat leftImgGray, rightImgGray;
			// convertimos a escala de grises para mejor obtención de keypoints
			cv::cvtColor(m_leftImgMat, leftImgGray, CV_RGB2GRAY);
			cv::cvtColor(m_rightImgMat, rightImgGray, CV_RGB2GRAY);
			// empezamos a obtener los keypoints con SURF
			cv::SurfFeatureDetector detector(300);
			std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
			detector.detect(leftImgGray, keypoints_left);
			detector.detect(rightImgGray, keypoints_right);

			// obtenemos los descriptores
			cv::SurfDescriptorExtractor extractor;
			cv::Mat descriptors_left, descriptors_right;
			extractor.compute(leftImgGray, keypoints_left, descriptors_left);
			extractor.compute(rightImgGray, keypoints_right, descriptors_right);

			// comparamos los descriptores obtenidos mediante el algoritmo FLANN
			cv::FlannBasedMatcher matcher;
			std::vector<cv::DMatch> matches;
			matcher.match(descriptors_left, descriptors_right, matches);

			double max_dist = 0, min_dist = 1000, dist = 0;
			for(int i=0;i<descriptors_left.rows;i++)
			{
				dist = matches[i].distance;
				if(dist < min_dist) min_dist = dist;
				if(dist > max_dist) max_dist = dist;
			}

			std::vector<cv::DMatch> good_matches;
			for(int i=0;i<descriptors_left.rows;i++)
			{
				if(matches[i].distance < 4*min_dist){
					good_matches.push_back(matches[i]);
				}
			} 

			std::vector<cv::Point2f> left, right;
			for(int i=0;i<good_matches.size();i++)
			{
				left.push_back(keypoints_left[good_matches[i].queryIdx].pt);
				right.push_back(keypoints_right[good_matches[i].trainIdx].pt);
			}
			if(left.size() < 4 || right.size() < 4) return;

			// obtenemos la matriz homográfica mediante RANSAC
			cv::Mat homography = cv::findHomography(left, right, CV_RANSAC);
			cv::Mat result;
			cv::warpPerspective(m_leftImgMat, result, homography, cv::Size(m_leftImgMat.cols+m_rightImgMat.cols, m_leftImgMat.rows));
			cv::Mat half(result, cv::Rect(0,0,m_rightImgMat.cols, m_rightImgMat.rows));
			m_rightImgMat.copyTo(half);
			cv::imshow("view", result);
			cv::waitKey(30);
#endif
		}	
	public:
		Panorama() :
		m_it(m_nh), 
		m_leftCamSub(m_it, "/robot6/trasera2/trasera2/rgb/image_raw", 1),
		m_rightCamSub(m_it, "/robot6/trasera1/trasera1/rgb/image_raw", 1),
		m_sync(t_SyncPolicy(10), m_leftCamSub, m_rightCamSub)
		{
			m_sync.registerCallback(boost::bind(&Panorama::ImageCallback, this, _1, _2));
			cv::namedWindow("view");
    		cv::startWindowThread();
#if SHOW_CAPTURE_IMAGES 
    		cv::namedWindow("leftImage");
    		cv::startWindowThread();
    		cv::namedWindow("rightImage");
    		cv::startWindowThread();
#endif
		}
		void ImageCallback(const sensor_msgs::ImageConstPtr& leftImg, 
						const sensor_msgs::ImageConstPtr& rightImg)
		{
			try {
				m_leftImgMat = cv_bridge::toCvCopy(leftImg, "bgr8")->image;
			}catch(cv_bridge::Exception& e){
				ROS_ERROR("Could not convert from '%s' to 'bgr8'.", leftImg->encoding.c_str());	
			}
			try {
				m_rightImgMat = cv_bridge::toCvCopy(rightImg, "bgr8")->image;
			}catch(cv_bridge::Exception& e){
				ROS_ERROR("Could not convert from '%s' to 'bgr8'.", rightImg->encoding.c_str());	
			}
		}
		void Init()
		{
			ros::Rate rate(10.0);
			while(ros::ok()) {
				if(!m_leftImgMat.empty() && !m_rightImgMat.empty()){
					StitchingImage();
				}
				ros::spinOnce();
				rate.sleep();
			}
		    ros::spin();
		    ros::shutdown();
		    cv::destroyWindow("view");
#if SHOW_CAPTURE_IMAGES
		    cv::destroyWindow("leftImage");
		    cv::destroyWindow("rightImage");
#endif
		}
};