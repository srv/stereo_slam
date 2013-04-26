#ifndef UTILS
#define UTILS

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float32.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace stereo_localization
{

class Utils
{
  public:

  /** \brief Compute the centroid of a point cloud
  	* @return a vector with the x,y,z centroid
    * \param point_cloud the source PointCloud
    * \param min_x minimum x value to take into account
    * \param max_x maximum x value to take into account
    * \param min_y minimum y value to take into account
    * \param max_y maximum y value to take into account
    * \param min_z minimum z value to take into account
    * \param max_z maximum z value to take into account
    */
  tf::Vector3 computeCentroid(PointCloud::Ptr point_cloud, double min_x, double max_x,
	double min_y, double max_y, double min_z, double max_z)
	{
	  double mean_x, mean_y, mean_z;
	  mean_x = 0.0;
	  mean_y = 0.0;
	  mean_z = 0.0;

		int count = 0;
		for (size_t i = 0; i < point_cloud->points.size(); ++i)
		{
		  const pcl::PointXYZ& point = point_cloud->points[i];
		  if (point.x >= min_x && point.x <= max_x &&
		      point.y >= min_y && point.y <= max_y &&
		      point.z >= min_z && point.z <= max_z &&
		      !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
		  {
		    mean_x += point.x;
		    mean_y += point.y;
		    mean_z += point.z;
		    count++;
		  }
		}
		if (count == 0)
		{
		  mean_x = -1;
		  mean_y = -1;
		  mean_z = -1;
		}
		else
		{
		  mean_x /= count;
		  mean_y /= count;
		  mean_z /= count;
		}
		 
		tf::Vector3 centroid = tf::Vector3(mean_x, mean_y, mean_z);

		return centroid;
	}

	/** \brief extract the keypoints of some image
  	* @return 
    * \param image the source image
    * \param key_points is the pointer for the resulting image key_points
    */
	void keypointDetector(const cv::Mat& image, std::vector<cv::KeyPoint>& key_points)
	{
		cv::initModule_nonfree();
		cv::Ptr<cv::FeatureDetector> cv_detector_;
		cv_detector_ = cv::FeatureDetector::create("SIFT");
  	cv_detector_->detect(image, key_points);
	}

	/** \brief extract the sift descriptors of some image
  	* @return 
    * \param image the source image
    * \param key_points keypoints of the source image
    * \param descriptors is the pointer for the resulting image descriptors
    */
	void descriptorExtraction(const cv::Mat& image, std::vector<cv::KeyPoint>& key_points, cv::Mat& descriptors)
	{
		cv::initModule_nonfree();
	  cv::Ptr<cv::DescriptorExtractor> cv_extractor_;
	  cv_extractor_ = cv::DescriptorExtractor::create("SIFT");
	  cv_extractor_->compute(image, key_points, descriptors);
	}	
};

} // namespace

#endif // UTILS


