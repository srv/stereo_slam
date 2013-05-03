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
  static tf::Vector3 computeCentroid(PointCloud::Ptr point_cloud, double min_x, double max_x,
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
	static void keypointDetector(const cv::Mat& image, std::vector<cv::KeyPoint>& key_points)
	{
		cv::initModule_nonfree();
		cv::Ptr<cv::FeatureDetector> cv_detector;
		cv_detector = cv::FeatureDetector::create("SIFT");
  	cv_detector->detect(image, key_points);
	}

	/** \brief extract the sift descriptors of some image
  	* @return 
    * \param image the source image
    * \param key_points keypoints of the source image
    * \param descriptors is the pointer for the resulting image descriptors
    */
	static void descriptorExtraction(const cv::Mat& image,
	 std::vector<cv::KeyPoint>& key_points, cv::Mat& descriptors)
	{
	  cv::Ptr<cv::DescriptorExtractor> cv_extractor;
	  cv_extractor = cv::DescriptorExtractor::create("SIFT");
	  cv_extractor->compute(image, key_points, descriptors);
	}

	/** \brief match descriptors of 2 images by threshold
  	* @return 
    * \param descriptors1 descriptors of image1
    * \param descriptors2 descriptors of image2
    * \param matches between both descriptors
    * \param matching_threshold threshold to determine correct matchings
    */
	static void thresholdMatching(const cv::Mat& descriptors1, 
		const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches, 
		double matching_threshold)
	{
		std::vector<std::vector<cv::DMatch> > matches12;

		cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
		descriptorMatcher = cv::DescriptorMatcher::create("FlannBased");

	  matches.clear();
	  matches12.clear();
	  int knn = 2;
	  descriptorMatcher->knnMatch(descriptors1, descriptors2, matches12, knn);
	  for( size_t m = 0; m < matches12.size(); m++ )
	  {
	    if (matches12[m].size() == 1)
	    {
	      matches.push_back(matches12[m][0]);
	    }
	    else if (matches12[m].size() == 2) // normal case
	    {
	      if (matches12[m][0].distance / matches12[m][1].distance 
	          < matching_threshold)
	      {
	        matches.push_back(matches12[m][0]);
	      }
	    }
	  }
	}

	/** \brief convert a matrix of type cv::Mat to std::vector
  	* @return std::vector matrix
    * \param input of type cv::Mat
    */
	static std::vector< std::vector<float> > cvMatToStdMatrix(cv::Mat input)
	{
		int round_zeros = 1000000000;

		std::vector< std::vector<float> > output;
		for (int i=0; i<input.rows; i++)
		{
		  std::vector<float> vec;
		  for (int j=0; j<input.cols; j++)
		  {
		  	float val = round((float)input.at<float>(i,j)*round_zeros)/round_zeros;
		    vec.push_back(val);
		  }
		  output.push_back(vec);
		}
		return output;
	}

	/** \brief convert a matrix of type std::vector to cv::Mat
  	* @return cv::Mat matrix
    * \param input of type std::vector< std::vector<float> >
    */
	static cv::Mat stdMatrixToCvMat(std::vector< std::vector<float> > input)
	{
		if (input.size() > 0)
		{
			std::vector<float> row0 = input[0];
			cv::Mat output(input.size(), row0.size(), cv::DataType<float>::type);

			for (unsigned int i=0; i<input.size(); i++)
			{
				std::vector<float> row = (std::vector<float>)input[i];
				for (unsigned int j=0; j<row.size(); j++)
				{
					output.at<float>(i,j) = (float)row[j];
				}
			}
			return output;
		}
		else
		{
			cv::Mat empty;
			return empty;
		}
	}	
};

} // namespace

#endif // UTILS


