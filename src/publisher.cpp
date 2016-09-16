#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "publisher.h"

namespace slam
{

  Publisher::Publisher()
  {
    ros::NodeHandle nhp("~");
    pub_clustering_ = nhp.advertise<sensor_msgs::Image>("keypoints_clustering", 2, true);
    pub_stereo_matches_img_ = nhp.advertise<sensor_msgs::Image>("stereo_matches_img", 2, true);
    pub_stereo_matches_num_ = nhp.advertise<std_msgs::Int32>("stereo_matches_num", 2, true);
  }

  void Publisher::publishClustering(const Frame frame)
  {
    if (pub_clustering_.getNumSubscribers() > 0)
      drawKeypointsClustering(frame);
  }

  void Publisher::publishStereoMatches(const Frame frame)
  {
    if (pub_stereo_matches_img_.getNumSubscribers() > 0 ||
        pub_stereo_matches_num_.getNumSubscribers() > 0)
      drawStereoMatches(frame);
  }

  void Publisher::drawKeypointsClustering(const Frame frame)
  {
    vector< vector<int> > clusters = frame.getClusters();
    if (clusters.size() == 0) return;

    cv::Mat img;
    frame.getLeftImg().copyTo(img);
    vector<cv::KeyPoint> kp = frame.getLeftKp();
    cv::RNG rng(12345);
    for (uint i=0; i<clusters.size(); i++)
    {
      cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
      for (uint j=0; j<clusters[i].size(); j++)
        cv::circle(img, kp[clusters[i][j]].pt, 5, color, -1);
    }

    // Draw text
    stringstream s;
    int baseline = 0;
    s << " Number of clusters: " << clusters.size();
    cv::Size text_size = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1.5, 1, &baseline);
    cv::Mat im_text = cv::Mat(img.rows + text_size.height + 20, img.cols, img.type());
    img.copyTo(im_text.rowRange(0, img.rows).colRange(0, img.cols));
    im_text.rowRange(img.rows, im_text.rows).setTo(cv::Scalar(255,255,255));
    cv::putText(im_text, s.str(), cv::Point(5, im_text.rows - 10), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0,0,0), 2, 8);

    // Publish
    cv_bridge::CvImage ros_image;
    ros_image.image = im_text.clone();
    ros_image.header.stamp = ros::Time::now();
    ros_image.encoding = "bgr8";
    pub_clustering_.publish(ros_image.toImageMsg());
  }

  void Publisher::drawStereoMatches(const Frame frame)
  {
    cv::Mat out;
    cv::Mat l_img = frame.getLeftImg();
    cv::Mat r_img = frame.getRightImg();
    vector<cv::KeyPoint> l_kp = frame.getNonFilteredLeftKp();
    vector<cv::KeyPoint> r_kp = frame.getNonFilteredRightKp();
    vector<cv::DMatch> matches = frame.getMatches();

    if (pub_stereo_matches_img_.getNumSubscribers() > 0)
    {
      cv::drawMatches(l_img, l_kp, r_img, r_kp, matches, out);

      // Draw text
      stringstream s;
      int baseline = 0;
      s << " Number of matches: " << matches.size();
      cv::Size text_size = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1.5, 1, &baseline);
      cv::Mat im_text = cv::Mat(out.rows + text_size.height + 20, out.cols, out.type());
      out.copyTo(im_text.rowRange(0, out.rows).colRange(0, out.cols));
      im_text.rowRange(out.rows, im_text.rows).setTo(cv::Scalar(255,255,255));
      cv::putText(im_text, s.str(), cv::Point(5, im_text.rows - 10), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0,0,0), 2, 8);

      // Publish
      cv_bridge::CvImage ros_image;
      ros_image.image = im_text.clone();
      ros_image.header.stamp = ros::Time::now();
      ros_image.encoding = "bgr8";
      pub_stereo_matches_img_.publish(ros_image.toImageMsg());
    }

    if (pub_stereo_matches_num_.getNumSubscribers() > 0)
    {
      std_msgs::Int32 msg;
      msg.data = lexical_cast<int>(matches.size());
      pub_stereo_matches_num_.publish(msg);
    }

  }

} //namespace slam