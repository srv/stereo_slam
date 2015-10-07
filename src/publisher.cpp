#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "publisher.h"

namespace slam
{

  Publisher::Publisher()
  {
    ros::NodeHandle nhp("~");
    pub_clustering_ = nhp.advertise<sensor_msgs::Image>("keypoints_clustering", 2, true);
  }

  void Publisher::publishClustering(const Frame frame)
  {
    if (pub_clustering_.getNumSubscribers() > 0)
      drawKeypointsClustering(frame);
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

} //namespace slam