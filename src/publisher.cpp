#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "publisher.h"

namespace slam
{

  Publisher::Publisher()
  {
    ros::NodeHandle nhp("~");
    pub_stereo_matching_  = nhp.advertise<sensor_msgs::Image>("stereo_matching",   2, true);
    pub_tracker_matching_ = nhp.advertise<sensor_msgs::Image>("tracker_matching", 2, true);
    pub_clustering_ = nhp.advertise<sensor_msgs::Image>("keypoints_clustering", 2, true);
  }

  void Publisher::publishTracking(Tracking *tracker)
  {
    if (pub_stereo_matching_.getNumSubscribers() > 0)
      drawStereoMatchings(tracker->getCurrentFrame());

    if (pub_tracker_matching_.getNumSubscribers() > 0)
      drawTrackerMatchings(tracker->getFixedFrame(),
                           tracker->getCurrentFrame(),
                           tracker->getMatches(),
                           tracker->getInliers());
  }

  void Publisher::publishClustering(const Frame frame)
  {
    if (pub_clustering_.getNumSubscribers() > 0)
      drawKeypointsClustering(frame);
  }

  void Publisher::drawStereoMatchings(const Frame frame)
  {
    cv::Mat l_img, r_img;
    frame.getLeftImg().copyTo(l_img);
    frame.getRightImg().copyTo(r_img);
    vector<cv::KeyPoint> l_kp = frame.getLeftKp();
    vector<cv::KeyPoint> r_kp = frame.getRightKp();

    // Sanity check
    if (l_img.rows == 0 || l_img.cols == 0 || r_img.rows == 0 || r_img.cols == 0) return;

    // Concat images
    cv::Mat img;
    cv::hconcat(l_img, r_img, img);

    // Draw keypoints
    const float r = 5;
    for (uint i=0; i<l_kp.size(); i++)
    {
      cv::Point2f l_pt1, l_pt2, r_pt1, r_pt2;
      l_pt1.x = l_kp[i].pt.x-r;
      l_pt1.y = l_kp[i].pt.y-r;
      l_pt2.x = l_kp[i].pt.x+r;
      l_pt2.y = l_kp[i].pt.y+r;
      r_pt1.x = l_img.cols+r_kp[i].pt.x-r;
      r_pt1.y = r_kp[i].pt.y-r;
      r_pt2.x = l_img.cols+r_kp[i].pt.x+r;
      r_pt2.y = r_kp[i].pt.y+r;

      cv::Point2f rkp(l_img.cols+r_kp[i].pt.x,r_kp[i].pt.y);

      cv::rectangle(img, l_pt1, l_pt2, cv::Scalar(0,255,0));
      cv::rectangle(img, r_pt1, r_pt2, cv::Scalar(0,255,0));
      cv::circle(img, l_kp[i].pt, 2, cv::Scalar(0,255,0), -1);
      cv::circle(img, rkp, 2, cv::Scalar(0,255,0), -1);
      cv::line(img, l_kp[i].pt, rkp, cv::Scalar(0,255,0), 1, 8, 0);
    }

    // Draw text
    stringstream s;
    int baseline = 0;
    s << " Stereo Matches: " << l_kp.size();
    cv::Size text_size = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
    cv::Mat im_text = cv::Mat(img.rows + text_size.height + 10, img.cols, img.type());
    img.copyTo(im_text.rowRange(0, img.rows).colRange(0, img.cols));
    im_text.rowRange(img.rows, im_text.rows) = cv::Mat::zeros(text_size.height + 10, img.cols, img.type());
    cv::putText(im_text, s.str(), cv::Point(5, im_text.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 1, 8);

    // Publish
    cv_bridge::CvImage ros_image;
    ros_image.image = im_text.clone();
    ros_image.header.stamp = ros::Time::now();
    ros_image.encoding = "bgr8";
    pub_stereo_matching_.publish(ros_image.toImageMsg());
  }

  void Publisher::drawTrackerMatchings(const Frame fixed_frame,
                                            const Frame current_frame,
                                            const vector<cv::DMatch> matches,
                                            const vector<int> inliers)
  {
    cv::Mat f_img, c_img;
    fixed_frame.getLeftImg().copyTo(f_img);
    current_frame.getLeftImg().copyTo(c_img);
    vector<cv::KeyPoint> f_kp = fixed_frame.getLeftKp();
    vector<cv::KeyPoint> c_kp = current_frame.getLeftKp();

    // Sanity check
    if (f_img.rows == 0 || f_img.cols == 0 || c_img.rows == 0 || c_img.cols == 0) return;

    // Only draw true matchings
    vector<cv::Point2f> f_matched_kp, c_matched_kp;
    for(uint i=0; i<matches.size(); i++)
    {
      f_matched_kp.push_back(f_kp[matches[i].trainIdx].pt);
      c_matched_kp.push_back(c_kp[matches[i].queryIdx].pt);
    }

    // Concat images
    cv::Mat img;
    cv::vconcat(c_img, f_img, img);

    // Draw keypoints
    const float r = 5;
    for (uint i=0; i<f_matched_kp.size(); i++)
    {
      cv::Point2f f_pt1, f_pt2, c_pt1, c_pt2;
      f_pt1.x = f_matched_kp[i].x-r;
      f_pt1.y = c_img.rows+f_matched_kp[i].y-r;
      f_pt2.x = f_matched_kp[i].x+r;
      f_pt2.y = c_img.rows+f_matched_kp[i].y+r;
      c_pt1.x = c_matched_kp[i].x-r;
      c_pt1.y = c_matched_kp[i].y-r;
      c_pt2.x = c_matched_kp[i].x+r;
      c_pt2.y = c_matched_kp[i].y+r;

      cv::Point2f fkp(f_matched_kp[i].x, c_img.rows+f_matched_kp[i].y);

      // Is inlier?
      cv::Scalar color;
      if (find(inliers.begin(), inliers.end(), i) != inliers.end())
      {
        color = cv::Scalar(0,255,0);
        cv::line(img, c_matched_kp[i], fkp, color, 1, 8, 0);
      }
      else
        color = cv::Scalar(255,0,0);

      cv::rectangle(img, f_pt1, f_pt2, color);
      cv::rectangle(img, c_pt1, c_pt2, color);
      cv::circle(img, c_matched_kp[i], 2, color, -1);
      cv::circle(img, fkp, 2, color, -1);
    }

    // Draw text
    stringstream s;
    int baseline = 0;
    s << " Tracker Matches: " << f_matched_kp.size();
    cv::Size text_size = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
    cv::Mat im_text = cv::Mat(img.rows + text_size.height + 10, img.cols, img.type());
    img.copyTo(im_text.rowRange(0, img.rows).colRange(0, img.cols));
    im_text.rowRange(img.rows, im_text.rows) = cv::Mat::zeros(text_size.height + 10, img.cols, img.type());
    cv::putText(im_text, s.str(), cv::Point(5, im_text.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 1, 8);

    // Publish
    cv_bridge::CvImage ros_image;
    ros_image.image = im_text.clone();
    ros_image.header.stamp = ros::Time::now();
    ros_image.encoding = "bgr8";
    pub_tracker_matching_.publish(ros_image.toImageMsg());
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
        cv::circle(img, kp[clusters[i][j]].pt, 2, color, -1);
    }

    // Draw text
    stringstream s;
    int baseline = 0;
    s << " Number of clusters: " << clusters.size();
    cv::Size text_size = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
    cv::Mat im_text = cv::Mat(img.rows + text_size.height + 10, img.cols, img.type());
    img.copyTo(im_text.rowRange(0, img.rows).colRange(0, img.cols));
    im_text.rowRange(img.rows, im_text.rows) = cv::Mat::zeros(text_size.height + 10, img.cols, img.type());
    cv::putText(im_text, s.str(), cv::Point(5, im_text.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 1, 8);

    // Publish
    cv_bridge::CvImage ros_image;
    ros_image.image = im_text.clone();
    ros_image.header.stamp = ros::Time::now();
    ros_image.encoding = "bgr8";
    pub_clustering_.publish(ros_image.toImageMsg());
  }

} //namespace slam