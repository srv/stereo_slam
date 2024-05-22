#include "cluster.hpp"
#include "tools.hpp"

namespace slam
{
  Cluster::Cluster() : id_(-1){}

  Cluster::Cluster(int id, int frame_id, tf::Transform camera_pose, std::vector<cv::KeyPoint> kp_l, std::vector<cv::KeyPoint> kp_r, cv::Mat orb_desc, cv::Mat sift_desc, std::vector<cv::Point3f> points) :
                  id_(id), frame_id_(frame_id), camera_pose_(camera_pose), kp_l_(kp_l), kp_r_(kp_r), orb_desc_(orb_desc), sift_desc_(sift_desc), points_(points){}

  std::vector<cv::Point3f> Cluster::getWorldPoints()
  {
    std::vector<cv::Point3f> out;
    for (uint i=0; i<points_.size(); i++)
    {
      cv::Point3f p = tools::Tools::transformPoint(points_[i], camera_pose_);
      out.push_back(p);
    }

    return out;
  }
}