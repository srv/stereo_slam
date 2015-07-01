#include "cluster.h"

namespace slam
{
  Cluster::Cluster() : id_(-1){}

  Cluster::Cluster(int id, int frame_id, tf::Transform pose, vector<cv::KeyPoint> kp, cv::Mat orb_desc, cv::Mat sift_desc, vector<cv::Point3f> points) :
                  id_(id), frame_id_(frame_id), pose_(pose), kp_(kp), orb_desc_(orb_desc), sift_desc_(sift_desc), points_(points){}
}