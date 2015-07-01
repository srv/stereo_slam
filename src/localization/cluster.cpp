#include "localization/cluster.h"

namespace slam
{
  Cluster::Cluster() : id_(-1){}

  Cluster::Cluster(int id, vector<cv::KeyPoint> kp, cv::Mat orb_desc, cv::Mat sift_desc, vector<cv::Point3f> points) :
                  id_(id), kp_(kp), orb_desc_(orb_desc), sift_desc_(sift_desc), points_(points){}
}