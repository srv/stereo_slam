#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include "stereo_slam/GraphPoses.h"

using namespace std;
using namespace boost;
namespace fs  = filesystem;

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloud;
typedef pcl::IterativeClosestPoint<PointRGB, PointRGB> IterativeClosestPoint;

class Icp
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Pointcloud subscriber
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber graph_poses_sub_;

  // Stores pointclouds
  vector< pair<int, PointCloud::Ptr> > pointclouds_list_;
  mutex mutex_pc_;

  public:
  Icp() : nh_private_("~")
  {
    // Subscribe to pointclouds
    point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud", 1, &Icp::pointCloudCb, this);

    // Subscribe to graph poses
    graph_poses_sub_ = nh_.subscribe<stereo_slam::GraphPoses>("graph_poses", 1, &Icp::graphPosesCb, this);
  }

  private:

  void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    PointCloud::Ptr cloud(new PointCloud);
    fromROSMsg(*cloud_msg, *cloud);

    // Extract the frame id (frame refers to image, not coordinate system)
    int frame_id = lexical_cast<int>(cloud_msg->header.frame_id);

    mutex::scoped_lock(mutex_pc_);
    pointclouds_list_.push_back(make_pair(frame_id, cloud));
  }

  void graphPosesCb(const stereo_slam::GraphPosesConstPtr& poses_msg)
  {
    // Parse message
    vector< pair<int, tf::Transform> > graph_poses;
    vector<int> id = poses_msg->id;
    for(uint i=0; i<id.size(); i++)
    {
      tf::Vector3 t(poses_msg->x[i], poses_msg->y[i], poses_msg->z[i]);
      tf::Quaternion q(poses_msg->qx[i], poses_msg->qy[i], poses_msg->qz[i], poses_msg->qw[i]);
      tf::Transform pose(q, t);
      graph_poses.push_back(make_pair(id[i], pose));
    }

    // Align pointclouds
    int frame_id_0 = -1;
    int frame_id_1 = -1;
    PointCloud::Ptr cloud_0(new PointCloud);
    PointCloud::Ptr cloud_1(new PointCloud);
    {
      mutex::scoped_lock(mutex_pc_);
      if (pointclouds_list_.size() > 1)
      {
        frame_id_0 = pointclouds_list_[0].first;
        frame_id_1 = pointclouds_list_[1].first;
        pcl::copyPointCloud(*pointclouds_list_[0].second, *cloud_0);
        pcl::copyPointCloud(*pointclouds_list_[1].second, *cloud_1);
      }
      else
        return;
    }

    // Get the transformation between clouds
    tf::Transform tf_0, tf_1;
    bool found_0 = getCloudPose(graph_poses, frame_id_0, tf_0);
    bool found_1 = getCloudPose(graph_poses, frame_id_1, tf_1);

    if (found_0 && found_1)
    {
      // Initial alignment
      tf::Transform tf_01 = tf_0.inverse() * tf_1;
      Eigen::Affine3d tf_01_eigen;
      transformTFToEigen(tf_01, tf_01_eigen);
      pcl::transformPointCloud(*cloud_1, *cloud_1, tf_01_eigen);

      // ICP Align
      bool converged;
      double score;
      tf::Transform correction;
      pairAlign(cloud_1, cloud_0, correction, converged, score);
    }
  }

  void pairAlign(PointCloud::Ptr src,
                 PointCloud::Ptr tgt,
                 tf::Transform &output,
                 bool &converged,
                 double &score)
  {
    PointCloud::Ptr aligned (new PointCloud);
    IterativeClosestPoint icp;
    icp.setMaxCorrespondenceDistance(0.07);
    icp.setRANSACOutlierRejectionThreshold(0.005);
    icp.setTransformationEpsilon(0.000001);
    icp.setEuclideanFitnessEpsilon(0.0001);
    icp.setMaximumIterations(100);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.align(*aligned);

    // The transform
    output = matrix4fToTf(icp.getFinalTransformation());

    // The measures
    converged = icp.hasConverged();
    score = icp.getFitnessScore();
  }

  tf::Transform matrix4fToTf(Eigen::Matrix4f in)
  {
    tf::Vector3 t_out;
    t_out.setValue(static_cast<double>(in(0,3)),static_cast<double>(in(1,3)),static_cast<double>(in(2,3)));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(in(0,0)), static_cast<double>(in(0,1)), static_cast<double>(in(0,2)),
                  static_cast<double>(in(1,0)), static_cast<double>(in(1,1)), static_cast<double>(in(1,2)),
                  static_cast<double>(in(2,0)), static_cast<double>(in(2,1)), static_cast<double>(in(2,2)));

    tf::Quaternion q_out;
    tf3d.getRotation(q_out);
    tf::Transform out(q_out, t_out);
    return out;
  }

  bool getCloudPose(vector< pair<int, tf::Transform> > graph_poses, int id, tf::Transform &out)
  {
    out.setIdentity();
    bool found = false;

    for (uint i=0; i<graph_poses.size(); i++)
    {
      if (graph_poses[i].first == id)
      {
        out = graph_poses[i].second;
        found = true;
        break;
      }
    }
    return found;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp");
  Icp node;

  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();

  return 0;
}






