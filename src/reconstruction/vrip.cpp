#include "reconstruction/vrip.h"
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/common/common.h>

using namespace boost;
namespace fs=boost::filesystem;

namespace reconstruction
{
  Vrip::Vrip() {};

  tf::Transform Vrip::readCloudPose(int cloud_id)
  {
    string poses_file = "/home/plnegre/workspace/ros/turbot/src/stereo_slam/data/reconstruction/graph_vertices.txt";

    string line;
    bool found = false;
    ifstream file(poses_file.c_str());
    while (getline(file, line))
    {
      int i = 0;
      string value;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double qx = 0.0;
      double qy = 0.0;
      double qz = 0.0;
      double qw = 1.0;
      istringstream ss(line);
      while(getline(ss, value, ','))
      {
        if (i == 1)
        {
          if (value == lexical_cast<string>(cloud_id))
            found = true;
        }
        else
        {
          if (found)
          {
            if (i == 2)
              x = boost::lexical_cast<double>(value);
            else if (i == 3)
              y = boost::lexical_cast<double>(value);
            else if (i == 4)
              z = boost::lexical_cast<double>(value);
            else if (i == 5)
              qx = boost::lexical_cast<double>(value);
            else if (i == 6)
              qy = boost::lexical_cast<double>(value);
            else if (i == 7)
              qz = boost::lexical_cast<double>(value);
            else if (i == 8)
              qw = boost::lexical_cast<double>(value);
            break;
          }
        }
        i++;
      }

      if (found)
      {
        // Build the tf
        tf::Vector3 t(x, y, z);
        tf::Quaternion q(qx, qy, qz, qw);
        tf::Transform transf(q, t);
        return transf;
      }
    }

    // If not found, return empty transform
    tf::Transform empty;
    empty.setIdentity();
    return empty;
  }

  void Vrip::merge()
  {
    // Read the pointclouds directory
    string clouds_path = "/home/plnegre/workspace/ros/turbot/src/stereo_slam/data/reconstruction/pointclouds/";
    typedef vector<fs::path> vec;
    vec v;
    copy(
        fs::directory_iterator(clouds_path),
        fs::directory_iterator(),
        back_inserter(v)
        );

    sort(v.begin(), v.end());
    vec::const_iterator it(v.begin());

    while (it!=v.end())
    {
      // Check if the directory entry is an directory.
      if (fs::is_directory(*it))
      {
        it++;
        continue;
      }

      // Check the extension
      string extension = it->filename().extension().string();
      if (extension != ".pcd")
      {
        it++;
        continue;
      }

      // Cloud id
      int cloud_id = lexical_cast<int>(it->stem().string());

      // Read the current pointcloud
      string cloud_filename = clouds_path + it->filename().string();
      PointCloudRGB::Ptr in_cloud(new PointCloudRGB);
      if (pcl::io::loadPCDFile<PointRGB> (cloud_filename, *in_cloud) == -1)
      {
        ROS_WARN_STREAM("[Reconstruction:] Couldn't read the file: " << cloud_filename);
        continue;
      }

      // Read the pose of this pointcloud
      tf::Transform cloud_pose = readCloudPose(cloud_id);

      ROS_INFO_STREAM("KK: " << cloud_pose.getOrigin().x());

      it++;
    }
  }
}
