#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <mysql/mysql.h>
#include <math.h>

#include "stereo_slam/GraphPoses.h"

using namespace std;
using namespace boost;
namespace fs  = filesystem;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class SaveClouds
{
  struct PointCloudMinMax
  {
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;

    PointCloudMinMax(pcl::PointXYZRGB min, pcl::PointXYZRGB max)
    {
      min_pt = min;
      max_pt = max;
    }
  };

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Pointcloud subscriber
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber graph_poses_sub_;

  // Node parameters
  bool save_pcd_;
  bool save_db_;
  string db_host_, db_user_, db_pass_, db_name_;

  // Global variables
  string clouds_dir_;
  vector< pair<int, PointCloud::Ptr> > pointclouds_list_;
  vector< pair<int, PointCloudMinMax> > pointclouds_minmax_;
  mutex mutex_pc_;

  public:
  SaveClouds() : nh_private_("~")
  {
    nh_private_.param("save_pcd", save_pcd_, false);
    nh_private_.param("save_db", save_db_, false);
    nh_private_.param("db_host", db_host_, string("localhost"));
    nh_private_.param("db_user", db_user_, string("root"));
    nh_private_.param("db_pass", db_pass_, string("root"));
    nh_private_.param("db_name", db_name_, string("slam"));

    // Subscribe to pointclouds
    point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud", 5, &SaveClouds::pointCloudCb, this);

    // Subscribe to graph poses
    graph_poses_sub_ = nh_.subscribe<stereo_slam::GraphPoses>("graph_poses", 1, &SaveClouds::graphPosesCb, this);

    // Create the pointcloud directory
    clouds_dir_ = ros::package::getPath("stereo_slam") + "/pointclouds/";
    if (fs::is_directory(clouds_dir_))
      fs::remove_all(clouds_dir_ );
    if (save_pcd_)
    {
      fs::path dir1(clouds_dir_ );
      if (!fs::create_directory(dir1))
        ROS_ERROR("[SaveClouds:] ERROR -> Impossible to create the pointclouds directory.");
    }

    if (save_db_)
    {
      // Database connection
      MYSQL *connect; // Create a pointer to the MySQL instance
      connect = mysql_init(NULL); // Initialise the instance
      connect = mysql_real_connect(connect,
          db_host_.c_str(),
          db_user_.c_str(),
          db_pass_.c_str(),
          NULL, 0, NULL, 0);
      if(connect)
      {
        ROS_INFO("[SaveClouds:] DB connection succeeded.");

        // Create database
        string query = "DROP DATABASE " + db_name_ + ";";
        mysql_query(connect, query.c_str());
        query = "CREATE DATABASE " + db_name_ + ";";
        mysql_query(connect, query.c_str());

        // Create tables
        connect = mysql_init(NULL);
        connect = mysql_real_connect(connect,
            db_host_.c_str(),
            db_user_.c_str(),
            db_pass_.c_str(),
            db_name_.c_str(),
            0, NULL, 0);
        query = "CREATE TABLE IF NOT EXISTS `poses` ("
          "`pose_id` int(11) NOT NULL,"
          "`x` float NOT NULL,"
          "`y` float NOT NULL,"
          "`z` float NOT NULL,"
          "`qx` float NOT NULL,"
          "`qy` float NOT NULL,"
          "`qz` float NOT NULL,"
          "`qw` float NOT NULL,"
          "KEY (`pose_id`)"
          ") ENGINE=InnoDB DEFAULT CHARSET=utf8;";
        mysql_query(connect, query.c_str());
        query = "CREATE TABLE IF NOT EXISTS `points` ("
          "`pose_id` int(11) NOT NULL,"
          "`x` float NOT NULL,"
          "`y` float NOT NULL,"
          "`z` float NOT NULL,"
          "`r` int(3) NOT NULL,"
          "`g` int(3) NOT NULL,"
          "`b` int(3) NOT NULL"
          ") ENGINE=InnoDB DEFAULT CHARSET=utf8;";
        mysql_query(connect, query.c_str());
        mysql_close(connect);
        ROS_INFO("[SaveClouds:] DB tables created successfully.");
      }
      else
      {
        ROS_INFO("[SaveClouds:] DB connection Failed!");
      }
    }
  }

  private:

  void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    PointCloud::Ptr cloud(new PointCloud);
    fromROSMsg(*cloud_msg, *cloud);

    // Extract the frame id (frame referes to image, not coordinate system)
    int frame_id = lexical_cast<int>(cloud_msg->header.frame_id);

    // Save cloud to pcd
    if (save_pcd_)
    {
      string pc_filename = clouds_dir_ + lexical_cast<string>(frame_id) + ".pcd";
      pcl::io::savePCDFileBinary(pc_filename, *cloud);
    }

    if (save_db_)
    {
      // Save cloud into the history
      mutex::scoped_lock lock(mutex_pc_);
      pointclouds_list_.push_back(make_pair(frame_id, cloud)); 

      // Save cloud min and max
      pcl::PointXYZRGB min, max;
      pcl::getMinMax3D(*cloud, min, max);
      PointCloudMinMax minmax(min, max);
      pointclouds_minmax_.push_back(make_pair(frame_id, minmax)); 
    }
  }

  void insertPointCloud(PointCloud::Ptr cloud, string frame_id)
  {
    // Insert into database
    // You should increase the 'max_allowed_packet' up to 50M to this query take effect.
    // 'max_allowed_packet' can be changed in '/etc/mysql/my.cnf'.
    string q = "INSERT INTO points (pose_id, x, y, z, r, g, b) VALUES ";
    for (uint n=0; n<cloud->points.size(); n++)
    {
      string x = lexical_cast<string>(roundf(cloud->points[n].x * 1000) / 1000 );
      string y = lexical_cast<string>(roundf(cloud->points[n].y * 1000) / 1000 );
      string z = lexical_cast<string>(roundf(cloud->points[n].z * 1000) / 1000 );

      int rgb = *reinterpret_cast<const int*>(&(cloud->points[n].rgb));
      uint8_t uint_r = (rgb >> 16) & 0x0000ff;
      uint8_t uint_g = (rgb >> 8)  & 0x0000ff;
      uint8_t uint_b = (rgb)       & 0x0000ff; 
      string r = lexical_cast<string>((int)uint_r);
      string g = lexical_cast<string>((int)uint_g);
      string b = lexical_cast<string>((int)uint_b);
      q += "('"+frame_id+"','"+x+"','"+y+"','"+z+"', '"+r+"', '"+g+"', '"+b+"'),";
    }
    q = q.substr(0, q.size()-1);
    ros::WallTime start = ros::WallTime::now();
    query(q);
    ros::WallDuration d = ros::WallTime::now() - start;
    ROS_INFO_STREAM("INSERTING: " << frame_id);
    ROS_INFO_STREAM("TIME: " << d.toSec());
  }

  void insertGraphPoses(vector< pair<int, tf::Transform> > graph_poses)
  {
    for (uint i=0; i<graph_poses.size(); i++)
    {
      // Extract pose info
      string pose_id = lexical_cast<string>(graph_poses[i].first);
      string x = lexical_cast<string>(graph_poses[i].second.getOrigin().x());
      string y = lexical_cast<string>(graph_poses[i].second.getOrigin().y());
      string z = lexical_cast<string>(graph_poses[i].second.getOrigin().z());
      string qx = lexical_cast<string>(graph_poses[i].second.getRotation().x());
      string qy = lexical_cast<string>(graph_poses[i].second.getRotation().y());
      string qz = lexical_cast<string>(graph_poses[i].second.getRotation().z());
      string qw = lexical_cast<string>(graph_poses[i].second.getRotation().w());

      string q = "SELECT pose_id FROM poses WHERE pose_id='" + pose_id + "'";
      MYSQL_RES *res = query(q);
      if (mysql_num_rows(res)>0)
      {
        // Update
        q = "UPDATE poses SET x='"+x+"',y='"+y+"',z='"+z+"',qx='"+qx+"',qy='"+qy+"',qz='"+qz+"',qw='"+qw+"' WHERE pose_id='"+pose_id+"'";
        query(q);
      }
      else
      {
        // Insert
        q = "INSERT INTO poses (pose_id, x, y, z, qx, qy, qz, qw) VALUES ";
        q += "('"+pose_id+"','"+x+"','"+y+"','"+z+"', '"+qx+"', '"+qy+"', '"+qz+"', '"+qw+"')";
        query(q);
      }
    }
  }

  void graphPosesCb(const stereo_slam::GraphPosesConstPtr& poses_msg)
  {
    if (!save_db_) return;

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

    // Insert/update graph poses
    insertGraphPoses(graph_poses);

    // Insert/update pointclouds
    vector<int> erase_ids;
    uint pointclouds_size;
    vector< pair<int, PointCloudMinMax> > pointclouds_minmax;
    {
      mutex::scoped_lock(mutex_pc_);
      pointclouds_size = pointclouds_list_.size();
      pointclouds_minmax = pointclouds_minmax_;
    }
    for (uint i=0; i<pointclouds_size; i++)
    {
      // Extract info
      int frame_id;
      int first_frame_id;
      PointCloud::Ptr cloud;
      {
        mutex::scoped_lock(mutex_pc_);
        frame_id = pointclouds_list_[i].first;
        cloud = pointclouds_list_[i].second; 
      } 

      // First cloud will be inserted directly
      first_frame_id = pointclouds_minmax[0].first;
      if (frame_id == first_frame_id)
      {
        insertPointCloud(cloud, lexical_cast<string>(frame_id));
        erase_ids.push_back(frame_id);
      } 
      else
      {
        // Extract the min/max and the pose of the previous pointcloud
        int prev_frame_id = frame_id - 1;
        int idx_minmax = -1;
        for (uint j=0; j<pointclouds_minmax.size(); j++)
        {
          if (pointclouds_minmax[j].first == prev_frame_id)
          {
            idx_minmax = j;
            break;
          }
        }

        if (idx_minmax >= 0)
        {
          // Extract the overlapping region with the previous pointcloud
          pcl::PointXYZRGB min_pt = pointclouds_minmax[idx_minmax].second.min_pt;
          pcl::PointXYZRGB max_pt = pointclouds_minmax[idx_minmax].second.max_pt;
          Eigen::Vector4f emin_pt(min_pt.x, min_pt.y, min_pt.z, 1.0);
          Eigen::Vector4f emax_pt(max_pt.x, max_pt.y, max_pt.z, 1.0);
          Eigen::Affine3f trans(transformA2B(graph_poses, frame_id, prev_frame_id));

          // Remove the overlapping region
          int init_size = cloud->points.size();
          pcl::CropBox<pcl::PointXYZRGB> crop_filter;
          crop_filter.setInputCloud(cloud);
          crop_filter.setMin(emin_pt);
          crop_filter.setMax(emax_pt);
          crop_filter.setTransform(trans);
          crop_filter.filter(*cloud);
          int final_size = cloud->points.size();
          ROS_INFO_STREAM("SIZE REDUCTION: " << init_size << " -> " << final_size);

          // Insert the reduced cloud into the database
          insertPointCloud(cloud, lexical_cast<string>(frame_id));
          erase_ids.push_back(frame_id);
        }
      }
    }

    // Delete inserted pointclouds from the list
    {
      mutex::scoped_lock(mutex_pc_);
      for (uint i=0; i<erase_ids.size(); i++)
      {
        int id = erase_ids[i];
        int idx = -1;
        for (uint j=0; j<pointclouds_list_.size(); j++)
        {
          if (pointclouds_list_[j].first == id)
          {
            idx = j;
            break;
          }
        }
        if (idx >= 0)
          pointclouds_list_.erase(pointclouds_list_.begin() + idx);
      }
    }
  }

  Eigen::Affine3d transformA2B(vector< pair<int, tf::Transform> > graph_poses, int a, int b)
  {
    tf::Transform tf_a;
    tf::Transform tf_b;
    tf_a.setIdentity();
    tf_b.setIdentity();
    bool found_a = false;
    bool found_b = false;

    for (uint i=0; i<graph_poses.size(); i++)
    {
      if (graph_poses[i].first == a)
      {
        tf_a = graph_poses[i].second;
        found_a = true;
      }
      if (graph_poses[i].first == b)
      {
        tf_b = graph_poses[i].second;
        found_b = true;
      }
      if (found_a & found_b) break;
    }

    Eigen::Affine3d out;
    tf::Transform tf_c = tf_a.inverse()*tf_b;
    tf::poseTFToEigen(tf_c, out);
    return out;
  }

  MYSQL_RES *query(string q)
  {
    MYSQL_RES *res;
    MYSQL *connect;
    connect = mysql_init(NULL);
    connect = mysql_real_connect(connect,
        db_host_.c_str(),
        db_user_.c_str(),
        db_pass_.c_str(),
        db_name_.c_str(),
        0, NULL, 0);
    if(connect)
    {
      mysql_query(connect, q.c_str());
      res = mysql_store_result(connect);
      mysql_close(connect);
      return res;
    }
    else
    {
      exit(1);
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_clouds");
  SaveClouds node;

  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();

  return 0;
}






