#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <mysql/mysql.h>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/filter.h>
#include "common/tools.h"

#define EVENT_SIZE  ( sizeof (struct inotify_event) )
#define BUF_LEN     ( 1024 * ( EVENT_SIZE + 16 ) )

using namespace std;
using namespace tools;
using namespace boost;

namespace fs=boost::filesystem;

typedef pcl::PointXYZRGB                  PointRGB;
typedef pcl::PointCloud<PointRGB>         PointCloudRGB;

class InserterNode
{
private:

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Database
  string db_host_, db_user_, db_pass_, db_name_;
  MYSQL *connect_;
  int mod_id_;

  // Cloud parameters
  vector<string> processed_clouds_;
  vector<int> cloud_sizes_;

  // Lock
  bool lock_;

public:

  string work_dir_;

  /**
   * Class constructor
   */
  InserterNode() : nh_private_("~")
  {
    // Read parameters
    nh_private_.param("work_dir",   work_dir_,    string(""));
    nh_private_.param("db_host",    db_host_,     string(""));
    nh_private_.param("db_user",    db_user_,     string(""));
    nh_private_.param("db_pass",    db_pass_,     string(""));
    db_name_ = "reconstruction";

    // Init workspace
    if (work_dir_[work_dir_.length()-1] != '/')
      work_dir_ += "/";

    // Init the locks
    lock_ = false;

    // Init the poses modification id
    mod_id_ = 0;

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
      ROS_INFO("[Inserter:] DB connection succeeded.");

      // Create database
      //string query = "DROP DATABASE " + db_name_ + ";";
      //mysql_query(connect, query.c_str());
      string query = "CREATE DATABASE IF NOT EXISTS " + db_name_ + ";";
      mysql_query(connect, query.c_str());

      query = "SET bulk_insert_buffer_size =1024*1024*128;";

      // Create tables
      connect = mysql_init(NULL);
      connect = mysql_real_connect(connect,
                                   db_host_.c_str(),
                                   db_user_.c_str(),
                                   db_pass_.c_str(),
                                   db_name_.c_str(),
                                   0, NULL, 0);
      query = "DROP TABLE IF EXISTS `poses`;";
      mysql_query(connect, query.c_str());
      query = "CREATE TABLE IF NOT EXISTS `poses` ("
              "`id` int(11) NOT NULL AUTO_INCREMENT,"
              "`node_id` int(11) NOT NULL,"
              "`mod_id` int(11) NOT NULL,"
              "`x` float NOT NULL,"
              "`y` float NOT NULL,"
              "`z` float NOT NULL,"
              "`qx` float NOT NULL,"
              "`qy` float NOT NULL,"
              "`qz` float NOT NULL,"
              "`qw` float NOT NULL,"
              "PRIMARY KEY (`id`)"
              ") ENGINE=InnoDB DEFAULT CHARSET=utf8 AUTO_INCREMENT=1;";
      mysql_query(connect, query.c_str());
      query = "DROP TABLE IF EXISTS `clouds`;";
      mysql_query(connect, query.c_str());
      query = "CREATE TABLE IF NOT EXISTS `clouds` ("
              "`id` int(11) NOT NULL AUTO_INCREMENT,"
              "`node_id` int(11) NOT NULL,"
              "`x` float NOT NULL,"
              "`y` float NOT NULL,"
              "`z` float NOT NULL,"
              "`rgb` float NOT NULL,"
              "PRIMARY KEY (`id`)"
              ") ENGINE=InnoDB DEFAULT CHARSET=utf8 AUTO_INCREMENT=1;";
      mysql_query(connect, query.c_str());
      mysql_close(connect);

      // Init the connection
      connect_ = mysql_init(NULL);
      connect_ = mysql_real_connect(connect_,
                                   db_host_.c_str(),
                                   db_user_.c_str(),
                                   db_pass_.c_str(),
                                   db_name_.c_str(),
                                   0, NULL, 0);

      // TODO: Move database to RAM to speedup the process
      // http://tomislavsantek.iz.hr/2011/03/moving-mysql-databases-to-ramdisk-in-ubuntu-linux/

      ROS_INFO("[Inserter:] DB tables created successfully.");
    }
    else
    {
      ROS_INFO("[Inserter:] DB connection Failed!");
    }
  }

  /**
   * Launch a mysql query and return the output
   */
  MYSQL_RES *query(string q, bool get_res)
  {
    MYSQL_RES *res;

    if(connect_)
    {
      mysql_query(connect_, q.c_str());

      if (get_res)
        res = mysql_store_result(connect_);
    }

    return res;
  }

  /**
   * Insert callback
   */
  void insert()
  {
    // Lock insert
    if (lock_) return;
    lock_ = true;

    // Update the clouds table
    string node_id = lexical_cast<string>(processed_clouds_.size());
    string filename = work_dir_ + "clouds/" + node_id + ".pcd";
    if (insertCloud(node_id))
    {
      processed_clouds_.push_back(node_id);

      // Only update the poses when a new cloud is inserted
      insertPoses();
    }

    // Unlock
    lock_ = false;
  }

  /**
   * Insert/update the cloud poses
   */
  void insertPoses()
  {
    // Read the poses file
    vector< pair<string, tf::Transform> > cloud_poses;
    Tools::readPoses(work_dir_, cloud_poses);

    // Stores the number of points to be updated in this iteration
    int total_updated_points = 0;

    // Maximum processed cloud
    int max = cloud_poses.size();
    if (processed_clouds_.size() < max)
      max = processed_clouds_.size();

    // Init the insert query
    string q_insert = "";

    // Init the update query
    vector<string> id_val, x_val, y_val, z_val, qx_val, qy_val, qz_val, qw_val, mod_id_val;

    // Loop poses
    for (uint i=0; i<max; i++)
    {
      string node_id = cloud_poses[i].first;
      tf::Transform pose = cloud_poses[i].second;
      string mod_id = lexical_cast<string>(mod_id_);

      // Extract the fields
      tf::Vector3 trans = pose.getOrigin();
      tf::Quaternion rot = pose.getRotation();
      string x  = lexical_cast<string>(r3d(trans.x()));
      string y  = lexical_cast<string>(r3d(trans.y()));
      string z  = lexical_cast<string>(r3d(trans.z()));
      string qx = lexical_cast<string>(r3d(rot.x()));
      string qy = lexical_cast<string>(r3d(rot.y()));
      string qz = lexical_cast<string>(r3d(rot.z()));
      string qw = lexical_cast<string>(r3d(rot.w()));

      // Check if this pose exist into the database
      MYSQL_RES *res = query("SELECT id, x, y, z FROM poses WHERE node_id='" + node_id + "'", true);
      bool exists = false;
      if (res)
      {
        if (mysql_num_rows(res) > 0)
        {
          exists = true;
        }
      }
      if (exists)
      {
        // Check if pose has changed
        MYSQL_ROW row = mysql_fetch_row(res);
        float new_x = lexical_cast<float>(x);
        float new_y = lexical_cast<float>(y);
        float new_z = lexical_cast<float>(z);
        float old_x = lexical_cast<float>(row[1]);
        float old_y = lexical_cast<float>(row[2]);
        float old_z = lexical_cast<float>(row[3]);
        float diff = poseDiff(new_x, new_y, new_z, old_x, old_y, old_z);
        if (diff >= 0.01) // 1cm
        {
          // Store for pose update in a single query
          id_val.push_back(lexical_cast<string>(row[0]));
          x_val.push_back(x);
          y_val.push_back(y);
          z_val.push_back(z);
          qx_val.push_back(qx);
          qy_val.push_back(qy);
          qz_val.push_back(qz);
          qw_val.push_back(qw);
          mod_id_val.push_back(mod_id);

          total_updated_points += cloud_sizes_[lexical_cast<int>(node_id)];
        }
      }
      else
      {
        if (q_insert.size() == 0)
        {
          q_insert = "INSERT INTO poses (node_id, mod_id, x, y, z, qx, qy, qz, qw) VALUES ";
        }
        // Insert the pose
        q_insert += "('"+node_id+"','"+mod_id+"','"+x+"','"+y+"','"+z+"', '"+qx+"', '"+qy+"', '"+qz+"', '"+qw+"'),";
      }

      // Free the results from the database query
      mysql_free_result(res);
    }

    // Insert query
    if (q_insert.size() > 0)
    {
      q_insert = q_insert.substr(0, q_insert.size()-1) + ";";
      query(q_insert, false);
    }

    // Update query
    if (id_val.size() > 0)
    {
      string q_id = "";
      string q_x = "x = CASE id ";
      string q_y = "y = CASE id ";
      string q_z = "z = CASE id ";
      string q_qx = "qx = CASE id ";
      string q_qy = "qy = CASE id ";
      string q_qz = "qz = CASE id ";
      string q_qw = "qw = CASE id ";
      string q_mod_id = "mod_id = CASE id ";
      for (uint i=0; i<id_val.size(); i++)
      {
        q_id      += id_val[i]+",";
        q_x       += "WHEN "+id_val[i]+" THEN "+x_val[i]+" ";
        q_y       += "WHEN "+id_val[i]+" THEN "+y_val[i]+" ";
        q_z       += "WHEN "+id_val[i]+" THEN "+z_val[i]+" ";
        q_qx      += "WHEN "+id_val[i]+" THEN "+qx_val[i]+" ";
        q_qy      += "WHEN "+id_val[i]+" THEN "+qy_val[i]+" ";
        q_qz      += "WHEN "+id_val[i]+" THEN "+qz_val[i]+" ";
        q_qw      += "WHEN "+id_val[i]+" THEN "+qw_val[i]+" ";
        q_mod_id  += "WHEN "+id_val[i]+" THEN "+mod_id_val[i]+" ";
      }

      q_id = q_id.substr(0, q_id.size()-1);
      string q_update = "UPDATE poses SET "+q_x+"END, "+q_y+"END, "+q_z+"END, "+q_qx+"END, "+q_qy+"END, "+q_qz+"END, "+q_qw+"END, "+q_mod_id+"END WHERE id IN ("+q_id+")";
      query(q_update, false);
    }
    
    // Sum of all points
    int total_points = 0;
    for (uint i=0; i<max; i++)
    {
      total_points += cloud_sizes_[i];
    }

    ROS_INFO_STREAM("[Inserter:] Total processed clouds: " << processed_clouds_.size() << " (Points: " << total_points << " / " << total_updated_points << ")");

    // Increase the poses modification id
    mod_id_++;
  }

  /**
   * Insert a pointcloud into the database
   */
  bool insertCloud(string node_id)
  {
    // The cloud path
    string filename = work_dir_ + "clouds/" + node_id + ".pcd";

    // Check if it exists or not
    if(!fs::exists(filename))
      return false;

    // Read the cloud
    PointCloudRGB::Ptr in_cloud(new PointCloudRGB);
    if (pcl::io::loadPCDFile<PointRGB> (filename, *in_cloud) == -1)
    {
      ROS_WARN_STREAM("[Inserter:] Couldn't read the file: " << filename);
      return false;
    }

    // Filter cloud
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    cloud = filterCloud(in_cloud);

    // Insert into database
    // You should increase the 'max_allowed_packet' up to 500M to this query take effect.
    // 'max_allowed_packet' can be changed in '/etc/mysql/my.cnf'.
    string q = "INSERT INTO clouds (node_id, x, y, z, rgb) VALUES ";
    for (uint n=0; n<cloud->points.size(); n++)
    {
      string x = lexical_cast<string>(r3d(cloud->points[n].x));
      string y = lexical_cast<string>(r3d(cloud->points[n].y));
      string z = lexical_cast<string>(r3d(cloud->points[n].z));
      string rgb = lexical_cast<string>(cloud->points[n].rgb);
      q += "('"+node_id+"','"+x+"','"+y+"','"+z+"', '"+rgb+"'),";
    }
    q = q.substr(0, q.size()-1) + ";";

    // Launch the query in a separate thread to continue reading pointclouds
    //boost::thread query_thread(&InserterNode::query, this, q, false);
    query(q, false);

    // Save the size of the cloud
    cloud_sizes_.push_back(cloud->points.size());

    return true;
  }


  /**
   * Round float number to 3 decimals
   */
  float r3d(float f)
  {
    return roundf(f * 1000) / 1000;
  }

  /**
   * Filter a pointcloud
   */
  PointCloudRGB::Ptr filterCloud(PointCloudRGB::Ptr cloud)
  {
    // Remove isolated points
    //pcl::RadiusOutlierRemoval<PointRGB> outrem;
    //outrem.setInputCloud(cloud);
    //outrem.setRadiusSearch(0.04);
    //outrem.setMinNeighborsInRadius(50);
    //outrem.filter(*cloud);
    pcl::StatisticalOutlierRemoval<PointRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(40);
    sor.setStddevMulThresh(2.0);
    sor.filter(*cloud);

    return cloud;
  }

  /**
   * Compute the difference between two poses
   */
  float poseDiff(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2)
  {
    return sqrt( (x_1-x_2)*(x_1-x_2) + (y_1-y_2)*(y_1-y_2) + (z_1-z_2)*(z_1-z_2) );
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "inserter");
  InserterNode node;

  ros::Rate r(20); // Hz
  while(1)
  {
    node.insert();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

