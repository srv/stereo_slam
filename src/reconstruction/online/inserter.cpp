#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <mysql/mysql.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <errno.h>
#include <boost/filesystem.hpp>
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
  int mod_id_;

  // Cloud parameters
  double voxel_size_;
  vector<string> processed_clouds_;

public:

  string work_dir_;

  /**
   * Class constructor
   */
  InserterNode() : nh_private_("~")
  {
    // Read parameters
    nh_private_.param("work_dir",   work_dir_,    string(""));
    nh_private_.param("voxel_size", voxel_size_,  0.005);
    nh_private_.param("db_host",    db_host_,     string("localhost"));
    nh_private_.param("db_user",    db_user_,     string("root"));
    nh_private_.param("db_pass",    db_pass_,     string("root"));
    db_name_ = "reconstruction";

    // Init workspace
    if (work_dir_[work_dir_.length()-1] != '/')
      work_dir_ += "/";

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

      // Move database to RAM to speedup the process
      // http://tomislavsantek.iz.hr/2011/03/moving-mysql-databases-to-ramdisk-in-ubuntu-linux/
      // TODO

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
    }
    return res;
  }

  /**
   * Graph vertices file change callback
   */
  void fileChangeCb(int fd)
  {
    int i = 0;
    char buffer[BUF_LEN];

    // This blocks the execution until the file is changed
    int length = read(fd, buffer, BUF_LEN);

    // Update the poses table
    vector< pair<string, tf::Transform> > cloud_poses;
    Tools::readPoses(work_dir_, cloud_poses);
    updatePoses(cloud_poses);

    // Update the clouds table
    string node_id = lexical_cast<string>(processed_clouds_.size());
    string filename = work_dir_ + "clouds/" + node_id + ".pcd";
    if(fs::exists(filename))
    {
      ROS_INFO_STREAM("[Inserter:] Inserting pointcloud " << node_id << " into database...");
      if (insertCloud(node_id))
        processed_clouds_.push_back(node_id);
    }
  }

  /**
   * Update the cloud poses
   */
  void updatePoses(vector< pair<string, tf::Transform> > cloud_poses)
  {
    for (uint i=0; i<cloud_poses.size(); i++)
    {
      string node_id = cloud_poses[i].first;
      tf::Transform pose = cloud_poses[i].second;
      string mod_id = lexical_cast<string>(mod_id_);

      // Extract the fields
      tf::Vector3 trans = pose.getOrigin();
      tf::Quaternion rot = pose.getRotation();
      string x  = lexical_cast<string>(trans.x());
      string y  = lexical_cast<string>(trans.y());
      string z  = lexical_cast<string>(trans.z());
      string qx = lexical_cast<string>(rot.x());
      string qy = lexical_cast<string>(rot.y());
      string qz = lexical_cast<string>(rot.z());
      string qw = lexical_cast<string>(rot.w());

      // Check if this pose exist into the database
      MYSQL_RES *res = query("SELECT id, x, y, z FROM poses WHERE node_id='" + node_id + "'");
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
          // Update the pose into the database
          string id = lexical_cast<string>(row[0]);
          string q = "UPDATE poses SET x='"+x+"',y='"+y+"',z='"+z+"',qx='"+qx+"',qy='"+qy+"',qz='"+qz+"',qw='"+qw+"',mod_id='"+mod_id+"' WHERE id='"+id+"'";
          query(q);
        }
      }
      else
      {
        // Insert the pose
        string q = "INSERT INTO poses (node_id, mod_id, x, y, z, qx, qy, qz, qw) VALUES ";
        q += "('"+node_id+"','"+mod_id+"','"+x+"','"+y+"','"+z+"', '"+qx+"', '"+qy+"', '"+qz+"', '"+qw+"')";
        query(q);
      }
    }

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

    // Read the cloud
    PointCloudRGB::Ptr in_cloud(new PointCloudRGB);
    if (pcl::io::loadPCDFile<PointRGB> (filename, *in_cloud) == -1)
    {
      ROS_WARN_STREAM("[Inserter:] Couldn't read the file: " << filename);
      return false;
    }

    // Filter cloud
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    cloud = filterCloud(in_cloud, voxel_size_);

    // Insert into database
    // You should increase the 'max_allowed_packet' up to 500M to this query take effect.
    // 'max_allowed_packet' can be changed in '/etc/mysql/my.cnf'.
    string q = "INSERT INTO clouds (node_id, x, y, z, rgb) VALUES ";
    for (uint n=0; n<cloud->points.size(); n++)
    {
      string x = lexical_cast<string>(cloud->points[n].x);
      string y = lexical_cast<string>(cloud->points[n].y);
      string z = lexical_cast<string>(cloud->points[n].z);
      string rgb = lexical_cast<string>(cloud->points[n].rgb);
      q += "('"+node_id+"','"+x+"','"+y+"','"+z+"', '"+rgb+"'),";
    }
    query(q);

    return true;
  }

  /**
   * Filter a pointcloud
   */
  PointCloudRGB::Ptr filterCloud(PointCloudRGB::Ptr in_cloud, double voxel_size)
  {
    // Remove nans
    vector<int> indices;
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indices);
    indices.clear();

    // Voxel grid filter (used as x-y surface extraction. Note that leaf in z is very big)
    pcl::ApproximateVoxelGrid<PointRGB> grid;
    grid.setLeafSize(voxel_size, voxel_size, 0.5);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);

    // Remove isolated points
    pcl::RadiusOutlierRemoval<PointRGB> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.04);
    outrem.setMinNeighborsInRadius(50);
    outrem.filter(*cloud);
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

  // The graph file
  string graph_file = node.work_dir_ + "graph_vertices.txt";

  // Wait until file is available
  while(!fs::exists(graph_file))
    sleep(1);

  // Watch descriptor
  int fd = inotify_init();
  if (fd < 0) {
    ROS_ERROR("[Inserter:] Impossible to create the file monitor.");
    exit(1);
  }
  int wd = inotify_add_watch(fd, graph_file.c_str(), IN_MODIFY | IN_CREATE | IN_DELETE);
  if (wd < 0) {
    ROS_ERROR_STREAM("[Inserter:] Impossible watch the graph vertices file. " << strerror(errno));
    exit(1);
  }

  ros::Rate r(1); // Hz
  while(1)
  {
    node.fileChangeCb(fd);
    ros::spinOnce();
    r.sleep();
  }
  close(fd);
  return 0;
}

