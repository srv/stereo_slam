#include "registration/base.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stdlib.h>
#include "../polybool/polybool.h"
#include <common/tools.h>

using namespace POLYBOOLEAN; 
/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return
  * \param nh public node handler
  * \param nhp private node handler
  */
registration::RegistrationBase::RegistrationBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Read the node parameters
  readParameters();

  // Initialize the registration
  init();
}

/** \brief Messages callback. This function is called when synchronized odometry, images and pointcloud
  * messages are received.
  * @return
  * \param odom_msg ros odometry message of type nav_msgs::Odometry
  * \param l_img left stereo image message of type sensor_msgs::Image
  * \param r_img right stereo image message of type sensor_msgs::Image
  * \param l_info left stereo info message of type sensor_msgs::CameraInfo
  * \param r_info right stereo info message of type sensor_msgs::CameraInfo
  * \param cloud_msg ros pointcloud message of type sensor_msgs::PointCloud2
  */
void registration::RegistrationBase::msgsCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                                  const sensor_msgs::ImageConstPtr& l_img_msg,
                                                  const sensor_msgs::ImageConstPtr& r_img_msg,
                                                  const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                                  const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                                                  const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //if (!first_iter_) return;
  ROS_INFO_STREAM(" hem rebut un missatge sincronitzat");
  // Get the cloud
  PointCloud::Ptr pcl_cloud(new PointCloud);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

  // Filter the cloud
  filter(pcl_cloud);
  
  // calculate bounding box
  pcl::PointXYZRGB min_pt, max_pt;
  pcl::getMinMax3D(*pcl_cloud, min_pt, max_pt);

  // compute 2D vertex from bounding box
  rectangle current_rectangle, current_rectangle_transformed;
  //Point corner_down_right, corner_down_left, corner_up_right, corner_up_left;
  //compute_2D_vertex(min_pt, max_pt, &corner_down_right, &corner_down_left, &corner_up_right, &corner_up_left);
  compute_2D_vertex(min_pt, max_pt, &current_rectangle);
  // view original bounding box and vertex: 
  ROS_INFO_STREAM("bounding box: (" << min_pt.x << "," << min_pt.y << ")" << ";;" << " ( " << max_pt.x << " , " << max_pt.y << ")");


  double crcdlx, crcdly, crcdrx, crcdry, crcurx, crcury, crculx, crculy;
  double lrcdlx,lrcdly,lrcdrx,lrcdry,lrcurx,lrcury,lrculx,lrculy;

  crcdlx=(1000*current_rectangle.corner_down_left.x);
  crcdly=(1000*current_rectangle.corner_down_left.y);
  crcdrx=(1000*current_rectangle.corner_down_right.x);
  crcdry=(1000*current_rectangle.corner_down_right.y);
  crcurx=(1000*current_rectangle.corner_up_right.x);
  crcury=(1000*current_rectangle.corner_up_right.y);
  crculx=(1000*current_rectangle.corner_up_left.x);
  crculy=(1000*current_rectangle.corner_up_left.y);


  ROS_INFO_STREAM(" rectangle vertex: (" << current_rectangle.corner_down_left.x*1000 << "," << current_rectangle.corner_down_left.y*1000 << ")" << "\n" );
  ROS_INFO_STREAM("(" << current_rectangle.corner_down_right.x*1000 << "," << current_rectangle.corner_down_right.y*1000 << ")" << "\n" );
  ROS_INFO_STREAM("(" << current_rectangle.corner_up_right.x*1000 << "," << current_rectangle.corner_up_right.y*1000 << ")" << "\n" );
  ROS_INFO_STREAM("(" << current_rectangle.corner_up_left.x*1000 << "," << current_rectangle.corner_up_left.y*1000 << ")" << "\n" );
  ROS_INFO_STREAM(" last rectangle vertex: (" << last_rectangle.corner_down_left.x*1000 << "," << last_rectangle.corner_down_left.y*1000 << ")" << "\n" );
  ROS_INFO_STREAM("(" << last_rectangle.corner_down_right.x*1000 << "," << last_rectangle.corner_down_right.y*1000 << ")" << "\n" );
  ROS_INFO_STREAM("(" << last_rectangle.corner_up_right.x*1000<< "," << last_rectangle.corner_up_right.y*1000 << ")" << "\n" );
  ROS_INFO_STREAM("(" << last_rectangle.corner_up_left.x*1000<< "," << last_rectangle.corner_up_left.y*1000 << ")" << "\n" );
// convert points into tfVector4, 
  double zero=0.0;
  double uno=1.0;
  tf::Transform current_odom;
  current_odom = slam::Tools::odomTotf(*odom_msg); // transform odom message into a TF
  
  if (first_iter_ == false) // firt iteration there is not last_odom. 
  {
  tf::Transform t3 = last_odom.inverseTimes(current_odom); // transform points from the current polygon with respect to the 
  // previous polygon. (tflast.inverse)*tfcurrent
  tf::tfVector4 currrectp1(crcdlx,crcdly,zero,uno);
  tf::tfVector4 currrectp2(crcdrx,crcdry,zero,uno);
  tf::tfVector4 currrectp3(crcurx,crcury,zero,uno);
  tf::tfVector4 currrectp4(crculx,crculy,zero,uno); // change of structure to 
  // multiply a vector of 4 components by a tf matrix of 3x4. The result is a 1x3 vector.
  
  tf::Vector3 currrectp1_transformed=t3*currrectp1;
  tf::Vector3 currrectp2_transformed=t3*currrectp2;
  tf::Vector3 currrectp3_transformed=t3*currrectp3;
  tf::Vector3 currrectp4_transformed=t3*currrectp4;// current polygon vertex transformed 
  // to the last point cloud system of coordinates

  ROS_INFO_STREAM(" Vector3 1 " << currrectp1_transformed.x() << "," << currrectp1_transformed.y());
  ROS_INFO_STREAM(" Vector3 2 " << currrectp2_transformed.x() << "," << currrectp2_transformed.y());
  ROS_INFO_STREAM(" Vector3 3 " << currrectp3_transformed.x() << "," << currrectp3_transformed.y());
  ROS_INFO_STREAM(" Vector3 4 " << currrectp4_transformed.x() << "," << currrectp4_transformed.y());


  current_rectangle_transformed.corner_down_left.x=currrectp1_transformed.x();
  current_rectangle_transformed.corner_down_left.y=currrectp1_transformed.y();
  current_rectangle_transformed.corner_down_right.x=currrectp2_transformed.x();
  current_rectangle_transformed.corner_down_right.y=currrectp2_transformed.y();
  current_rectangle_transformed.corner_up_right.x=currrectp3_transformed.x();
  current_rectangle_transformed.corner_up_right.y=currrectp3_transformed.y();
  current_rectangle_transformed.corner_up_left.x=currrectp4_transformed.x();
  current_rectangle_transformed.corner_up_left.y=currrectp4_transformed.y();

 // calculate intersection area, if any. 
  double Area= compute_Intersect_Area(current_rectangle_transformed, last_rectangle);

    if (Area < 0) 
      ROS_INFO_STREAM("Error in the Area Calculation");

  }
  
  // draw the cloud and the box
  pcl::visualization::PCLVisualizer viewer;
  viewer.addPointCloud(pcl_cloud);
  viewer.addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);
  viewer.spin();
  last_rectangle=current_rectangle;
  last_odom=current_odom;
  first_iter_ = false;
}

void registration::RegistrationBase::compute_2D_vertex(pcl::PointXYZRGB minims, pcl::PointXYZRGB maxims, rectangle *r1)
{
r1->corner_down_left.x=minims.x;
r1->corner_down_left.y=minims.y;
r1->corner_down_right.x=maxims.x;
r1->corner_down_right.y=minims.y;
r1->corner_up_right.x=maxims.x;
r1->corner_up_right.y=maxims.y;
r1->corner_up_left.x=minims.x;
r1->corner_up_left.y=maxims.y;
}

/*****************************************************************************************
compute the intersection area between 2 polygons in 2D using the Polybool lib. 
*****************************************************************************************/

double registration::RegistrationBase::compute_Intersect_Area(rectangle current_rectangle, rectangle last_rectangle)
{

    INT32 crcdlx, crcdly, crcdrx, crcdry, crcurx, crcury, crculx, crculy;
    INT32 lrcdlx,lrcdly,lrcdrx,lrcdry,lrcurx,lrcury,lrculx,lrculy;

    crcdlx=(INT32)(current_rectangle.corner_down_left.x);
    crcdly=(INT32)(current_rectangle.corner_down_left.y);
    crcdrx=(INT32)(current_rectangle.corner_down_right.x);
    crcdry=(INT32)(current_rectangle.corner_down_right.y);
    crcurx=(INT32)(current_rectangle.corner_up_right.x);
    crcury=(INT32)(current_rectangle.corner_up_right.y);
    crculx=(INT32)(current_rectangle.corner_up_left.x);
    crculy=(INT32)(current_rectangle.corner_up_left.y);

    lrcdlx=(INT32)(1000*last_rectangle.corner_down_left.x);
    lrcdly=(INT32)(1000*last_rectangle.corner_down_left.y);
    lrcdrx=(INT32)(1000*last_rectangle.corner_down_right.x);
    lrcdry=(INT32)(1000*last_rectangle.corner_down_right.y);
    lrcurx=(INT32)(1000*last_rectangle.corner_up_right.x);
    lrcury=(INT32)(1000*last_rectangle.corner_up_right.y);
    lrculx=(INT32)(1000*last_rectangle.corner_up_left.x);
    lrculy=(INT32)(1000*last_rectangle.corner_up_left.y);

    ROS_INFO_STREAM("(" << crcdlx << crcdly << crcdrx << crcdry << crcurx << crcury << crculx << crculy << 
     lrcdlx << lrcdly << lrcdrx << lrcdry << lrcurx << lrcury << lrculx << lrculy << ")" << "\n" );

    PAREA *A=NULL;
    PAREA *B=NULL;      
    PLINE2 *pline=NULL; // polyline
    int i;
    // build grid struct with corners of the point cloud (x,y) plane. 
    GRID2 current_rect[4] = { {crcdlx,crcdly}, {crcdrx,crcdry}, {crcurx,crcury}, {crculx,crculy} };
    GRID2 last_rect[4] = { {lrcdlx,lrcdly}, {lrcdrx,lrcdry}, {lrcurx,lrcury}, {lrculx,lrculy} };
    // construct 2D polygon of current point cloud by creating a polyline from the points
    for (i = 0; i < 4; i++) 
    PLINE2::Incl(&pline, current_rect[i]); // includes a vertex in the contour pline
    //pline->Prepare(); 
    bool a1 = pline->Prepare();
    double a = pline->Prepare2();

    if (a > 0)
      ROS_INFO_STREAM(" 1st polygon (current) is coherent, area = " << a/2); // I do not why (yet....) area given by the routine is double.
    else
      return -1;

    if (not pline->IsOuter()) // make sure the contour is outer 
    pline->Invert();
    PAREA::InclPline(&A, pline); 
    pline = NULL; 

    for (i = 0; i < 4; i++) 
    PLINE2::Incl(&pline, last_rect[i]);
    a1 = pline->Prepare();
    a = pline->Prepare2();
    
    if (a > 0)
      ROS_INFO_STREAM(" 2nd polygon (previous) is coherent, area = " << a/2);  //I do not why (yet....) area given by the routine is double.
    else
      return -1;

    if (not pline->IsOuter()) // make sure the contour is outer 
    pline->Invert();
    PAREA::InclPline(&B, pline); 

    // boolean operation intersection
    PAREA * R = NULL; 
    int err;
    PAREA * pa = NULL;
    double nArea;
    PLINE2 * pline_intern;

    if (a > 0 )
    { // if both polygons are ok and have area different from 0, search for the intersection 
      ROS_INFO_STREAM("both polygons ok, search for the intersection");
      err = PAREA::Boolean(A, B, &R, PAREA::IS); // OpCode defined in polybool.h; IS--> intersection 
      pa = R;
      if ((pa->cntr)!= NULL){
      pline_intern = pa->cntr;
      nArea=(pline_intern->Prepare2())/(2*1000000); // I do not why (yet....) area given by the routine is double.
      // area in squared meters.   
      ROS_INFO_STREAM("Consecutive Point Clouds with overlap !! Area:" << nArea);
      }
      else{
        ROS_INFO_STREAM("Consecutive Point Clouds without overlap !! ");
        return -1;
      }
    }
    //const PAREA * pa = area;
    //const PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
    else {
      return -1;
      ROS_INFO_STREAM("one of both polygons is ko, do not search for the intersection");
    }

  // com evaluem R ?? 
    
    
  // delete all polygons 
    PAREA::Del(&A); 
    PAREA::Del(&B); 
    PAREA::Del(&R);
    return nArea;
}

/** \brief Reads the stereo slam node parameters
  * @return
  */
void registration::RegistrationBase::readParameters()
{
  Params params;

  // Topic parameters
  string odom_topic, left_topic, right_topic, left_info_topic, right_info_topic, cloud_topic;
  nh_private_.param("odom_topic", odom_topic, string("/odometry"));
  nh_private_.param("left_topic", left_topic, string("/left/image_rect_color"));
  nh_private_.param("right_topic", right_topic, string("/right/image_rect_color"));
  nh_private_.param("left_info_topic", left_info_topic, string("/left/camera_info"));
  nh_private_.param("right_info_topic", right_info_topic, string("/right/camera_info"));
  nh_private_.param("cloud_topic", cloud_topic, string("/points2"));

  // Filter parameters
  nh_private_.param("x_filter_min", params.x_filter_min, -2.0);
  nh_private_.param("x_filter_max", params.x_filter_max, 2.0);
  nh_private_.param("y_filter_min", params.y_filter_min, -2.0);
  nh_private_.param("y_filter_max", params.y_filter_max, 2.0);
  nh_private_.param("z_filter_min", params.z_filter_min, 0.2);
  nh_private_.param("z_filter_max", params.z_filter_max, 2.0);
  nh_private_.param("voxel_size_x", params.voxel_size_x, 0.005);
  nh_private_.param("voxel_size_y", params.voxel_size_y, 0.005);
  nh_private_.param("voxel_size_z", params.voxel_size_z, 0.005);
  nh_private_.param("radius_search", params.radius_search, 0.2);
  nh_private_.param("min_neighors_in_radius", params.min_neighors_in_radius, 20);

  // Set the class parameters
  setParams(params);

  // Topics subscriptions
  image_transport::ImageTransport it(nh_);
  odom_sub_       .subscribe(nh_, odom_topic,       1);
  left_sub_       .subscribe(it,  left_topic,       1);
  right_sub_      .subscribe(it,  right_topic,      1);
  left_info_sub_  .subscribe(nh_, left_info_topic,  1);
  right_info_sub_ .subscribe(nh_, right_info_topic, 1);
  cloud_sub_      .subscribe(nh_, cloud_topic,      1);
}

/** \brief Initializes the registration node
  * @return true if init OK
  */
bool registration::RegistrationBase::init()
{
  // Callback synchronization
  exact_sync_.reset(new ExactSync(ExactPolicy(5),
                                  odom_sub_,
                                  left_sub_,
                                  right_sub_,
                                  left_info_sub_,
                                  right_info_sub_,
                                  cloud_sub_) );
  exact_sync_->registerCallback(boost::bind(
      &registration::RegistrationBase::msgsCallback,
      this, _1, _2, _3, _4, _5, _6));

  first_iter_ = true;
  last_rectangle.corner_down_left.x=0.0;
  last_rectangle.corner_down_left.y=0.0;
  last_rectangle.corner_down_right.x=0.0;
  last_rectangle.corner_down_right.y=0.0;
  last_rectangle.corner_up_right.x=0.0;
  last_rectangle.corner_up_right.y=0.0;
  last_rectangle.corner_up_left.x=0.0;
  last_rectangle.corner_up_left.y=0.0;
  
  return true;
}

/** \brief Filter the input cloud
  * \param input cloud
  */
void registration::RegistrationBase::filter(PointCloud::Ptr cloud)
{
  // NAN and limit filtering
  pcl::PassThrough<pcl::PointXYZRGB> pass_;

  // X-filtering
  pass_.setFilterFieldName("x");
  pass_.setFilterLimits(params_.x_filter_min, params_.x_filter_max);
  pass_.setInputCloud(cloud);
  pass_.filter(*cloud);

  // Y-filtering
  pass_.setFilterFieldName("y");
  pass_.setFilterLimits(params_.y_filter_min, params_.y_filter_max);
  pass_.setInputCloud(cloud);
  pass_.filter(*cloud);

  // Z-filtering
  pass_.setFilterFieldName("z");
  pass_.setFilterLimits(params_.z_filter_min, params_.z_filter_max);
  pass_.setInputCloud(cloud);
  pass_.filter(*cloud);


  // Downsampling using voxel grid
  pcl::VoxelGrid<pcl::PointXYZRGB> grid_;
  grid_.setLeafSize(params_.voxel_size_x,
                    params_.voxel_size_y,
                    params_.voxel_size_z);
  grid_.setDownsampleAllData(true);
  grid_.setInputCloud(cloud);
  grid_.filter(*cloud);

  // Remove isolated points
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter(*cloud);

}