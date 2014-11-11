#include "reconstruction/online/base.h"
#include <boost/filesystem.hpp>

namespace fs=boost::filesystem;


/** \brief Class constructor. Reads node parameters and initialize some properties.
  * @return
  * \param nh public node handler
  * \param nhp private node handler
  */
reconstruction::ReconstructionBase::ReconstructionBase(
  ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nh_private_(nhp)
{
  // Read the node parameters
  readParameters();

  // Initialize the node
  init();
}


/** \brief Reads the reconstruction node parameters
  */
void reconstruction::ReconstructionBase::readParameters()
{
  Params params;

  // Operational directory
  string work_dir;
  nh_private_.param("work_dir", work_dir, string(""));
  if (work_dir[work_dir.length()-1] != '/')
    work_dir += "/";
  params.work_dir = work_dir + "reconstruction/";

  // Create the directory
  if (fs::is_directory(params.work_dir))
    fs::remove_all(params.work_dir);
  fs::path dir(params.work_dir);
  if (!fs::create_directory(dir))
    ROS_ERROR("[StereoSlam:] ERROR -> Impossible to create the reconstruction directory.");

  // Set class parameters
  setParams(params);

  // Requester parameters
  reconstruction::Requester::Params requester_params;
  nh_private_.param("get_point_cloud_srv", requester_params.get_point_cloud_srv, string(""));
  nh_private_.param("get_graph_srv", requester_params.get_graph_srv, string(""));
  requester_params.work_dir = params.work_dir;
  requester_params.nh_private = nh_private_;
  requester_params.nh = nh_;
  requester_.setParams(requester_params);
}


/** \brief Initialize the node
  */
void reconstruction::ReconstructionBase::init()
{
  // Start the requester
  requester_.start();
}