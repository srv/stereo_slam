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
}

/** \brief Get the receiver
  * @return the receiver of this class
  */
reconstruction::Receiver reconstruction::ReconstructionBase::getReceiver()
{
  return receiver_;
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
    ROS_ERROR("[Reconstruction:] ERROR -> Impossible to create the reconstruction directory.");

  // Set class parameters
  setParams(params);

  // Receiver parameters
  reconstruction::Receiver::Params receiver_params;
  nh_private_.param("start_srv",        receiver_params.start_srv,          string(""));
  nh_private_.param("stop_srv",         receiver_params.stop_srv,           string(""));
  //nh_private_.param("min_pose_change",  viewer_params.min_pose_change,      0.005);
  receiver_params.work_dir = params.work_dir;
  receiver_params.nh_private = nh_private_;
  receiver_params.nh = nh_;
  receiver_.setParams(receiver_params);
}


/** \brief Initialize the node
  */
void reconstruction::ReconstructionBase::start()
{
  ROS_INFO_STREAM("[Reconstruction:] Starting reconstruction...");

  // Start the receiver
  receiver_.start();
}

/** \brief Finalizes the node
  */
void reconstruction::ReconstructionBase::stop()
{
  ROS_INFO_STREAM("[Reconstruction:] Finalizing reconstruction...");
  receiver_.stop();
}