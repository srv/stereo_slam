/**
 * @file
 * @brief 3D reconstruction using the output of the stereo_slam (presentation).
 */

#ifndef BASE_H
#define BASE_H

#include <ros/ros.h>
#include "requester.h"

using namespace std;

namespace reconstruction
{

class ReconstructionBase
{

public:

	// Constructor
  ReconstructionBase(ros::NodeHandle nh, ros::NodeHandle nhp);

  struct Params
  {
    string work_dir;              //!> Working directory.

    // Default settings
    Params () {
      work_dir                    = "";
    }
  };

  /**
   * @param params new parameters
   */
  inline void setParams(const Params& params)
  {
    params_ = params;
  }

  /**
   * @return current parameters
   */
  inline Params params() const { return params_; }

protected:

	// Node handlers
	ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Protected functions and callbacks
  void readParameters();
  void init();

private:

  Params params_;                         //!> Stores parameters
  reconstruction::Requester requester_;   //!> Requester object
};

} // namespace

#endif // BASE_H