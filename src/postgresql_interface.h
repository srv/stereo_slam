#ifndef MAP_NODE_H_
#define MAP_NODE_H_

#include <string>
#include <database_interface/db_class.h>
#include <opencv2/features2d/features2d.hpp>

namespace stereo_localization
{

class GraphNodes : public database_interface::DBClass
{

public:

  // Table columns
  database_interface::DBField<int> id_;

  database_interface::DBField<double> pose_x_;
  database_interface::DBField<double> pose_y_;
  database_interface::DBField<double> pose_z_;
  database_interface::DBField< std::vector<double> > pose_rotation_;
  database_interface::DBField< std::vector< std::vector<double> > > descriptors_;  

  GraphNodes() : 
    id_(database_interface::DBFieldBase::TEXT, 
		this, "id", "graph_nodes", true),
    pose_x_(database_interface::DBFieldBase::TEXT, 
		    this, "pose_x", "graph_nodes", true),
    pose_y_(database_interface::DBFieldBase::TEXT, 
        this, "pose_y", "graph_nodes", true),
    pose_z_(database_interface::DBFieldBase::TEXT, 
        this, "pose_z", "graph_nodes", true),
    pose_rotation_(database_interface::DBFieldBase::TEXT, 
        this, "pose_rotation", "graph_nodes", true),
    descriptors_(database_interface::DBFieldBase::TEXT, 
        this, "descriptors", "graph_nodes", true)
  {
    
    primary_key_field_ = &id_;

    // All other fields go into the fields_ array of the DBClass
    fields_.push_back(&pose_x_);
    fields_.push_back(&pose_y_);
    fields_.push_back(&pose_z_);
    fields_.push_back(&pose_rotation_);
    fields_.push_back(&descriptors_);

    //optional: let all fields be read automatically when an instance 
    //is retrieved from the database
    setAllFieldsReadFromDatabase(true);
    //optional: let all fields be written automatically when an instance 
    //is saved the database
    setAllFieldsWriteToDatabase(true);
    //(these options are usful if you have a very large field (e.g. a 
    // binary bitmap) which you do not 
    //want retrieved automatically whenever you get info 
    //from the database

    id_.setSequenceName("graph_nodes_id_seq");
    id_.setWriteToDatabase(false);
  }
};

} // Namespace

#endif