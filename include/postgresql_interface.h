#ifndef MAP_NODE_H_
#define MAP_NODE_H_

#include <string>
#include <database_interface/db_class.h>

namespace stereo_slam
{

class GraphDB : public database_interface::DBClass
{

public:

  // Table columns
  database_interface::DBField<int> id_;
  database_interface::DBField< std::vector< std::vector<float> > > keypoints_;
  database_interface::DBField< std::vector< std::vector<float> > > descriptors_;
  database_interface::DBField< std::vector< std::vector<float> > > points3d_;

  GraphDB() : 
    id_(database_interface::DBFieldBase::TEXT, 
		this, "id", "graph", true),
    keypoints_(database_interface::DBFieldBase::TEXT, 
        this, "keypoints", "graph", true),
    descriptors_(database_interface::DBFieldBase::TEXT, 
        this, "descriptors", "graph", true),
    points3d_(database_interface::DBFieldBase::TEXT, 
        this, "points3d", "graph", true)
  {
    
    primary_key_field_ = &id_;

    // All other fields go into the fields_ array of the DBClass
    fields_.push_back(&keypoints_);
    fields_.push_back(&descriptors_);
    fields_.push_back(&points3d_);

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

    id_.setSequenceName("graph_id_seq");
    id_.setWriteToDatabase(false);
  }
};

} // Namespace

#endif