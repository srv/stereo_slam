#ifndef MAP_NODE_H_
#define MAP_NODE_H_

#include <string>
#include <database_interface/db_class.h>

// all database classes must inherit from database_interface::DBClass
class MapNode : public database_interface::DBClass
{

//fields are made public in this toy example for easy access, but you
//can treat them as you would any member data of your C++ classes
public:

  //key requirement: all fields that are to be stored in the 
  //database must be wrapped as database_interface::DBField<>, 
  //templated on the type of data they hold
  
  database_interface::DBField<int> id_;

  database_interface::DBField<std::string> descriptor_;

  database_interface::DBField<std::string> points_;

  database_interface::DBField<double> cam_x_;
  database_interface::DBField<double> cam_y_;
  database_interface::DBField<double> cam_z_;

  
  //key requirement: all fields must be initialized in the constructor
  //a field constructor takes the following arguments:
  // - the type of serialization used (TEXT for all fields in this toy example)
  // - the owner of the field ( usually "this", or the instance of the DBClass 
  //   that owns that field)
  // - the name of the table column corresponding to that field
  // - the name of the table in which the field is stored
  // - whether it is allowed to modify en entry in the database using a reference 
  //   to this field
  MapNode() : 
    id_(database_interface::DBFieldBase::TEXT, 
		this, "id", "image_info", true),
    descriptor_(database_interface::DBFieldBase::TEXT, 
			this, "descriptor", "image_info", true),
    points_(database_interface::DBFieldBase::TEXT, 
		       this, "points", "image_info", true),
    cam_x_(database_interface::DBFieldBase::TEXT, 
		    this, "cam_x", "image_info", true),
    cam_y_(database_interface::DBFieldBase::TEXT, 
        this, "cam_y", "image_info", true),
    cam_z_(database_interface::DBFieldBase::TEXT, 
        this, "cam_z", "image_info", true)
  {
    //finally, all fields must be registered with the DBClass itself

    //one field MUST be a primary key
    //all instances of DBClass have a primary_key_field_ pointer, 
    //which must be set on construction
    primary_key_field_ = &id_;

    //all other fields go into the fields_ array of the DBClass
    fields_.push_back(&descriptor_);
    fields_.push_back(&points_);
    fields_.push_back(&cam_x_);
    fields_.push_back(&cam_y_);
    fields_.push_back(&cam_z_);

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
  }
};

#endif