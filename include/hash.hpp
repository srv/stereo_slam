#ifndef HASH_HPP
#define HASH_HPP

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <Eigen/Eigen>
#include <Eigen/Dense>


namespace haloc
{

class Hash
{

public:

  // Class constructor
  Hash();

  struct Params
  {
    //Default constructor sets all values to defaults.
    Params();

    // Class parameters
    int num_proj;                   //!> Number of projections required

    // Default values
    static const int              DEFAULT_NUM_PROJ = 3;
  };

  // Set the parameter struct
  void setParams(const Params& params);

  // Returns true if the class has been initialized
  bool isInitialized();

  // Return current parameters
  inline Params params() const { return params_; }

  // Initialize class
  void init(cv::Mat desc);

  // Compute the hash
  std::vector<float> getHash(cv::Mat desc);

  // Compute the distance between 2 hashes
  float match(std::vector<float> hash_1, std::vector<float> hash_2);

private:

  // Init the random vectors for projections
  void initProjections(int desc_size);

  // Compute a random vector
  std::vector<float> compute_random_vector(uint seed, int size);

  // Make a vector unit
  std::vector<float> unit_vector(std::vector<float> x);

  // Properties
  int h_size_;                              //!> Size of the hash
  Params params_;                           //!> Stores parameters
  bool initialized_;                        //!> True when class has been initialized
  std::vector< std::vector<float> > r_;     //!> Vector of random values
};

} // namespace

#endif // HASH_HPP