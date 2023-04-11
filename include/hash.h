#ifndef HASH_H
#define HASH_H

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <Eigen/Eigen>
#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;

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
  void init(Mat desc);

  // Compute the hash
  vector<float> getHash(Mat desc);

  // Compute the distance between 2 hashes
  float match(vector<float> hash_1, vector<float> hash_2);

private:

  // Init the random vectors for projections
  void initProjections(int desc_size);

  // Compute a random vector
  vector<float> compute_random_vector(uint seed, int size);

  // Make a vector unit
  vector<float> unit_vector(vector<float> x);

  // Properties
  Params params_;                           //!> Stores parameters
  vector< vector<float> > r_;               //!> Vector of random values
  int h_size_;                              //!> Size of the hash
  bool initialized_;                        //!> True when class has been initialized
};

} // namespace

#endif // HASH_H