#ifndef HASH_H
#define HASH_H

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <Eigen/Eigen>
#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;

namespace slam
{

class Hash
{

public:

  /** \brief Hash class constructor
    */
  Hash();

  struct Params
  {
    /** \brief Parameter constructor. Sets the parameter struct to default values.
     */
    Params();

    // Class parameters
    int num_proj; //!> Number of projections required

    // Default values
    static const int DEFAULT_NUM_PROJ = 2;
  };

  /** \brief Sets the parameters
   * \param the parameters set
   */
  void setParams(const Params& params);

  /** \brief Return true if the class has been initialized
    * @return
    */
  bool isInitialized();

  /** \brief Get class params
   */
  inline Params params() const { return params_; }

  /** \brief Initialize hash
   * \param sample descriptors matrix
   */
  void init(Mat desc);

  /** \brief Compute the hash
   * @return hash vector
   * \param cvMat containing the descriptors of the image
   */
  vector<float> getHash(Mat desc);

  /** \brief Compute the distance between 2 hashes
   * @return the distance
   * \param hash 1
   * \param hash 2
   */
  float match(vector<float> hash_1, vector<float> hash_2);

private:

  /** \brief Compute the random vector/s needed to generate the hash
   * \param size of the initial descriptor matrix.
   */
  void initProjections(int desc_size);

  /** \brief Computes a random vector of some size
   * @return random vector
   * \param seed to generate the random values
   * \param desired size
   */
  vector<float> compute_random_vector(uint seed, int size);

  /** \brief Make a vector unit
   * @return the "unitized" vector
   * \param the "non-unitized" vector
   */
  vector<float> unit_vector(vector<float> x);

  Params params_; //!> Stores parameters

  vector< vector<float> > r_; //!> Vector of random values

  int h_size_; //!> Size of the hash

  bool initialized_; //!> True when class has been initialized
};

} // namespace

#endif // HASH_H