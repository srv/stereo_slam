#include <ros/ros.h>
#include "hash.hpp"
#include <opencv2/core/eigen.hpp>


/** \brief Parameter constructor. Sets the parameter struct to default values.
  */
haloc::Hash::Params::Params() :
  num_proj(DEFAULT_NUM_PROJ)
{}

/** \brief Hash class constructor
  */
haloc::Hash::Hash()
{
  // Initializations
  h_size_ = -1;
  initialized_ = false;
}

/** \brief Return true if the class has been initialized
  * @return
  */
bool haloc::Hash::isInitialized()
{
  return initialized_;
}

/** \brief Sets the class parameters
  * @return
  * \param stuct of parameters
  */
void haloc::Hash::setParams(const Params& params)
{
  params_ = params;
}

// Class initialization
void haloc::Hash::init(cv::Mat desc)
{
  // Create the random projections vectors
  initProjections(desc.rows);

  // Set the size of the descriptors
  h_size_ = params_.num_proj * desc.cols;

  // Set initialized to true
  initialized_ = true;
}

/** \brief Compute the hash
  * @return hash vector
  * \param cvMat containing the descriptors of the image
  */
std::vector<float> haloc::Hash::getHash(cv::Mat desc)
{
  // Initialize the histogram with 0's
  std::vector<float> hash(h_size_, 0.0);

  // Sanity check
  if (desc.rows == 0) return hash;

  // Project the descriptors
  uint k = 0;
  for (uint i=0; i<r_.size(); i++)
  {
    for (int n=0; n<desc.cols; n++)
    {
      float desc_sum = 0.0;
      for (int m=0; m<desc.rows; m++)
        desc_sum += r_[i][m]*desc.at<float>(m, n);

      hash[k] = desc_sum/(float)desc.rows;
      k++;
    }
  }

  return hash;
}

/** \brief Compute the random vector/s needed to generate the hash
  * @return
  * \param size of the initial descriptor matrix.
  * \param true to generate 'n' random orthogonal projections, false to generate 'n' random non-orthogonal projections.
  */
void haloc::Hash::initProjections(int desc_size)
{
  // Initializations
  int seed = time(NULL);
  r_.clear();

  // The size of the descriptors may vary... We multiply the current descriptor size
  // for a scalar to handle the larger cases.
  int v_size = 6*desc_size;

  // We will generate N-orthogonal vectors creating a linear system of type Ax=b.
  // Generate a first random vector
  std::vector<float> r = compute_random_vector(seed, v_size);
  r_.push_back(unit_vector(r));

  std::cout << "params_.num_proj: " <<  params_.num_proj << std::endl;

  // Generate the set of orthogonal vectors
  for (int i=1; i<params_.num_proj; i++)
  {
    // Generate a random vector of the correct size
    std::vector<float> new_v = compute_random_vector(seed + i, v_size - i);

    // Get the right terms (b)
    Eigen::VectorXf b(r_.size());
    for (uint n=0; n<r_.size(); n++)
    {
      std::vector<float> cur_v = r_[n];
      float sum = 0.0;
      for (uint m=0; m<new_v.size(); m++)
      {
        sum += new_v[m]*cur_v[m];
      }
      b(n) = -sum;
    }

    // Get the matrix of equations (A)
    Eigen::MatrixXf A(i, i);
    for (uint n=0; n<r_.size(); n++)
    {
      uint k=0;
      for (uint m=r_[n].size()-i; m<r_[n].size(); m++)
      {
        A(n,k) = r_[n][m];
        k++;
      }
    }

    // Apply the solver
    Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);

    // Add the solutions to the new vector
    for (uint n=0; n<r_.size(); n++)
      new_v.push_back(x(n));
    new_v = unit_vector(new_v);

    // Push the new vector
    r_.push_back(new_v);
  }
}

/** \brief Computes a random vector of some size
  * @return random vector
  * \param seed to generate the random values
  * \param desired size
  */
std::vector<float> haloc::Hash::compute_random_vector(uint seed, int size)
{
  srand(seed);
  std::vector<float> h;
  for (int i=0; i<size; i++)
    h.push_back( (float)rand()/RAND_MAX );
  return h;
}

/** \brief Make a vector unit
  * @return the "unitized" vector
  * \param the "non-unitized" vector
  */
std::vector<float> haloc::Hash::unit_vector(std::vector<float> x)
{
  // Compute the norm
  float sum = 0.0;
  for (uint i=0; i<x.size(); i++)
    sum += pow(x[i], 2.0);
  float x_norm = sqrt(sum);

  // x^ = x/|x|
  for (uint i=0; i<x.size(); i++)
    x[i] = x[i] / x_norm;

  return x;
}

/** \brief Compute the distance between 2 hashes
  * @return the distance
  * \param hash 1
  * \param hash 2
  */
float haloc::Hash::match(std::vector<float> hash_1, std::vector<float> hash_2)
{
  // Compute the distance
  float sum = 0.0;
  for (uint i=0; i<hash_1.size(); i++)
    sum += fabs(hash_1[i] - hash_2[i]);

  return sum;
}
