#include <ros/ros.h>
#include "localization/hash.h"
#include <opencv2/core/eigen.hpp>

namespace slam
{

  Hash::Params::Params() :
    num_proj(DEFAULT_NUM_PROJ)
  {}

  Hash::Hash()
  {
    // Initializations
    h_size_ = -1;
    initialized_ = false;
  }

  bool Hash::isInitialized()
  {
    return initialized_;
  }

  void Hash::setParams(const Params& params)
  {
    params_ = params;
  }

  void Hash::init(cv::Mat desc)
  {
    // Create the random projections vectors
    initProjections(desc.rows);

    // Set the size of the descriptors
    h_size_ = params_.num_proj * desc.cols;

    // Set initialized to true
    initialized_ = true;
  }

  vector<float> Hash::getHash(cv::Mat desc)
  {
    // Initialize the histogram with 0's
    vector<float> hash(h_size_, 0.0);

    // Sanity check
    if (desc.rows == 0) return hash;

    // Project the descriptors
    uint k = 0;
    for (uint i=0; i<r_.size(); i++)
    {
      for (int n=0; n<desc.cols; n++)
      {
        float desc_sum = 0.0;
        for (uint m=0; m<desc.rows; m++)
          desc_sum += r_[i][m]*desc.at<float>(m, n);

        hash[k] = desc_sum/(float)desc.rows;
        k++;
      }
    }

    return hash;
  }

  void Hash::initProjections(int desc_size)
  {
    // Initializations
    int seed = time(NULL);
    r_.clear();

    // The size of the descriptors may vary... We multiply the current descriptor size
    // for a scalar to handle the larger cases.
    int v_size = 6*desc_size;

    // We will generate N-orthogonal vectors creating a linear system of type Ax=b.

    // Generate a first random vector
    vector<float> r = compute_random_vector(seed, v_size);
    r_.push_back(unit_vector(r));

    // Generate the set of orthogonal vectors
    for (uint i=1; i<params_.num_proj; i++)
    {
      // Generate a random vector of the correct size
      vector<float> new_v = compute_random_vector(seed + i, v_size - i);

      // Get the right terms (b)
      VectorXf b(r_.size());
      for (uint n=0; n<r_.size(); n++)
      {
        vector<float> cur_v = r_[n];
        float sum = 0.0;
        for (uint m=0; m<new_v.size(); m++)
        {
          sum += new_v[m]*cur_v[m];
        }
        b(n) = -sum;
      }

      // Get the matrix of equations (A)
      MatrixXf A(i, i);
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
      VectorXf x = A.colPivHouseholderQr().solve(b);

      // Add the solutions to the new vector
      for (uint n=0; n<r_.size(); n++)
        new_v.push_back(x(n));
      new_v = unit_vector(new_v);

      // Push the new vector
      r_.push_back(new_v);
    }
  }

  vector<float> Hash::compute_random_vector(uint seed, int size)
  {
    srand(seed);
    vector<float> h;
    for (int i=0; i<size; i++)
      h.push_back( (float)rand()/RAND_MAX );
    return h;
  }

  vector<float> Hash::unit_vector(vector<float> x)
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

  float Hash::match(vector<float> hash_1, vector<float> hash_2)
  {
    // Compute the distance
    float sum = 0.0;
    for (uint i=0; i<hash_1.size(); i++)
      sum += fabs(hash_1[i] - hash_2[i]);

    return sum;
  }

} //namespace slam
