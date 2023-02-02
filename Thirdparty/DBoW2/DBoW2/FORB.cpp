/**
 * File: FORB.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: functions for Surf64 descriptors
 * License: see the LICENSE.txt file
 *
 */

#include <vector>
#include <string>
#include <sstream>

#include "FClass.h"
#include "FORB.h"

using namespace std;

namespace DBoW2
{

  // --------------------------------------------------------------------------

  void FORB::meanValue(const std::vector<FORB::pDescriptor> &descriptors,
                          FORB::TDescriptor &mean)
  {
    // mean.resize(0);
    // mean.resize(FORB::L, 0);
    mean.zeros(1, FORB::L, CV_32F);

    float s = descriptors.size();

    vector<FORB::pDescriptor>::const_iterator it;
    for (it = descriptors.begin(); it != descriptors.end(); ++it)
    {
      const FORB::TDescriptor &desc = **it;
      mean += desc/s;
      // for (int i = 0; i < FORB::L; i += 4)
      // {
      //   mean[i] += desc[i] / s;
      //   mean[i + 1] += desc[i + 1] / s;
      //   mean[i + 2] += desc[i + 2] / s;
      //   mean[i + 3] += desc[i + 3] / s;
      // }
    }
  }

  // --------------------------------------------------------------------------

  double FORB::distance(const FORB::TDescriptor &a, const FORB::TDescriptor &b)
  {
    double sqd = cv::norm(a, b, cv::NORM_L2);
    // for (int i = 0; i < FORB::L; i += 4)
    // {
    //   sqd += (a[i] - b[i]) * (a[i] - b[i]);
    //   sqd += (a[i + 1] - b[i + 1]) * (a[i + 1] - b[i + 1]);
    //   sqd += (a[i + 2] - b[i + 2]) * (a[i + 2] - b[i + 2]);
    //   sqd += (a[i + 3] - b[i + 3]) * (a[i + 3] - b[i + 3]);
    // }
    return sqd;
  }

  // --------------------------------------------------------------------------

  std::string FORB::toString(const FORB::TDescriptor &a)
  {
    stringstream ss;
    for (int i = 0; i < FORB::L; ++i)
    {
      ss << to_string(a.at<float>(0,i)) << " ";
    }
    return ss.str();
  }

  // --------------------------------------------------------------------------

  void FORB::fromString(FORB::TDescriptor &a, const std::string &s)
  {
    a.create(1, FORB::L, CV_32F);

    stringstream ss(s);
    for (int i = 0; i < FORB::L; ++i)
    {
      ss >> a.at<float>(0,i);
    }
  }

  // --------------------------------------------------------------------------

  void FORB::toMat32F(const std::vector<TDescriptor> &descriptors,
                         cv::Mat &mat)
  {
    if (descriptors.empty())
    {
      mat.release();
      return;
    }

    const int N = descriptors.size();
    const int L = FORB::L;

    mat.create(N, L, CV_32F);

    for (int i = 0; i < N; ++i)
    {
      const TDescriptor &desc = descriptors[i];
      desc.copyTo(mat.row(i));
      // float *p = mat.ptr<float>(i);
      // for (int j = 0; j < L; ++j, ++p)
      // {
      //   *p = desc[j];
      // }
    }
  }

  // --------------------------------------------------------------------------

} // namespace DBoW2