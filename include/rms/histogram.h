#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <set>

namespace rms
{

// key: point index, value: GFH norm
using t_gfh = std::pair<size_t, float>;
using t_indices = std::vector<size_t>;

/* //{ class Histogram1D */

class Histogram1D {

public:
  Histogram1D(const size_t no_bins, const float min_value, const float max_value, const std::vector<t_gfh> &gfh);

  float  getBinWidth();
  size_t getBinCount();

  void selectByUniformnessMaximization();
  void selectByUniformnessMaximization(float &entropy);

  void selectByNormMaximization();
  void selectByNormMaximization(float &entropy);

  t_indices getSelectedIndices();

private:
  size_t _no_bins;
  size_t _point_count_remaining;
  size_t _point_count_selected;

  size_t _bin_sel_ptr;

  float _bin_width;
  float _val_min;
  float _val_max;

  // vector (size: number of bins) of vectors of pairs (first: index in original cloud; second: value) sorted in descending order by pair value
  std::vector<std::vector<t_gfh>> _counts_remaining;
  std::vector<std::vector<t_gfh>> _counts_selected;

  void construct(const std::vector<t_gfh> &gfh);

  size_t             getBin(const float point);
  std::vector<float> estimatePDF();
  float              computeEntropy(const std::vector<float> &pdf);
};

//}

}  // namespace rms
