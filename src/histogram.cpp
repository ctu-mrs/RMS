#include <rms/histogram.h>

namespace rms
{

/*//{ Histogram1D() */
Histogram1D::Histogram1D(const size_t no_bins, const float min_value, const float max_value, const std::vector<t_gfh> &gfh) {
  _no_bins   = no_bins;
  _val_min   = min_value;
  _val_max   = max_value;
  _bin_width = (_val_max - _val_min) / _no_bins;

  // L25: variable "k"
  _bin_sel_ptr = _no_bins - 1;

  construct(gfh);
}

/*//}*/

/*//{ getBinWidth() */
float Histogram1D::getBinWidth() {
  return _bin_width;
}
/*//}*/

/*//{ getBinCount() */
size_t Histogram1D::getBinCount() {
  return _no_bins;
}
/*//}*/

/*//{ getBin() */
size_t Histogram1D::getBin(const float point) {

  if (point < _val_min) {
    std::cout << "[Histogram1D::getBin] Given parameter is lower than minimal bound, returning first bin." << std::endl;
    return 0;
  } else if (point > _val_max) {
    std::cout << "[Histogram1D::getBin] Given parameter is greater than maximal bound, returning last bin." << std::endl;
    return _no_bins - 1;
  }

  const size_t b = std::floor(point / _bin_width);

  // Handle edge case: point == _val_max
  if (b == _no_bins) {
    return _no_bins - 1;
  }

  return b;
}
/*//}*/

/*//{ construct() */
void Histogram1D::construct(const std::vector<t_gfh> &gfh) {

  // L15-L17
  _counts_remaining.resize(_no_bins, {});
  _counts_selected.resize(_no_bins, {});

  // Discretize values to bins
  for (const auto &idx_gfh : gfh) {
    const auto b = getBin(idx_gfh.second);  // Get bin index given the gfh norm
    _counts_remaining[b].emplace_back(idx_gfh);
  }

  // L21: Sort each bin by the value in ascending order (will select from back later)
  // Note: sorting with only the primary key here
  for (auto &i_p_pairs : _counts_remaining) {
    std::stable_sort(i_p_pairs.begin(), i_p_pairs.end(), [](const auto &a, const auto &b) { return a.second < b.second; });
  }

  // Reserve max space for selected counts
  for (size_t i = 0; i < _counts_remaining.size(); i++) {
    _counts_selected.at(i).reserve(_counts_remaining.at(i).size());
  }

  _point_count_remaining = gfh.size();
  _point_count_selected  = 0;
}
/*//}*/

/*//{ selectByUniformnessMaximization() */
// Selects 1 point from the histogram such that the histogram uniformness is maximized
void Histogram1D::selectByUniformnessMaximization() {

  // Nothing to do if there's nothing to remove
  if (_point_count_remaining == 0) {
    return;
  }

  // Find first non-empty bin going from right (TODO: could be improved by keeping list of nonempty bins instead)
  while (_counts_remaining.at(_bin_sel_ptr).empty()) {

    // Handle cyclic variable
    if (_bin_sel_ptr == 0) {
      _bin_sel_ptr = _no_bins - 1;
    } else {
      _bin_sel_ptr--;
    }
  }

  // Retrieve bin at index "k"
  auto &bin = _counts_remaining.at(_bin_sel_ptr);

  // Store the point with the highest norm
  _counts_selected.at(_bin_sel_ptr).emplace_back(bin.back());

  // Reduce the peak's count by 1 (remove point with the lowest norm)
  bin.pop_back();

  // Reduce total point count
  _point_count_remaining--;
  _point_count_selected++;

  // L28: In next iter, start at the bin on the left (handle cyclic variable)
  if (_bin_sel_ptr == 0) {
    _bin_sel_ptr = _no_bins - 1;
  } else {
    _bin_sel_ptr--;
  }
}
/*//}*/

/*//{ selectByUniformnessMaximization() */
void Histogram1D::selectByUniformnessMaximization(float &entropy) {

  // Reduce by one point
  selectByUniformnessMaximization();

  // estimate pdf
  const auto &pdf = estimatePDF();

  // return entropy
  entropy = computeEntropy(pdf);
}
/*//}*/

/*//{ selectByNormMaximization() */
// Select 1 point from the histogram such that the uniformness is maximized
void Histogram1D::selectByNormMaximization() {

  // Nothing to do if there's nothing to remove
  if (_point_count_remaining == 0) {
    return;
  }

  // Find first non-empty bin going from right
  while (_counts_remaining.at(_bin_sel_ptr).empty()) {
    _bin_sel_ptr--;
  }

  auto &bin = _counts_remaining.at(_bin_sel_ptr);

  // store the point with the highest norm
  _counts_selected.at(_bin_sel_ptr).emplace_back(bin.back());

  // reduce the bin's count by 1 (remove point with the lowest norm)
  bin.pop_back();

  // reduce total point count
  _point_count_remaining--;
  _point_count_selected++;
}
/*//}*/

/*//{ selectByNormMaximization() */
void Histogram1D::selectByNormMaximization(float &entropy) {

  // Reduce by one point
  selectByNormMaximization();

  // estimate pdf
  const auto &pdf = estimatePDF();

  // return entropy
  entropy = computeEntropy(pdf);
}
/*//}*/

/*//{ getSelectedIndices() */
t_indices Histogram1D::getSelectedIndices() {

  t_indices indices;
  indices.reserve(_point_count_selected);

  // L29: merge indices stored in all bins
  for (const auto &bin : _counts_selected) {
    for (const auto &i_p : bin) {
      indices.emplace_back(i_p.first);
    }
  }

  return indices;
}
/*//}*/

/*//{ estimatePDF() */
std::vector<float> Histogram1D::estimatePDF() {

  std::vector<float> pdf;
  if (_point_count_selected == 0) {
    return pdf;
  }
  pdf.reserve(_no_bins);

  // frequentist's approch (counts -> pdf)
  std::transform(_counts_selected.cbegin(), _counts_selected.cend(), std::back_inserter(pdf),
                 [&](const auto &bin) { return float(bin.size()) / float(_point_count_selected); });

  return pdf;
}
/*//}*/

/*//{ computeEntropy() */
float Histogram1D::computeEntropy(const std::vector<float> &pdf) {
  // Eq. (29)
  float entropy = 0.0f;
  for (const float p : pdf) {
    if (p > 0.0) {
      entropy -= p * std::log10(p);
    }
  }
  return entropy;
}
/*//}*/

}  // namespace rms
