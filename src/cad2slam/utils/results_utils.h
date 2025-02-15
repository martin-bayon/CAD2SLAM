#pragma once

#include <vector>
#include <numeric>
#include <valarray>

namespace cad2slam{
  float calculateMin(std::vector<float> values, bool negativeFree);
  float calculateMax(std::vector<float> values, bool negativeFree);
  float calculateMean(std::vector<float> values, bool negativeFree);
  float calculateStdDev(std::vector<float> values, bool negativeFree);
}