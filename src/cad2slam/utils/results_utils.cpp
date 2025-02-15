#include "results_utils.h"

namespace cad2slam{
  std::vector<float> removeNegativeValues(std::vector<float> values){
    std::vector<float> positive_values;
    for(float v : values){
      if(v>=0){
        positive_values.push_back(v);
      }
    }
    return positive_values;
  }

  float calculateMean(std::vector<float> values, bool negativeFree){
    std::vector<float> positive_values;
    if(negativeFree){
      positive_values = values;
    }else{
      positive_values = removeNegativeValues(values);
    }
    float sum = std::accumulate(positive_values.begin(), positive_values.end(), 0.0);
    float mean = sum/positive_values.size();
    return mean;
  }

  float calculateStdDev(std::vector<float> values, bool negativeFree){
    std::vector<float> positive_values;
    if(negativeFree){
      positive_values = values;
    }else{
      positive_values = removeNegativeValues(values);
    }
    float mean = calculateMean(positive_values, true);
    float sq_sum = std::inner_product(positive_values.begin(), positive_values.end(), positive_values.begin(), 0.0f);
    float std_dev = std::sqrt(sq_sum / positive_values.size() - mean * mean);
    return std_dev;
  }

  float calculateMin(const std::vector<float> values, bool negativeFree) {
    std::vector<float> positive_values;
    if(negativeFree){
      positive_values = values;
    }else{
      positive_values = removeNegativeValues(values);
    }
    return *std::min_element(positive_values.begin(), positive_values.end());
  }

  float calculateMax(const std::vector<float> values, bool negativeFree) {
    std::vector<float> positive_values;
    if(negativeFree){
      positive_values = values;
    }else{
      positive_values = removeNegativeValues(values);
    }
    return *std::max_element(positive_values.begin(), positive_values.end());
  }
}