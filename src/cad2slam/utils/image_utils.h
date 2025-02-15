#pragma once

#include "opencv2/opencv.hpp"

namespace cad2slam{
  bool containsEmptySpace(cv::Mat image);

  const int hue_range = 180;
  const int total_colors = 10;
  cv::Scalar index2hueValue(int index);
}
