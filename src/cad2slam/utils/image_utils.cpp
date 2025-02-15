#include "image_utils.h"

namespace cad2slam{
  bool containsEmptySpace(cv::Mat image){
    cv::Vec3b white(255, 255, 255);
    for(int i=0; i<image.rows; i++) {
      for (int j = 0; j < image.cols; j++) {
        cv::Vec3b curr_color = image.at<cv::Vec3b>(i, j);
        if (curr_color == white) {
          return true;
        }
      }
    }
    return false;
  }

  cv::Scalar index2hueValue(int index){
    index = index % total_colors;
    int hue = index * hue_range/total_colors;

    cv::Mat color(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
    cv::cvtColor(color, color, cv::COLOR_HSV2BGR);
    cv::Scalar result(color.at<cv::Vec3b>(0, 0)[0], color.at<cv::Vec3b>(0, 0)[1], color.at<cv::Vec3b>(0, 0)[2]);
    return result;
  }
}