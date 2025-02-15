#include "lidar_simulator.h"

namespace cad2slam {

  void LidarSimulator::initialize() {
    if(!_initialized){
      checkParams();
      _initialized = true;
    }
  }

  void LidarSimulator::checkParams() {
    //TODO -- Do I need to check anything here?
    std::cerr << "[INFO] checkParams is successful" << std::endl;
  }

  std::vector<Vector2i> LidarSimulator::simulateLidarPixels(cv::Mat image, Vector2i sensor_pixel, float sensor_theta) {
    cv::Mat image_copy = image.clone();
    std::vector<Vector2i> pixels;

    float initial_angle = sensor_theta - param_fov.value() / 2;
    float angle_increment = param_fov.value()/param_num_measurements.value();

    for(int i=0; i<param_num_measurements.value(); i++) {
      float curr_angle = initial_angle + i * angle_increment;

      Vector2f direction(cos(-curr_angle), sin(-curr_angle));
      Vector2f curr_pixel(sensor_pixel.x(), sensor_pixel.y());

      for(float distance=0; distance<param_max_distance_p.value(); distance+=0.005f){
        curr_pixel += (direction * distance);

        if (curr_pixel.x() >= 0 && curr_pixel.y() >= 0 && curr_pixel.x() < image.cols && curr_pixel.y() < image.rows) {
          cv::Vec3b pixel_color = image.at<cv::Vec3b>(cv::Point(curr_pixel.x(), curr_pixel.y()));

          if (pixel_color != cv::Vec3b(255, 255, 255)) {
            pixels.push_back(curr_pixel.cast<int>());
            break;
          }
        }
      }
    }
    return pixels;
  }
}