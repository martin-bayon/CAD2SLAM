#pragma once

#include "srrg_config/configurable.h"
#include "srrg_pcl/point.h"
#include "cad2slam/map_manager/map_manager.h"

namespace cad2slam {
  using namespace srrg2_core;

  class LidarSimulator : public Configurable {
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      using ThisType = LidarSimulator;
      using BaseType = Configurable;

      PARAM(PropertyInt, num_measurements, "Number of measurements", 1080, nullptr);
      PARAM(PropertyFloat, fov, "Field of View", 360*CV_PI/180, nullptr);
      PARAM(PropertyFloat, max_distance_p, "Max distance (pixels)", 1000, nullptr);
      PARAM(PropertyFloat, max_distance_m, "Max distance (meters)", 15, nullptr);

      //TODO -- It does not make sense to have a max distance in pixels and in meters. Adapt it and make it map dependent

      std::vector<Vector2i> simulateLidarPixels(cv::Mat image, Vector2i sensor_pixel, float sensor_theta);

  protected:
      bool _initialized = false;

      void initialize();
      void checkParams();
  };
}