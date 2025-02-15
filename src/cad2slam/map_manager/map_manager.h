#pragma once

#include "srrg_config/configurable.h"
#include "srrg_geometry/geometry_defs.h"
#include "opencv2/opencv.hpp"

#include "cad2slam/utils/yaml_utils.h"

#include "cad2slam/config_master/config_master.h"

namespace cad2slam {
  using namespace srrg2_core;

  class MapManager : public Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = MapManager;
    using BaseType = Configurable;

    PARAM(PropertyConfigurable_<ConfigMaster> , config_master, "Config master", nullptr, nullptr);
    PARAM(PropertyString, yaml_path, "YAML file path", "", nullptr);
    PARAM(PropertyString, window_name, "Window name", "map", nullptr);

    std::string getImagePath();

    cv::Mat getImage();
    cv::Mat getResizedImage();
    Vector2i getImageDim();
    Vector2i getResizedImageDim();

    Vector2f pixel2Point(Vector2i pixel);
    Vector2i point2Pixel(Vector2f point);

    Vector2i resized2original(Vector2i resized_pixel);
    Vector2i original2resized(Vector2i original_pixel);

  protected:
    void initialize();
    void checkParams();
    void loadYaml();
    void loadImage();
    void resizeImage();


    bool _initialized = false;
    YamlData _yaml_data;

    cv::Mat _original_image;
    cv::Mat _resized_image;

    Vector2i _original_image_dim;
    Vector2i _resized_image_dim;
    float _resize_scale;
  };
}