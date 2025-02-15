#include "map_manager.h"

#include <utility>

#include "srrg_config/configurable_command.h"

namespace cad2slam {

  void MapManager::initialize() {
    if(!_initialized){
      checkParams();
      loadYaml();
      loadImage();
      resizeImage();
      _initialized = true;
    }
  }

  void MapManager::checkParams() {
    if(param_yaml_path.value() == ""){
      throw std::runtime_error("[MapManager::checkParams - " + name() + "] yaml_file_path is not set - Exiting...");
    }
    if(!param_config_master.value()){
      throw std::runtime_error("[MapManager::checkParams - " + name() + "] config_master is not set - Exiting...");
    }
  }

  void MapManager::loadYaml() {
    _yaml_data = readYaml(param_yaml_path.value());
  }

  void MapManager::loadImage() {
    _original_image = cv::imread(_yaml_data.image_path, cv::IMREAD_COLOR);
    if(_original_image.empty()){
      throw std::runtime_error("[MapManager::loadImage] Could not open or find the image [" + _yaml_data.image_path + "] - Exiting...");
    }
    _original_image_dim = Vector2i(_original_image.cols, _original_image.rows);
  }

  void MapManager::resizeImage() {
    Vector2i target_resize_dim = param_config_master->getTargetResizeDim();
    if(_original_image_dim.x() == target_resize_dim.x() && _original_image_dim.y() == target_resize_dim.y()){
      _resized_image = _original_image;
      _resized_image_dim = _original_image_dim;
      _resize_scale = 1.0;
      return;
    }

    double original_aspect_ratio = static_cast<double>(_original_image_dim.x()) / static_cast<double>(_original_image_dim.y());
    int resized_x = std::min(target_resize_dim.x(), static_cast<int>(target_resize_dim.y() * original_aspect_ratio));
    int resized_y = std::min(target_resize_dim.y(), static_cast<int>(target_resize_dim.x() / original_aspect_ratio));
    _resized_image_dim = Vector2i(resized_x, resized_y);
    cv::Size resize_dim(resized_x, resized_y);
    cv::resize(_original_image, _resized_image, resize_dim);
    _resize_scale = (static_cast<float>(_original_image_dim.x())/_resized_image_dim.x() + static_cast<float>(_original_image_dim.y())/_resized_image_dim.y()) / 2;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////

  std::string MapManager::getImagePath(){
    initialize();
    return _yaml_data.image_path;
  }

  cv::Mat MapManager::getImage() {
    initialize();
    return _original_image;
  }

  cv::Mat MapManager::getResizedImage() {
    initialize();
    return _resized_image;
  }

  Vector2i MapManager::getImageDim() {
    initialize();
    return _original_image_dim;
  }

  Vector2i MapManager::getResizedImageDim() {
    initialize();
    return _resized_image_dim;
  }

  Vector2f MapManager::pixel2Point(Vector2i pixel) {
    initialize();
    return Vector2f(pixel.x()*_yaml_data.resolution, -pixel.y()*_yaml_data.resolution);
  }

  Vector2i MapManager::point2Pixel(Vector2f point) {
    initialize();
    return Vector2i(point.x() / _yaml_data.resolution, -point.y() / _yaml_data.resolution);
  }

  Vector2i MapManager::resized2original(Vector2i resized_pixel) {
    initialize();
    Vector2i original_pixel = (resized_pixel.cast<float>() * _resize_scale).cast<int>();
    return original_pixel;
  }

  Vector2i MapManager::original2resized(Vector2i original_pixel) {
    initialize();
    Vector2i resized_pixel = (original_pixel.cast<float>() / _resize_scale).cast<int>();
    return resized_pixel;

  }
}