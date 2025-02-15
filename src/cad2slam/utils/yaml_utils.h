#pragma once

#include "srrg_geometry/geometry_defs.h"
#include "boost/filesystem.hpp"
#include "yaml-cpp/yaml.h"


namespace cad2slam {
  using namespace srrg2_core;

  enum YamlMode {trinary=0, scale=1, raw=2};

  //According to the ROS Map Server package definition of YAML file for a occupancy grid map
  //https://wiki.ros.org/map_server
  struct YamlData {
    std::string yaml_path = "";               //Path of the YAML file
    std::string image_path = "";              //Path to the occupancy grid image
    float resolution = 1.0;                   //Resolution of the map (meters/pixel)
    Vector3f origin = Vector3f::Identity();   //2D pose of the lower-left pixel in the map. Counterclockwise rotation
    float occupied_thresh = 1.0;              //Upper threshold to consider a pixel as completely occupied
    float free_thresh = 0.0;                  //Lower threshold to consider a pixel as completely free
    int negate = 0;                           //Whether the free/occupied semantics should be reserved
    YamlMode mode = YamlMode::scale;          //Affects the value interpretation of a pixel
  };

  template<typename T>
  void operator >> (const YAML::Node& node, T& i){
    i = node.as<T>();
  }

  YamlData readYaml(std::string yaml_path_);

}
