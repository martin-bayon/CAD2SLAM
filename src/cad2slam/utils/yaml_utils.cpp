#include "yaml_utils.h"

namespace cad2slam {

  //mb - TODO -- Up to this moment, we only read the image and resolution tags of the YAML because those are the only ones this package use
  YamlData readYaml(std::string yaml_path_){
    std::ifstream fin(yaml_path_);
    if(fin.fail()){
      throw std::runtime_error("[YamlUtils::readYaml] Cannot open file [" + yaml_path_ + "] - Exiting...");
    }

    std::string image_tag = "image";
    std::string resolution_tag = "resolution";

    YamlData yaml_data;
    yaml_data.yaml_path = yaml_path_;

    YAML::Node doc = YAML::Load(fin);

    try{
      std::string image_path;
      doc[image_tag] >> image_path;

      boost::filesystem::path mapfpath(image_path);
      if (mapfpath.is_absolute()) {
        yaml_data.image_path = mapfpath.string();
      }else{
        boost::filesystem::path current_dir = boost::filesystem::current_path();
        boost::filesystem::path yaml_dir(yaml_path_);
        boost::filesystem::path abs_path = current_dir / yaml_dir.parent_path() / mapfpath;
        yaml_data.image_path = abs_path.string();
      }
    }catch(YAML::Exception &){
      throw std::runtime_error("[YamlUtils::readYaml] The YAML file does not contain the tag [" + image_tag + "] or it is invalid - Exiting...");
    }

    try{
      doc[resolution_tag] >> yaml_data.resolution;
    }catch(YAML::Exception &){
      throw std::runtime_error("[YamlUtils::readYaml] The YAML file does not contain the tag [" + resolution_tag + "] or it is invalid - Exiting...");
    }

    return yaml_data;
  }
}