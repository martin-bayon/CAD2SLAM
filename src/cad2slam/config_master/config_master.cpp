#include "config_master.h"

namespace cad2slam {

  Vector2i ConfigMaster::getTargetResizeDim() {
    if(param_target_resize_dim.value() == Vector2i(0,0)){
      throw std::runtime_error("[ConfigMaster::getTargetResizeDim] target_resize_dim cannot be (0,0) - Exiting...");
    }
    return param_target_resize_dim.value();
  }

  std::string ConfigMaster::getInitialGraphFilename() {
    if(param_initial_graph_filename.value().empty()){
      throw std::runtime_error("[ConfigMaster::getInitialGraphFilename] initial_graph_filename is not set - Exiting...");
    }
    return param_initial_graph_filename.value();
  }

  std::string ConfigMaster::getAugmentedGraphFilename() {
    if(param_augmented_graph_filename.value().empty()){
      throw std::runtime_error("[ConfigMaster::getAugmentedGraphFilename] augmented_graph_filename is not set - Exiting...");
    }
    return param_augmented_graph_filename.value();
  }

  std::string ConfigMaster::getOptimizedGraphFilename() {
    if(param_optimized_graph_filename.value().empty()){
      throw std::runtime_error("[ConfigMaster::getOptimizedGraphFilename] optimized_graph_filename is not set - Exiting...");
    }
    return param_optimized_graph_filename.value();
  }

  std::string ConfigMaster::getCorrespondenceAnchorsFilename() {
    if(param_correspondence_anchors_filename.value().empty()){
      throw std::runtime_error("[ConfigMaster::getCorrespondenceAnchorsFilename] correspondence_anchors_filename is not set - Exiting...");
    }
    return param_correspondence_anchors_filename.value();
  }

  std::vector<CorrespondenceAnchor> ConfigMaster::getCorrespondenceAnchors() {
    std::vector<CorrespondenceAnchor> correspondenceAnchors;
    std::string correspondenceAnchorsFilename = getCorrespondenceAnchorsFilename();
    Deserializer des;
    SerializablePtr o;
    des.setFilePath(correspondenceAnchorsFilename);
    while((o=des.readObjectShared())){
      auto correspondence = std::dynamic_pointer_cast<CorrespondenceAnchor>(o);
      if(correspondence){
        correspondenceAnchors.push_back(*correspondence);
      }
    }
    if(correspondenceAnchors.empty()){
      throw std::runtime_error("[ConfigMaster::getCorrespondenceAnchors] the list of correspondence anchors is empty. Maybe the input file was empty - Exiting...");
    }
    return correspondenceAnchors;
  }
}