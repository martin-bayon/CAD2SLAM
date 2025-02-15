#pragma once

#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"
#include "srrg_property/property_eigen.h"
#include "srrg_geometry/geometry_defs.h"
#include "srrg_boss/deserializer.h"

#include "cad2slam/data_structures/correspondence_anchor.h"

namespace cad2slam {
  using namespace srrg2_core;

  class ConfigMaster : public Configurable{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = ConfigMaster;
    using BaseType = Configurable;

    PARAM(PropertyEigen_<Vector2i>, target_resize_dim, "Target dimensions for the map managers image", Vector2i(0,0), nullptr);
    PARAM(PropertyString, initial_graph_filename, "Initial graph filename", "", nullptr);
    PARAM(PropertyString, augmented_graph_filename, "Augmented graph filename", "", nullptr);
    PARAM(PropertyString, optimized_graph_filename, "Optimized graph filename", "", nullptr);
    PARAM(PropertyString, correspondence_anchors_filename, "Correspondence anchors filename", "", nullptr);

    Vector2i getTargetResizeDim();
    std::string getInitialGraphFilename();
    std::string getAugmentedGraphFilename();
    std::string getOptimizedGraphFilename();
    std::string getCorrespondenceAnchorsFilename();

    std::vector<CorrespondenceAnchor> getCorrespondenceAnchors();
  };
}
