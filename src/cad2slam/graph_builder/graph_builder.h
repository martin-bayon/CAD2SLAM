#pragma once

#include "srrg_config/configurable.h"
#include "srrg_solver/solver_core/factor_graph.h"

#include "cad2slam/config_master/config_master.h"
#include "cad2slam/map_manager/map_manager.h"
#include "cad2slam/variables_and_factors/all_types.h"
#include "cad2slam/utils/image_utils.h"
#include "cad2slam/utils/comparator_utils.h"
#include "cad2slam/utils/graph_utils.h"
#include "cad2slam/data_structures/correspondence_anchor.h"

namespace cad2slam {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  class GraphBuilder : public Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = GraphBuilder;
    using BaseType = Configurable;

    PARAM(PropertyConfigurable_<ConfigMaster>, config_master, "Config master", nullptr, nullptr);
    PARAM(PropertyConfigurable_<MapManager>, cad_map_manager, "Cad map manager", nullptr, nullptr);
    PARAM(PropertyBool, use_horizontal_and_vertical_factors, "Generate horizontal & vertical factors", true, nullptr);
    PARAM(PropertyBool, use_diagonal_factors, "Generate diagonal factors", false, nullptr);
    PARAM(PropertyInt, cell_size, "Cell size", 0, nullptr);

    GraphBuilder();

    void saveInitialGraph();
    void saveAugmentedGraph();
    FactorGraphPtr getInitialGraph();
    FactorGraphPtr getAugmentedGraph();

  protected:
    bool cmdSaveInitialGraph(std::string& response);
    bool cmdSaveAugmentedGraph(std::string& response);

    void initialize();
    void checkParams();
    void generateInitialGraph();
    void generateVariables();
    void generateK2KFactors();
    void augmentGraph();
    void addAnchorFactor(CorrespondenceAnchor anchor);

    Vector2i calculateGridDim();
    Vector2i roiIndex2RoiCenterPixel(Vector2i roi_index);
    cv::Rect pixel2RoiRect(Vector2i pixel);
    std::vector<Vector2i> getNeighbourIndices(Vector2i roi_index);
    std::vector<Vector2i> getAllNeighbourIndices(Vector2i roi_index);


    bool _is_initialized = false;
    bool _is_initial_graph_ready = false;
    bool _is_augmented_graph_ready = false;
    FactorGraphPtr _graph;
    std::map<Vector2i, int, Vector2iComparator> _roi_index2graph_id;
    std::set<std::pair<int,int>> _factors_variables_indices;
  };
}