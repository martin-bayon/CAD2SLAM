#pragma once

#include <unistd.h>

#include "srrg_config/configurable.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_viewer/active_drawable.h"

#include "cad2slam/config_master/config_master.h"
#include "cad2slam/variables_and_factors/all_types.h"

namespace cad2slam {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  class GraphOptimizer : public Configurable, public ActiveDrawable{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = GraphOptimizer;
    using BaseType = Configurable;

    PARAM(PropertyConfigurable_<ConfigMaster>, config_master, "Config master", nullptr, nullptr);
    PARAM(PropertyConfigurable_<Solver>, solver, "Solver", nullptr, nullptr);

    PARAM(PropertyInt, max_iterations, "Max iterations", 10, nullptr);
    PARAM(PropertyEigen_<Matrix3f>, directional_k2k_info_matrix, "Keyframe2Keyframe Information matrix to enable direction aware factors", Matrix3f::Identity(), nullptr);
    PARAM(PropertyEigen_<Matrix2f>, k2a_info_matrix, "Keyframe2Anchor Information matrix", Matrix2f::Identity(), nullptr);
    PARAM(PropertyBool, debug_visualization, "Debug visualization", false, nullptr);
    PARAM(PropertyFloat, outer_factor_gain, "Outer factor gain", -1, nullptr);

    GraphOptimizer();

    void loadGraph();
    void setGraph(FactorGraphPtr graph_);
    void optimize();

    void saveGraph();
    FactorGraphPtr getGraph();

    void _drawImpl(ViewerCanvasPtr canvas) const override;

  protected:
    bool cmdOptimize(std::string& response);
    bool cmdLoadGraph(string &response);
    bool cmdSaveGraph(string &response);

    void initialize();
    void checkParams();
    void checkGraph();

    void assignInfoMatrices();

    FactorGraphPtr _graph;
    bool _initialized = false;
    bool _is_graph_ready = false;
    bool _is_graph_optimized = false;
  };

}