#include <iostream>
#include "graph_optimizer.h"
#include "srrg_config/configurable_command.h"

namespace cad2slam {

  GraphOptimizer::GraphOptimizer() {
    addCommand(new ConfigurableCommand_<GraphOptimizer, typeof(&GraphOptimizer::cmdOptimize), std::string>(
            this,
            "optimize",
            "Optimizes the graph",
            &GraphOptimizer::cmdOptimize));

    addCommand(new ConfigurableCommand_<GraphOptimizer, typeof(&GraphOptimizer::cmdLoadGraph), std::string>(
            this,
            "loadGraph",
            "Loads a graph from a file",
            &GraphOptimizer::cmdLoadGraph));

    addCommand(new ConfigurableCommand_<GraphOptimizer, typeof(&GraphOptimizer::cmdSaveGraph), std::string>(
            this,
            "saveGraph",
            "Saves the graph from a file",
            &GraphOptimizer::cmdSaveGraph));
  };

  void GraphOptimizer::initialize() {
    if(!_initialized){
      checkParams();
      _initialized = true;
    }
  }

  void GraphOptimizer::checkParams() {
    if(!param_config_master.value()) {
      throw std::runtime_error("[GraphOptimizer::checkParams] config_master is not set - Exiting...");
    }
    if(!param_solver.value()) {
      throw std::runtime_error("[GraphOptimizer::checkParams] solver is not set - Exiting...");
    }
    if(param_solver->param_max_iterations.size() != 0){
      throw std::runtime_error("[GraphOptimizer::checkParams] solver.param_max_iterations should be empty. Please, set the iterations in the GraphOptimizer param instead and remove it from the Solver configurable - Exiting...");
    }
    if(param_max_iterations.value() == 0){
      throw std::runtime_error("[GraphOptimizer::checkParams] max_iterations cannot be 0 - Exiting...");
    }
    std::cerr << "[INFO] checkParams is successful" << std::endl;
  }

  void GraphOptimizer::checkGraph() {
    if(!_is_graph_ready){
      throw std::runtime_error("[GraphOptimizer::checkGraph] graph is not ready, please load the graph before trying to use it - Exiting...");
    }
  }

  void GraphOptimizer::assignInfoMatrices() {
    bool update_k2k = true;
    bool update_k2a = true;

    auto k2k_directional_info = param_directional_k2k_info_matrix.value();
    auto k2a_info = param_k2a_info_matrix.value();

    if(k2k_directional_info == Matrix3f::Identity()) {
      std::cerr << "[INFO] The k2k_info_matrix param is set to the Identity. Keyframe2Keyframe information matrices will not be updated" << std::endl;
      update_k2k = false;
    }
    if(k2a_info == Matrix2f::Identity()){
      std::cerr << "[INFO] The k2a_info_matrix param is set to the Identity. Keyframe2Anchor information matrices will not be updated" << std::endl;
      update_k2a = false;
    }

    for(auto it : _graph->factors()) {
      auto k2k_factor = dynamic_cast<FactorKeyframe2Keyframe*>(it.second);
      if(k2k_factor && update_k2k){
        Matrix3f omega = Matrix3f::Identity();
        auto R_ij = k2k_factor->rotationMatrix();
        auto lambda_xy = k2k_directional_info.block<2,2>(0, 0);
        omega.block<2,2>(0, 0) = R_ij * lambda_xy * R_ij.transpose();
        omega(2, 2) = k2k_directional_info(2, 2);

        if(k2k_factor->isOuter() && param_outer_factor_gain.value() >= 0){
          omega *= param_outer_factor_gain.value();
        }
        k2k_factor->setInformationMatrix(omega);
      }

      auto k2a_factor = dynamic_cast<FactorKeyframe2Anchor*>(it.second);
      if(k2a_factor && update_k2a) {
        k2a_factor->setInformationMatrix(k2a_info);
      }
    }
  }

  bool GraphOptimizer::cmdOptimize(std::string &response) {
    optimize();
    return true;
  }

  bool GraphOptimizer::cmdLoadGraph(std::string &response){
    loadGraph();
    return true;
  }

  bool GraphOptimizer::cmdSaveGraph(std::string &response) {
    saveGraph();
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////

  void GraphOptimizer::loadGraph(){
    initialize();
    _graph = FactorGraph::read(param_config_master->getAugmentedGraphFilename());
    if(!_graph){
      throw std::runtime_error("[GraphOptimizer::loadGraph] Could not load the graph from the file indicated in the Config master - Exiting...");
    }
    _is_graph_ready = true;
    _is_graph_optimized = false;
  }

  void GraphOptimizer::setGraph(FactorGraphPtr graph_){
    initialize();
    _graph = std::move(graph_);
    _is_graph_ready = true;
    _is_graph_optimized = false;
  }

  void GraphOptimizer::optimize(){
    initialize();
    checkGraph();
    assignInfoMatrices();

    param_solver->setGraph(_graph);
    param_solver->param_max_iterations.pushBack(1);

    if(param_debug_visualization.value()) {
      this->_need_redraw = true;
      ActiveDrawable::draw();
      usleep(5000000);
    }

    for(int i=0; i<param_max_iterations.value(); i++){
      param_solver->compute();
      auto stats = param_solver->lastIterationStats();
      std::cerr << stats << std::endl;
      if(param_debug_visualization.value()) {
        this->_need_redraw = true;
        ActiveDrawable::draw();
        usleep(500000);
      }
    }
    _is_graph_optimized = true;
  }

  FactorGraphPtr GraphOptimizer::getGraph() {
    if(!_is_graph_optimized){
      throw std::runtime_error("[GraphOptimizer::getGraph] The graph has not been optimized since it was loaded - Exiting...");
    }
    return _graph;
  }

  void GraphOptimizer::saveGraph() {
    if(!_is_graph_optimized){
      throw std::runtime_error("[GraphOptimizer::saveGraph] The graph has not been optimized since it was loaded - Exiting...");
    }
    _graph->write(param_config_master->getOptimizedGraphFilename());
  }

  void GraphOptimizer::_drawImpl(srrg2_core::ViewerCanvasPtr canvas) const {
    if (!canvas) {
      return;
    }
    for (const auto& it : _graph->variables()) {
      it.second->_drawImpl(canvas);
    }
    for (const auto& it : _graph->factors()) {
      it.second->_drawImpl(canvas);
    }
    canvas->flush();
  }
}