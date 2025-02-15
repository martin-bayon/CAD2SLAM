#include "graph_builder.h"

#include <srrg_config/configurable_command.h>

namespace cad2slam{

  GraphBuilder::GraphBuilder() {
    addCommand(new ConfigurableCommand_<GraphBuilder, typeof(&GraphBuilder::cmdSaveInitialGraph), std::string>(
            this,
            "saveInitialGraph",
            "Saves the initial graph generated from the cad blueprint to a file",
            &GraphBuilder::cmdSaveInitialGraph));

    addCommand(new ConfigurableCommand_<GraphBuilder, typeof(&GraphBuilder::cmdSaveAugmentedGraph), std::string>(
            this,
            "saveAugmentedGraph",
            "Saves the augmented graph with the correspondence_anchors factors to a file",
            &GraphBuilder::cmdSaveAugmentedGraph));
  }

  void GraphBuilder::initialize() {
    if(!_is_initialized){
      _graph = FactorGraphPtr(new FactorGraph());
      checkParams();
      _is_initialized = true;
    }
  }

  void GraphBuilder::checkParams() {
    if(!param_config_master.value()) {
      throw std::runtime_error("[GraphBuilder::checkParams] config_master is not set - Exiting...");
    }
    if(!param_cad_map_manager.value()) {
      throw std::runtime_error("[GraphBuilder::checkParams] cad_map_manager is not set - Exiting...");
    }
    if(param_cell_size.value() == 0){
      throw std::runtime_error("[GraphBuilder::checkParams] cell_size cannot be 0 - Exiting...");
    }
    if(!param_use_horizontal_and_vertical_factors.value() && !param_use_diagonal_factors.value()){
      throw std::runtime_error("[GraphBuilder::checkParams] both horizontal & vertical and diagonal factors are disabled - Exiting...");
    }
  }

  void GraphBuilder::generateInitialGraph() {
    if(!_is_initial_graph_ready){
      generateVariables();
      generateK2KFactors();
      _is_initial_graph_ready = true;
    }
  }

  void GraphBuilder::generateVariables() {
    KeyframeSE2Ptr curr_variable;
    Vector2i grid_dim = calculateGridDim();
    Vector2i roi_index;
    Vector2i roi_center_pixel;
    Vector2f roi_center_pose;
    cv::Rect roi_rect;
    cv::Mat roi_image_window;

    for(int i=0; i<grid_dim.x(); i++){
      for(int j=0; j<grid_dim.y(); j++){
        roi_index = Vector2i(i,j);
        roi_center_pixel = roiIndex2RoiCenterPixel(roi_index);
        roi_center_pose = param_cad_map_manager->pixel2Point(roi_center_pixel);
        roi_rect = pixel2RoiRect(roi_center_pixel);
        roi_image_window = param_cad_map_manager->getImage()(roi_rect);

        if(containsEmptySpace(roi_image_window)){
          curr_variable = KeyframeSE2Ptr(new KeyframeSE2());
          curr_variable->setCadPoint(roi_center_pose);
          curr_variable->setRoiIndex(roi_index);

          _graph->addVariable(curr_variable);
          _roi_index2graph_id.insert(std::make_pair(roi_index, curr_variable->graphId()));
        }
      }
    }
  }

  void GraphBuilder::generateK2KFactors(){
    FactorKeyframe2KeyframePtr curr_factor;
    std::vector<Vector2i> neighbour_indices;
    int graph_id0;
    int graph_id1;
    KeyframeSE2* var0;
    KeyframeSE2* var1;

    for(auto curr_it : _graph->variables()){
      auto curr_var = dynamic_cast<KeyframeSE2*>(curr_it.second);
      neighbour_indices = getNeighbourIndices(curr_var->getRoiIndex());
      bool missing_neighbour = false;
      for(auto neighbour_index : neighbour_indices){
        auto other_it = _roi_index2graph_id.find(neighbour_index);
        if(other_it == _roi_index2graph_id.end()){
          missing_neighbour = true;
          continue;
        }

        if(curr_var->graphId() < other_it->second){
          graph_id0 = curr_var->graphId();
          graph_id1 = other_it->second;
          var0 = curr_var;
          var1 = dynamic_cast<KeyframeSE2*>(_graph->variable(other_it->second));
        }else{
          graph_id0 = other_it->second;
          graph_id1 = curr_var->graphId();
          var0 = dynamic_cast<KeyframeSE2*>(_graph->variable(other_it->second));
          var1 = curr_var;
        }

        auto var_indices_pair = std::make_pair(graph_id0, graph_id1);
        if(_factors_variables_indices.find(var_indices_pair) == _factors_variables_indices.end()){
          _factors_variables_indices.insert(var_indices_pair);
          auto second_from_first = var0->estimate().inverse() * var1->estimate();

          curr_factor = FactorKeyframe2KeyframePtr(new FactorKeyframe2Keyframe());
          curr_factor->setVariableId(0, graph_id0);
          curr_factor->setVariableId(1, graph_id1);
          curr_factor->setMeasurement(second_from_first);

          _graph->addFactor(curr_factor);
        }
      }

      if(!param_use_horizontal_and_vertical_factors.value() || !param_use_diagonal_factors.value() || !missing_neighbour){
        std::vector<Vector2i> all_neighbour_indices = getAllNeighbourIndices(curr_var->getRoiIndex());
        for (auto neighbour_index : all_neighbour_indices){
          auto it = _roi_index2graph_id.find(neighbour_index);
          if(it == _roi_index2graph_id.end()){
            missing_neighbour = true;
            break;
          }
        }
      }
      if(missing_neighbour){
        curr_var->setOuter(true);
      }
    }
  }

  void GraphBuilder::augmentGraph(){
    //TODO -- Will have to check what happen when trying to augment the graph online - i.e. adding new anchors and optimizing at the same time
    if(!_is_augmented_graph_ready){
      auto correspondenceAnchors = param_config_master->getCorrespondenceAnchors();
      for(const auto& anchor_pair : correspondenceAnchors) {
        addAnchorFactor(anchor_pair);
      }
      _is_augmented_graph_ready = true;
    }
  }

  void GraphBuilder::addAnchorFactor(CorrespondenceAnchor anchor_pair){
    FactorKeyframe2AnchorPtr curr_factor;
    auto closest_keyframe = closestCadPoseKeyframe(anchor_pair.getCadPoint(), _graph);
    auto anchor_cad_point_from_keyframe = anchor_pair.getCadPoint() - closest_keyframe->getCadPoint();
    curr_factor = FactorKeyframe2AnchorPtr(new FactorKeyframe2Anchor());
    curr_factor->setVariableId(0, closest_keyframe->graphId());
    curr_factor->setSlamPoint(anchor_pair.getSlamPoint());
    curr_factor->setMeasurement(anchor_cad_point_from_keyframe);
    _graph->addFactor(curr_factor);
  }

  Vector2i GraphBuilder::calculateGridDim(){
    Vector2i image_dim = param_cad_map_manager->getImageDim();
    Vector2f grid_dim = image_dim.cast<float>() / static_cast<float>(param_cell_size.value());
    grid_dim = grid_dim.array().ceil();
    return grid_dim.cast<int>();
  }

  Vector2i GraphBuilder::roiIndex2RoiCenterPixel(srrg2_core::Vector2i roi_index) {
    return roi_index * param_cell_size.value() + Vector2i::Constant(param_cell_size.value()/2);
  }

  cv::Rect GraphBuilder::pixel2RoiRect(srrg2_core::Vector2i pixel) {
    int cell_size = param_cell_size.value();
    Vector2i image_dims = param_cad_map_manager->getImageDim();
    int x = std::max(0, std::min(pixel.x() - cell_size/2, image_dims.x() - cell_size));
    int y = std::max(0, std::min(pixel.y() - cell_size/2, image_dims.y() - cell_size));
    return cv::Rect(x, y, cell_size, cell_size);
  }

  std::vector<Vector2i> GraphBuilder::getNeighbourIndices(Vector2i roi_index) {
    std::vector<Vector2i> neighbours_indices;

    if(param_use_horizontal_and_vertical_factors.value()){
      neighbours_indices.push_back(Vector2i(roi_index.x() - 1, roi_index.y()));
      neighbours_indices.push_back(Vector2i(roi_index.x(), roi_index.y() - 1));
      neighbours_indices.push_back(Vector2i(roi_index.x(), roi_index.y() + 1));
      neighbours_indices.push_back(Vector2i(roi_index.x() + 1, roi_index.y()));
    }
    if(param_use_diagonal_factors.value()){
      neighbours_indices.push_back(Vector2i(roi_index.x() - 1, roi_index.y() - 1));
      neighbours_indices.push_back(Vector2i(roi_index.x() - 1, roi_index.y() + 1));
      neighbours_indices.push_back(Vector2i(roi_index.x() + 1, roi_index.y() - 1));
      neighbours_indices.push_back(Vector2i(roi_index.x() + 1, roi_index.y() + 1));
    }
    return neighbours_indices;
  }

  std::vector<Vector2i> GraphBuilder::getAllNeighbourIndices(Vector2i roi_index) {
    std::vector<Vector2i> neighbours_indices;
    neighbours_indices.push_back(Vector2i(roi_index.x() - 1, roi_index.y()));
    neighbours_indices.push_back(Vector2i(roi_index.x(), roi_index.y() - 1));
    neighbours_indices.push_back(Vector2i(roi_index.x(), roi_index.y() + 1));
    neighbours_indices.push_back(Vector2i(roi_index.x() + 1, roi_index.y()));
    neighbours_indices.push_back(Vector2i(roi_index.x() - 1, roi_index.y() - 1));
    neighbours_indices.push_back(Vector2i(roi_index.x() - 1, roi_index.y() + 1));
    neighbours_indices.push_back(Vector2i(roi_index.x() + 1, roi_index.y() - 1));
    neighbours_indices.push_back(Vector2i(roi_index.x() + 1, roi_index.y() + 1));
    return neighbours_indices;
  }

  bool GraphBuilder::cmdSaveInitialGraph(std::string &response) {
    saveInitialGraph();
    return true;
  }

  bool GraphBuilder::cmdSaveAugmentedGraph(std::string &response) {
    saveAugmentedGraph();
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////

  void GraphBuilder::saveInitialGraph() {
    initialize();
    generateInitialGraph();
    _graph->write(param_config_master->getInitialGraphFilename());
  }

  void GraphBuilder::saveAugmentedGraph() {
    initialize();
    generateInitialGraph();
    augmentGraph();
    _graph->write(param_config_master->getAugmentedGraphFilename());
  }

  FactorGraphPtr GraphBuilder::getInitialGraph() {
    initialize();
    generateInitialGraph();
    return _graph;
  }

  FactorGraphPtr GraphBuilder::getAugmentedGraph(){
    initialize();
    generateInitialGraph();
    augmentGraph();
    return _graph;
  }
}