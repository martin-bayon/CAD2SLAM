#include "graph_utils.h"

namespace cad2slam {

  KeyframeSE2* closestCadPoseKeyframe(Vector2f point, FactorGraphPtr graph){
    //TODO -- In the future, maybe implement a distance map to speed up association
    float closest_distance = std::numeric_limits<float>::max();
    KeyframeSE2* closest_keyframe;
    for(auto it : graph->variables()){
      auto var = dynamic_cast<KeyframeSE2*>(it.second);
      if(var){
        float distance = (var->getCadPoint() - point).norm();
        if(distance < closest_distance){
          closest_distance = distance;
          closest_keyframe = var;
        }
      }
    }
    if(!closest_keyframe){
      throw std::runtime_error("[closestCadPoseKeyframe] Could not find the closest keyframe variable for the point " + std::to_string(point.x()) + ", " + std::to_string(point.y()) + " - Exiting...");
    }
    return closest_keyframe;
  }

  std::vector<std::pair<float, KeyframeSE2*>>neighborhoodCadPoseKeyframe(Vector2f point, FactorGraphPtr graph, float threshold){
    std::vector<std::pair<float, KeyframeSE2*>> neighboringKeyframes;
    for(auto it : graph->variables()){
      auto var = dynamic_cast<KeyframeSE2*>(it.second);
      if(var){
        float distance = (var->getCadPoint() - point).norm();
        if(distance < threshold){
          neighboringKeyframes.emplace_back(distance, var);
        }
      }
    }
    if (neighboringKeyframes.empty()) {
      throw std::runtime_error("[neighborhoodCadPoseKeyframe] No keyframes found for the point " + std::to_string(point.x()) + ", " + std::to_string(point.y()) + " - Exiting...");
    }
    return neighboringKeyframes;
  }

  std::vector<std::pair<float, KeyframeSE2*>>neighborhoodCadPoseKeyframe(Vector2f point, FactorGraphPtr graph, int k){
    //TODO -- In the future, maybe implement a distance map to speed up association

    std::vector<std::pair<float, KeyframeSE2*>> distances;
    for(auto it : graph->variables()){
      auto var = dynamic_cast<KeyframeSE2*>(it.second);
      if(var){
        float distance = (var->getCadPoint() - point).norm();
        distances.emplace_back(distance, var);
      }
    }
    if (distances.empty()) {
      throw std::runtime_error("[neighborhoodCadPoseKeyframe] No keyframes found for the point " + std::to_string(point.x()) + ", " + std::to_string(point.y()) + " - Exiting...");
    }

    std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });

    std::vector<std::pair<float, KeyframeSE2*>> neighboringKeyframes;
    for (size_t i = 0; i < std::min<size_t>(k, distances.size()); ++i) {
      neighboringKeyframes.emplace_back(distances[i].first, distances[i].second);
    }

    if(neighboringKeyframes[0].first == 0){
      return {{1, neighboringKeyframes[0].second}};
    }
    return neighboringKeyframes;
  }



  KeyframeSE2* closestSlamPoseKeyframe(Vector2f point, FactorGraphPtr graph){
    //TODO -- In the future, maybe implement a distance map to speed up association
    float closest_distance = std::numeric_limits<float>::max();
    KeyframeSE2* closest_keyframe;
    for(auto it : graph->variables()){
      auto var = dynamic_cast<KeyframeSE2*>(it.second);
      if(var){
        //TODO -- Check
        float distance = (var->getSlamPoint() - point).norm();
        if(distance < closest_distance){
          closest_distance = distance;
          closest_keyframe = var;
        }
      }
    }
    if(!closest_keyframe){
      throw std::runtime_error("[closestSlamPoseKeyframe] Could not find the closest keyframe variable for the point " + std::to_string(point.x()) + ", " + std::to_string(point.y()) + " - Exiting...");
    }
    return closest_keyframe;
  }
}