#pragma once

#include "srrg_solver/solver_core/factor_graph.h"
#include "cad2slam/variables_and_factors/all_types.h"


namespace cad2slam {
  using namespace srrg2_solver;

  KeyframeSE2* closestCadPoseKeyframe(Vector2f point, FactorGraphPtr graph);
  KeyframeSE2* closestSlamPoseKeyframe(Vector2f point, FactorGraphPtr graph);


  std::vector<std::pair<float, KeyframeSE2*>> neighborhoodCadPoseKeyframe(Vector2f point, FactorGraphPtr graph, float threshold);
  std::vector<std::pair<float, KeyframeSE2*>> neighborhoodCadPoseKeyframe(Vector2f point, FactorGraphPtr graph, int k);
}
