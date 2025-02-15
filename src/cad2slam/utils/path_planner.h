#pragma once

#include <srrg_data_structures/path_matrix_distance_search.h>
#include "srrg2_navigation_2d/planner_2d.h"
#include "srrg_system_utils/system_utils.h"

using namespace srrg2_navigation_2d;
using namespace srrg2_core;

namespace cad2slam {
  using namespace srrg2_navigation_2d;
  using namespace srrg2_core;

  class PathPlanner {
  public:
      std::vector<Vector2i> doPathPlanning(Vector2i start_pixel, Vector2i goal_pixel, std::string yaml_filename);

  protected:
      PathMatrixDijkstraSearch _dijkstra_search;

      float _occupancy_threshold = 0.51;
      float _free_threshold = 0.49;
      float max_point_distance = 30.0f;
      float robot_radius = 0.2f;
      float grid_gain = 20.0;
      std::string occupancy_layer = "occupancy";
      std::string distance_layer = "distances";

      srrg2_core::GridMap2DPtr _grid_map = nullptr;
      float _resolution, _inverse_resolution;
      srrg2_core::Matrix_<uint8_t> _map;
      srrg2_core::Point2iVectorCloud _obstacles;
      srrg2_core::PathMatrix _distance_map;
      PathMatrix _global_pmap;
      Vector2fVector _free_cells;
      Vector3f _current_robot_pose;
      Vector3f _goal;

      Vector2i _robot_pose_pxl;
      Vector2i _goal_pose_pxl;

      StdVectorEigenVector3f _path;
      std::vector<Vector2i> _px_path;

      cv::Mat distance_cv;
      cv::Mat parent_cv;
      cv::Mat cad_cv;

      PathMatrixCostSearch::SearchStatus doDijkstra(const Vector2i& goal, const Vector2i& start, PathMatrix& pmap);
      inline float getCost(const Vector2i& pos, const PathMatrix& m);
      inline bool valueAndGradient(Vector3f& dest_, const Vector2i& pos_, const PathMatrix& matrix_, const float& gain);
      inline bool getCostAtSubPixel(Vector3f& dest_, const Vector2f& pos_, const PathMatrix& matrix_, const float& gain);
      inline Eigen::Vector2f grid2world(int r, int c);
      inline Eigen::Vector2i world2grid(const Eigen::Vector2f p);
      float computePathGrid(StdVectorEigenVector3f& path_, const Vector3f& current_pose_, PathMatrix& pm_, const GridMap2DHeader& mh_);
      float computePathGradient(StdVectorEigenVector3f& path_, const Vector3f& current_pose, PathMatrix& pm_, const GridMap2DHeader& mh_);
      void setMap(GridMap2DPtr grid_map_);
      bool loadMap(const std::string& yaml_filename);
      bool computePolicy(const Vector3f& goal);
  };

}
