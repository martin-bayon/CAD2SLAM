#include <srrg_data_structures/path_matrix_distance_search.h>
#include "srrg2_navigation_2d/planner_2d.h"
#include "srrg_system_utils/system_utils.h"

using namespace srrg2_navigation_2d;
using namespace srrg2_core;

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
StdVectorEigenVector2i _px_path;

cv::Mat distance_cv;
cv::Mat parent_cv;
cv::Mat cad_cv;


PathMatrixCostSearch::SearchStatus doDijkstra(const Vector2i& goal, const Vector2i& start, PathMatrix& pmap) {
  std::cerr << "In the beggining, the cost at start is: " << pmap.at(start).cost << std::endl;
  _dijkstra_search.setPathMatrix(&pmap);
  std::cerr << "Path matrix set" << std::endl;
  Point2iVectorCloud goals;
  Point2i goal_pt;
  goal_pt.coordinates() = goal;
  goals.push_back(goal_pt);
  std::cerr << "Goal pixel is: " << goal.transpose() << std::endl;
  std::cerr << "The start/pose pixel is: " << start.transpose() << std::endl;
  _dijkstra_search.reset();
  int num_good_goals = _dijkstra_search.setGoals(goals);
  std::cerr << "Num good goals: " << num_good_goals << std::endl;
  if (!num_good_goals) {
    return PathMatrixCostSearch::GoalNotFound;
  }
  _dijkstra_search.compute();

  std::cerr << "The cost at start is: " << pmap.at(start).cost << std::endl;
  std::cerr << "The distance at start is: " << pmap.at(start).distance << std::endl;
  std::cerr << "The heuristic at start is: " << pmap.at(start).heuristic << std::endl;
  std::cerr << "The dijkstra max cost is: " << _dijkstra_search.param_max_cost.constValue() << std::endl;


  std::cerr << "The result is: ";
  if (pmap.at(start).cost == _dijkstra_search.param_max_cost.constValue()) {
    return PathMatrixCostSearch::GoalNotFound;
    std::cerr << "Goal was not found" << std::endl;
  }
  std::cerr << "Goal was found successfully" << std::endl;
  return PathMatrixCostSearch::GoalFound;
}

inline float getCost(const Vector2i& pos, const PathMatrix& m) {
  if (! m.inside(pos))
    return -1;
  return m.at(pos).cost;
}

inline bool valueAndGradient(Vector3f& dest_, const Vector2i& pos_, const PathMatrix& matrix_, const float& gain) {
  float c   = getCost(pos_, matrix_);
  float cx1 = getCost(pos_ + Vector2i(-1, 0), matrix_);
  float cx2 = getCost(pos_ + Vector2i(1, 0), matrix_);
  float cy1 = getCost(pos_ + Vector2i(0, -1), matrix_);
  float cy2 = getCost(pos_ + Vector2i(0, 1), matrix_);
  if (c < 0 || cx1 < 0 || cx2 < 0 || cy1 < 0 || cy2 < 0) {
    return false;
  }
  dest_ = Vector3f(c, cx2 - cx1, cy2 - cy1);
  const auto& cell=matrix_.at(pos_);
  if (cell.parent) {
    Vector2i parent_pos=matrix_.pos(cell.parent);
    Vector2i i_parent=pos_-parent_pos;
    Vector2f d_parent=i_parent.cast<float>();
    dest_.tail<2>()+=gain*d_parent;
  }
  return true;
}

inline bool getCostAtSubPixel(Vector3f& dest_, const Vector2f& pos_, const PathMatrix& matrix_, const float& gain) {
  if (!matrix_.inside(pos_.x(), pos_.y())) {
    return false;
  }
  int x0 = pos_.x(); // rows
  int y0 = pos_.y(); // cols
  int x1 = x0 + 1;
  int y1 = y0 + 1;
  if (!matrix_.inside(x1, y1)) {
    return false;
  }

  const float dx  = pos_.x() - (float) x0;
  const float dy  = pos_.y() - (float) y0;
  const float dx1 = 1.f - dx;
  const float dy1 = 1.f - dy;
  dest_           = Vector3f::Zero();
  Vector3f temp   = Vector3f::Zero();
  bool ok         = valueAndGradient(temp, Vector2i(x0, y0), matrix_, gain);
  if (!ok) {
    return false;
  }
  dest_ += temp * dy1 * dx1;
  ok = valueAndGradient(temp, Vector2i(x0, y1), matrix_, gain);
  if (!ok) {
    return false;
  }
  dest_ += temp * dy1 * dx;
  ok = valueAndGradient(temp, Vector2i(x1, y0), matrix_, gain);
  if (!ok) {
    return false;
  }
  dest_ += temp * dy * dx1;
  ok = valueAndGradient(temp, Vector2i(x1, y1), matrix_, gain);
  if (!ok) {
    return false;
  }
  dest_ += temp * dy * dx;
  return true;
}

inline Eigen::Vector2f grid2world(int r, int c) {
  assert(_grid_map && "grid map not set");
  return _grid_map->indices2global(Vector2i(r, c));
}

inline Eigen::Vector2i world2grid(const Eigen::Vector2f p) {
  assert(_grid_map && "grid map not set");
  return _grid_map->global2indices(p);
}

float computePathGrid(StdVectorEigenVector3f& path_, const Vector3f& current_pose_, PathMatrix& pm_, const GridMap2DHeader& mh_) {
  path_.clear();
  int max_steps = 100000;
  int steps     = 0;
  Vector2i current_idx = mh_.global2indices(current_pose_.head<2>());
  Vector3f last_pose=current_pose_;
  last_pose.z()=0;
  float length = 0;

  std::cerr << "Start pose: " << current_pose_.transpose() << std::endl;
  std::cerr << "Start idx: " << mh_.global2indices(current_pose_.head<2>()).transpose() << std::endl;
  std::cerr << "Start cost: " << pm_.at(current_idx).cost << std::endl;
  std::cerr << "----------------------------" << std::endl;
  std::cerr << "Goal pose: " << _goal.transpose() << std::endl;
  std::cerr << "Goal idx: " << mh_.global2indices(_goal.head<2>()).transpose() << std::endl;
  std::cerr << "Goal cost: " << pm_.at(mh_.global2indices(_goal.head<2>())).cost << std::endl;
  std::cerr << "::::::::::::::::::::::::::::::::" << std::endl;

  while (steps < max_steps) {
    std::cerr << "steps: " << steps;
    std::cerr << " -- current_idx: " << current_idx.transpose();
    // bb next pose is parent of current pose in dijkstra search
    PathMatrixCell* parent = pm_.at(current_idx).parent;
    if (!parent) {
       std::cerr << "No parent!!!" << std::endl;
      return 0;
    }
    // bb retrieve index of the parent
    current_idx        = pm_.pos(parent);
    std::cerr << " -- Parent idx: " << current_idx.transpose();
    Vector3f trj_pose  = Vector3f::Zero();
    trj_pose.head<2>() = mh_.indices2global(current_idx);
    trj_pose(2)        = 0.f;
    std::cerr << " -- trj_pose: " << trj_pose.transpose();
    float delta=(trj_pose-last_pose).norm();
    length+=delta;
    last_pose=trj_pose;
    path_.push_back(trj_pose);
    _px_path.push_back(current_idx);
    ++steps;
    if (pm_.at(current_idx).cost == 0) {
      std::cerr << " -- Cost of the curr_idx has gone to 0 -- Return" << std::endl;
      return length;
    }else{
      std::cerr << " -- curr_idx.cost: " << pm_.at(current_idx).cost << std::endl;
    }
  }
  return -1;
}

float computePathGradient(StdVectorEigenVector3f& path_, const Vector3f& current_pose, PathMatrix& pm_, const GridMap2DHeader& mh_) {
  path_.clear();
  Vector2f indices_float = mh_.global2floatIndices(current_pose.head<2>());
  int max_steps          = 1000;
  int steps              = 0;
  // std::cerr << "start: " << current_pose.transpose() << std::endl;
  // std::cerr << "goal: " << _last_goal.transpose() << std::endl;
  Vector3f last_pose=current_pose;
  last_pose.z()=0;
  float length=0;
  while (steps < max_steps) {
    Vector2i indices = indices_float.cast<int>();
    Vector3f values;
    bool interpolation_ok = getCostAtSubPixel(values, indices_float, pm_, grid_gain);
    if (! interpolation_ok)
      return -1;

    const auto& cell = pm_.at(indices);
    if (cell.parent==&cell)
      return length;

    if (values(1) == 0 && values(2) == 0) {
      return -1;
    }
    Vector2f gradient = values.tail<2>();
    gradient.normalize();

    Vector3f trj_pose=Vector3f::Zero();
    trj_pose.head<2>() = mh_.floatIndices2global(indices_float);
    float delta=(trj_pose-last_pose).norm();
    if ( delta > _resolution) {
      path_.push_back(trj_pose);
      last_pose=trj_pose;
      length+=delta;
    }
    indices_float -= gradient * 0.5;
    ++steps;
  }
  return length;
}

void setMap(GridMap2DPtr grid_map_) {
  std::cerr << "setting map " << std::endl;
  assert(grid_map_ && "grid map not set");

  _grid_map            = grid_map_;
  float occ_threshold  = (1. - _occupancy_threshold) * 255;
  float free_threshold = (1. - _free_threshold) * 255;
  _resolution          = _grid_map->resolution();
  _inverse_resolution  = 1. / _resolution;

  std::cerr << FG_BBLUE("GRID MAP RESOLUTION: ") << _resolution << std::endl;
  using PropertyImageOccupancyType = Property_<ImageOccupancyType>;
  PropertyImageOccupancyType* occ_prop = _grid_map->property<PropertyImageOccupancyType>(occupancy_layer);
  if (!occ_prop) {
    throw std::runtime_error(std::string("occupancy layer [") + occupancy_layer + "] not found in map");
  }
  _map = occ_prop->value();
  _obstacles.clear();
  const size_t rows = _map.rows();
  const size_t cols = _map.cols();

  std::cerr << "setting map " << rows << " x " << cols << std::endl;

  // DEBUG
  std::cerr << "occ_th: " << occ_threshold << " free_th:" << free_threshold << std::endl;
  int free_count = 0, occ_count = 0, unknown_count = 0;
  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      unsigned char& src = _map.at(r, c);
      if (src < occ_threshold) {
        occ_count++;
        Point2i p;
        p.coordinates() = Eigen::Vector2i(r, c);
        _obstacles.push_back(p);
        std::cerr << "{"<<p.coordinates().x()<<","<<p.coordinates().y()<<"}" << std::endl;
        src = 0;
      } else if (src > free_threshold) {
        free_count++;
        src = 255;
      } else {
        unknown_count++;
        src = 127;
      }
    }
  }
  // std::cerr << "writing map" << std::endl;
  // cv::imwrite("proc_occ.pgm", _img_occ);
  std::cerr << "free: " << free_count << std::endl;
  std::cerr << "unknown: " << unknown_count << std::endl;
  std::cerr << "occupied: " << occ_count << std::endl;
  _distance_map.resize(rows, cols);

  /*
  std::cerr << "\tA1: " << _distance_map.pos(_distance_map.at(Vector2i(10, 10)).parent).transpose() << std::endl;
  std::cerr << "\tB1: " << _distance_map.pos(_distance_map.at(Vector2i(10, 16)).parent).transpose() << std::endl;
  std::cerr << "\tC1: " << _distance_map.pos(_distance_map.at(Vector2i(29, 67)).parent).transpose() << std::endl;
  std::cerr << "\tD1: " << _distance_map.pos(_distance_map.at(Vector2i(17, 67)).parent).transpose() << std::endl;
  std::cerr << "\tE1: " << _distance_map.pos(_distance_map.at(Vector2i(200, 200)).parent).transpose() << std::endl;
  std::cerr << "\tF1: " << _distance_map.pos(_distance_map.at(Vector2i(233, 200)).parent).transpose() << std::endl;
  std::cerr << "AAAA" << std::endl;
  std::cerr << "\tG1: " << _distance_map.pos(_distance_map.at(Vector2i(17, 67)).parent).transpose() << std::endl;
  std::cerr << "\tH1: " << _distance_map.pos(_distance_map.at(Vector2i(17, 67)).parent).transpose() << std::endl;
  std::cerr << "\tI1: " << _distance_map.pos(_distance_map.at(Vector2i(17, 67)).parent).transpose() << std::endl;
  */

  PathMatrixDistanceSearch dmap_calculator;
  dmap_calculator.setPathMatrix(&_distance_map);
  float dmax_in_pixels = max_point_distance * _inverse_resolution;
  dmap_calculator.param_max_distance_squared_pxl.setValue(dmax_in_pixels * dmax_in_pixels);
  dmap_calculator.setGoals(_obstacles);
  dmap_calculator.compute();

  /*
  std::cerr << "\tA1: " << _distance_map.pos(_distance_map.at(Vector2i(9, 9)).parent).transpose() << std::endl;
  std::cerr << "\tA2: " << _distance_map.pos(_distance_map.at(Vector2i(9, 10)).parent).transpose() << std::endl;
  std::cerr << "\tA3: " << _distance_map.pos(_distance_map.at(Vector2i(9, 11)).parent).transpose() << std::endl;
  std::cerr << "\tA4: " << _distance_map.pos(_distance_map.at(Vector2i(10, 9)).parent).transpose() << std::endl;
  std::cerr << "\tA5: " << _distance_map.pos(_distance_map.at(Vector2i(10, 10)).parent).transpose() << std::endl;
  std::cerr << "\tA6: " << _distance_map.pos(_distance_map.at(Vector2i(10, 11)).parent).transpose() << std::endl;
  std::cerr << "\tA7: " << _distance_map.pos(_distance_map.at(Vector2i(11, 9)).parent).transpose() << std::endl;
  std::cerr << "\tA8: " << _distance_map.pos(_distance_map.at(Vector2i(11, 10)).parent).transpose() << std::endl;
  std::cerr << "\tA9: " << _distance_map.pos(_distance_map.at(Vector2i(11, 11)).parent).transpose() << std::endl;

  std::cerr << "\tB1: " << _distance_map.pos(_distance_map.at(Vector2i(10, 16)).parent).transpose() << std::endl;
  std::cerr << "\tB2: " << _distance_map.pos(_distance_map.at(Vector2i(11, 16)).parent).transpose() << std::endl;
  */

  Matrix_<float>* distances = nullptr;
  if (distance_layer.length()) {
    using PropertyDistanceType = Property_<Matrix_<float>>;
    PropertyDistanceType* dist_prop = _grid_map->property<PropertyDistanceType>(distance_layer);
    if (!dist_prop) {
      dist_prop = new PropertyDistanceType(distance_layer, "float", _grid_map.get(), Matrix_<float>(), nullptr);
    }
    distances = &dist_prop->value();
    distances->resize(rows, cols);
  }

  int k = 0;
  _free_cells.resize(rows * cols);

  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      //update free list;
      unsigned char pixel     = _map.at(r, c);
      int distance_sign=(pixel==127)?-1:1;
      float &distance_px= _distance_map.at(r, c).distance;
      float &distance_mt= distances->at(r,c);
      distance_mt=distance_sign * sqrt(distance_px) * _resolution;
      distance_px*=distance_sign;
      if (pixel == 255 && distance_mt > robot_radius) {
        _free_cells[k] = grid2world(r, c);
        ++k;
      }
    }
  }

  // cv::imwrite("proc_dist.pgm", _img_dist);
  _free_cells.resize(k);
  std::cerr << "valid: " << _free_cells.size() << std::endl;
}

bool loadMap(const std::string& filename) {
  const int line_size_max = 1024;
  std::ifstream is(filename.c_str());
  if(!is) {
    std::cerr << "Returning a nullptr at 1" << std::endl;
    std::cerr << "No file at: " << filename.c_str() << std::endl;
    return false;
  }
  float resolution      = 0;
  float occupied_thresh = 0;
  float free_thresh     = 0;
  std::string image_file;
  bool negate = 0;
  while (is) {
    char line[line_size_max];
    is.getline(line, line_size_max);
    std::istringstream ls(line);
    std::string tag;
    ls >> tag;
    if (tag == "resolution:") {
      ls >> resolution;
      continue;
    }
    if (tag == "negate:") {
      ls >> negate;
      continue;
    }
    if (tag == "free_thresh:") {
      ls >> free_thresh;
      continue;
    }
    if (tag == "occupied_thresh:") {
      ls >> occupied_thresh;
      continue;
    }
    if (tag == "origin:") {
      continue;
    }
    if (tag == "image:") {
      ls >> image_file;
      continue;
    }
  }
  if (resolution == 0 || !image_file.length()) {
    std::cerr << "Returning a nullptr at 2" << std::endl;
    return false;
  }
  cv::Mat cv_img = cv::imread(image_file, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
  cad_cv = cv_img.clone();
  cv::cvtColor(cv_img, cv_img, cv::COLOR_BGR2GRAY);

  ImageUInt8 image;
  image.fromCv(cv_img);

  std::cerr << "Got map" << image.rows() << "x" << image.cols() << std::endl;
  std::cerr << "res: " << resolution << std::endl;
  size_t rows = image.rows();
  size_t cols = image.cols();
  GridMap2DPtr grid_map(new GridMap2D);
  grid_map->setSize(Eigen::Vector2i(rows, cols));
  grid_map->setResolution(resolution);
  using PropertyImageOccupancyType = Property_<ImageOccupancyType>;
  _occupancy_threshold = occupied_thresh;
  _free_threshold = free_thresh;
  std::cerr << "free threshold: " << _free_threshold << std::endl;
  std::cerr << "occupied threshold: " << _occupancy_threshold << std::endl;

  PropertyImageOccupancyType* occ_prop = new PropertyImageOccupancyType(
          occupancy_layer, "", grid_map.get(), Matrix_<uint8_t>(), nullptr);
  Matrix_<uint8_t>& occ_map = occ_prop->value();
  occ_map.resize(rows, cols);
  for (size_t r = 0; r < rows; ++r)
    for (size_t c = 0; c < cols; ++c) {
      occ_map.at(r, c) = image.at(r, c);
    }
  cv::Mat img2;

  // adjust external coords for ROS
  float x_size = rows * resolution;
  float y_size = cols * resolution;
  Eigen::Vector3f origin(y_size / 2, x_size / 2, -M_PI / 2);
  std::cerr << "Origin: " << origin << std::endl;
  grid_map->setOrigin(geometry2d::v2t(origin));
  setMap(grid_map);
  return true;
}

bool computePolicy(const Vector3f& goal) {
  bool return_value = true;
  if (!_grid_map) {
    return_value = false;
    //return false;
  }
  Vector2i goal_pxl = world2grid(goal.head<2>());
  _goal_pose_pxl = goal_pxl;
  if (!_global_pmap.inside(goal_pxl)) {
    std::cerr << "goal out of map" << std::endl;
    std::cerr << "Goal not Admissible" << std::endl;
    return_value = false;
    //return false;
  }else{
    std::cerr << "The goal is inside the map at: " << goal_pxl.transpose() << std::endl;
  }
  PathMatrixCell& goal_cell = _global_pmap.at(goal_pxl);
  if (goal_cell.distance < robot_radius) {
    std::cerr << "goal too close or inside obstacles" << std::endl;
    std::cerr << "The goal cell distance is: " << goal_cell.distance << std::endl;
    std::cerr << "Goal not Admissible" << std::endl;
    return_value = false;
    //return false;
  }{
    std::cerr << "The goal cell distance is: " << goal_cell.distance << std::endl;
  }

  Vector2i pose_pxl = _grid_map->global2indices(_current_robot_pose.head<2>());
  _robot_pose_pxl = pose_pxl;
  if (!_global_pmap.inside(pose_pxl)) {
    std::cerr << "The current_robot_pose is not inside the map" << std::endl;
    return_value = false;
    //return false;
  }else{
    std::cerr << "The robot is inside the map at: " << pose_pxl.transpose() << std::endl;
  }

  std::cerr << "-----------------------------------\nWe will do Dijkstra now\n-----------------------------------" << std::endl;


  PathMatrixCostSearch::SearchStatus search_status = doDijkstra(goal_pxl, pose_pxl, _global_pmap);
  if (search_status != PathMatrixCostSearch::SearchStatus::GoalFound) {
    std::cerr << "Goal Unreachable" << std::endl;
    return_value = false;
    //return false;
  }else{
    std::cerr << "The search status was: " << search_status << std::endl;
  }
  std::cerr << "Cruising" << std::endl;
  return return_value;
}

std::vector<Vector2i> doPathPlanning(Vector2i start_pixel, Vector2i goal_pixel, std::string cad_filename){
  loadMap(cad_filename);

  _global_pmap = _distance_map;

  //Planner is ready to recieve messages

  //Get Robot pose
  //_current_robot_pose = Vector3f(std::atoi(argv[1]), std::atoi(argv[2]), 0);
  auto curr_robot_world = grid2world(start_pixel.x(), start_pixel.y());
  _current_robot_pose = Vector3f(curr_robot_world.x(), curr_robot_world.y(), 0);

  //HandleMapChanged
  //Nothing to do here - The map won't change

  //HandleSetGoal
  //_goal = Vector3f(std::atoi(argv[3]), std::atoi(argv[4]), 0);
  auto goal_world = grid2world(goal_pixel.x(), goal_pixel.y());
  _goal = Vector3f(goal_world.x(), goal_world.y(), 0);

  //HadleCostMapChanged
  std::vector<float> cost_poly;
  cost_poly.push_back(1.0f);
  cost_poly.push_back(10.0f);
  float inv_resolution = 1. / _resolution;
  float alpha = inv_resolution;
  for (float& v : cost_poly) {
    v *= alpha;
    alpha *= inv_resolution;
  }
  _dijkstra_search.param_min_distance.setValue(pow(robot_radius / _resolution, 2));
  _dijkstra_search.param_max_cost.setValue(1e12);
  _dijkstra_search.param_cost_polynomial.setValue(cost_poly);


  std::cerr << "Goal: " << _goal.transpose() << std::endl;
  bool goal_ok = computePolicy(_goal);
  std::cerr << "goal_ok:" << goal_ok << std::endl;


  {
    ImageUInt8 path_map_image;
    ImageInt parent_map_image;
    cv::Mat m, p;

    _distance_map.toImage(path_map_image, PathMatrix::Distance);
    _distance_map.toImage(parent_map_image, PathMatrix::Parent);
    path_map_image.toCv(m);
    parent_map_image.toCv(p);

    int depth = m.depth();
    if (depth == CV_16F || depth == CV_32S) {
      // Convert the image depth to CV_8U (8-bit unsigned integer)
      m.convertTo(m, CV_8U);
    }

    depth = p.depth();
    if (depth == CV_16F || depth == CV_32S) {
      // Convert the image depth to CV_8U (8-bit unsigned integer)
      p.convertTo(p, CV_8U);
    }

    cv::Mat m_c, p_c;

    cv::cvtColor(m, m_c, cv::COLOR_GRAY2BGR);
    cv::cvtColor(p, p_c, cv::COLOR_GRAY2BGR);

    cv::circle(m_c, cv::Point(_robot_pose_pxl.y(), _robot_pose_pxl.x()), 3, cv::Scalar(0,0,255), -1);
    cv::circle(m_c, cv::Point(_goal_pose_pxl.y(), _goal_pose_pxl.x()), 3, cv::Scalar(255,0,0), -1);

    cv::circle(p_c, cv::Point(_robot_pose_pxl.y(), _robot_pose_pxl.x()), 3, cv::Scalar(0,0,255), -1);
    cv::circle(p_c, cv::Point(_goal_pose_pxl.y(), _goal_pose_pxl.x()), 3, cv::Scalar(255,0,0), -1);

    cv::circle(cad_cv, cv::Point(_robot_pose_pxl.y(), _robot_pose_pxl.x()), 3, cv::Scalar(0,0,255), -1);
    cv::circle(cad_cv, cv::Point(_goal_pose_pxl.y(), _goal_pose_pxl.x()), 3, cv::Scalar(255,0,0), -1);

    parent_cv = p_c.clone();
    distance_cv = m_c.clone();

    cv::imshow("parent map", p_c);
    cv::imshow("distance map", m_c);
    cv::waitKey();
  }


  if(!goal_ok){
    exit(-1);
  }

  std::cerr << "________________________________________" << std::endl;
  auto _distance_to_global_goal = computePathGrid(_path, _current_robot_pose, _global_pmap, *_grid_map);
  std::cerr << "________________________________________" << std::endl;

  std::cerr << "Distance to global goal: " << _distance_to_global_goal << std::endl;
  /*
  std::cerr << "[PATH]" << std::endl;
  for(auto step : _path){
    std::cerr << step.transpose() << std::endl;
  }
  */


  for(auto px : _px_path){
    cv::circle(parent_cv, cv::Point(px.y(), px.x()), 1, cv::Scalar(0,255,0), -1);
    cv::circle(distance_cv, cv::Point(px.y(), px.x()), 1, cv::Scalar(0,255,0), -1);
    cv::circle(cad_cv, cv::Point(px.y(), px.x()), 1, cv::Scalar(0,255,0), -1);
  }
  cv::imshow("Path in parent map", parent_cv);
  cv::imshow("Path in distance map", distance_cv);
  cv::imshow("Path in cad mao", cad_cv);
  cv::waitKey(0);


  std::cerr << "[DONE]" << std::endl;
}

int main( int argc, char** argv ) {
  doPathPlanning(Vector2i(std::atoi(argv[1]), std::atoi(argv[2])), Vector2i(std::atoi(argv[3]), std::atoi(argv[4])), argv[5]);

  //Result is in --> _px_path

}