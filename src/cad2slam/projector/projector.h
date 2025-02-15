#pragma once

#include "srrg_config/configurable.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "opencv2/opencv.hpp"

#include "cad2slam/config_master/config_master.h"
#include "cad2slam/map_manager/map_manager.h"
#include "cad2slam/utils/graph_utils.h"

namespace cad2slam {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  class Projector : public Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = Projector;
    using BaseType = Configurable;

    PARAM(PropertyConfigurable_<ConfigMaster>, config_master, "Config master", nullptr, nullptr);
    PARAM(PropertyConfigurable_<MapManager>, cad_map_manager, "Cad map manager", nullptr, nullptr);
    PARAM(PropertyConfigurable_<MapManager>, slam_map_manager, "Slam map manager", nullptr, nullptr);
    PARAM(PropertyBool, use_new_cad2slam, "Use new cad2slam", false, nullptr);
    PARAM(PropertyInt, k_neighbours, "K neighbours", 4, nullptr);
    PARAM(PropertyFloat, threshold, "threshold", 2, nullptr);

    Projector();

    void setGraph(FactorGraphPtr graph_);
    void loadGraph();
    void launchProjector();

    Vector3f cad2slam(Vector3f cad_pose);
    Vector3f slam2cad(Vector3f slam_pose);

    Vector3f cad2slam_OLD(Vector3f cad_pose);
    Vector3f cad2slam_NEW(Vector3f cad_pose);

    std::pair<Vector2i, float> cad_pixel2slam_pixel(Vector2i cad_pixel, float cad_theta);
    std::pair<Vector2i, float> slam_pixel2cad_pixel(Vector2i slam_pixel, float slam_theta);

    std::pair<Vector2i, float> cad_pixel2slam_pixel_no_resize(Vector2i cad_pixel, float cad_theta);
    std::pair<Vector2i, float> slam_pixel2cad_pixel_no_resize(Vector2i slam_pixel, float slam_theta);

    std::pair<Vector2i, float> cad_pixel2slam_pixel_OLD(Vector2i cad_pixel, float cad_theta);
    std::pair<Vector2i, float> cad_pixel2slam_pixel_NEW(Vector2i cad_pixel, float cad_theta);

  protected:
    void initialize();
    void checkParams();
    void checkGraph();

    bool cmdLoadGraph(std::string &response);

    static void cad2slam_callback(int event, int x, int y, int flags, void* userdata);
    static void slam2cad_callback(int event, int x, int y, int flags, void* userdata);
    static bool cadOriginalIsWhite(Vector2i resized_pixel, Projector* projector);
    static bool slamOriginalIsWhite(Vector2i resized_pixel, Projector* projector);

    bool _initialized = false;
    bool _is_graph_ready = false;
    FactorGraphPtr _graph;

    static constexpr int _arrow_length = 30;
    static constexpr int _arrow_thickness = 2;
    static std::vector<Vector2i> _prev_cad_pixels;
    static std::vector<Vector2i> _prev_slam_pixels;

    static constexpr int _white_pixel_threshold = 200;
  };
}
