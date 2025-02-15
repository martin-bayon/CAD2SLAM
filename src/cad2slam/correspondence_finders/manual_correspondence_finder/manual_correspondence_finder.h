#pragma once

#include "srrg_config/configurable.h"
#include "srrg_boss/serializer.h"
#include "srrg_geometry/geometry2d.h"

#include "cad2slam/config_master/config_master.h"
#include "cad2slam/map_manager/map_manager.h"
#include "cad2slam/data_structures/correspondence_finder_info.h"
#include "cad2slam/utils/image_utils.h"

namespace cad2slam {
  using namespace srrg2_core;

  class ManualCorrespondenceFinder : public Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType = ManualCorrespondenceFinder;
    using BaseType = Configurable;

    PARAM(PropertyConfigurable_<ConfigMaster>, config_master, "Config master", nullptr, nullptr);
    PARAM(PropertyConfigurable_<MapManager>, cad_map_manager, "Cad map manager", nullptr, nullptr);
    PARAM(PropertyConfigurable_<MapManager>, slam_map_manager, "Slam map manager", nullptr, nullptr);
    PARAM(PropertyInt, visualizer_correspondence_size, "Visualizer correspondence size", 5, nullptr);

    ManualCorrespondenceFinder();

    void selectCorrespondences();
    void visualizeCorrespondences();

  private:
    bool cmdOpenCorrespondenceFinder(std::string& response);

    void initialize();
    void checkParams();

    static void correspondence_finder_callback(int event, int x, int y, int flags, void *userdata);
    void saveCorrespondences();

    bool _is_initialized = false;

    std::vector<Vector2f> _cad_correspondence_points;
    std::vector<Vector2f> _slam_correspondence_points;

    CorrespondenceFinderInfo cad_info;
    CorrespondenceFinderInfo slam_info;
  };
}
