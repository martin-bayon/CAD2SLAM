#pragma once

#include "cad2slam/map_manager/map_manager.h"

namespace cad2slam {
    struct CorrespondenceFinderInfo {
        MapManager* map_manager;
        std::vector<Vector2f> points;
    };
}