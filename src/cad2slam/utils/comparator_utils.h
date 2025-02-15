#pragma once

#include "srrg_geometry/geometry_defs.h"

namespace cad2slam{
  using namespace srrg2_core;

  struct Vector2iComparator{
    inline bool operator()(const Vector2i& first, const Vector2i& second) const {
      if(first.x() < second.x()){
        return true;
      }else if (first.x() > second.x()){
        return false;
      }else{
        return first.y() < second.y();
      }
    }
  };
}