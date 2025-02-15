#pragma once

#include "srrg_geometry/geometry_defs.h"
#include "srrg_boss/identifiable.h"
#include "srrg_boss/object_data.h"

namespace cad2slam {
  using namespace srrg2_core;

  class PathPoses : public Serializable{
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      PathPoses();
      PathPoses(Vector2i start, Vector2i end);

      void serialize(ObjectData &data, IdContext &context) override;
      void deserialize(ObjectData &data, IdContext &context) override;

      Vector2i _start;
      Vector2i _end;
  };
}
