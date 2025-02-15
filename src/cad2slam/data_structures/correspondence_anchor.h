#pragma once

#include "srrg_geometry/geometry_defs.h"
#include "srrg_boss/identifiable.h"
#include "srrg_boss/object_data.h"

namespace cad2slam {
  using namespace srrg2_core;

  class CorrespondenceAnchor : public Serializable{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CorrespondenceAnchor();
    CorrespondenceAnchor(Vector2f cad_point, Vector2f slam_point);

    void serialize(ObjectData &data, IdContext &context) override;
    void deserialize(ObjectData &data, IdContext &context) override;

    void setCadPoint(Vector2f cad_point_);
    void setSlamPoint(Vector2f slam_point_);
    Vector2f getCadPoint();
    Vector2f getSlamPoint();

  protected:
    Vector2f _cad_point;
    Vector2f _slam_point;
  };
}