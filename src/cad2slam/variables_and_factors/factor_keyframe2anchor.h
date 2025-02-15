#pragma once

#include "srrg_solver/solver_core/error_factor.h"
#include "cad2slam/variables_and_factors/keyframe_se2.h"

namespace cad2slam {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  //mb - Adapted from srrg_solver/variables_and_factors/se2_pose_point_error_factor.h
  class FactorKeyframe2Anchor : public ErrorFactor_<2, KeyframeSE2>,
                                public MeasurementOwnerEigen_<Vector2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void errorAndJacobian(bool error_only = false) final;
    inline void setSlamPoint(const Vector2f slam_point_) {
      _slam_point = slam_point_;
    }

    void _drawImpl(srrg2_core::ViewerCanvasPtr canvas) const override;
    void serialize(ObjectData& data, IdContext& context) override;
    void deserialize(ObjectData& data, IdContext& context) override;

  protected:
    Vector2f _slam_point;
    Isometry2f _measurement_iso() const;
  };

  using FactorKeyframe2AnchorPtr = std::shared_ptr<FactorKeyframe2Anchor>;
}
