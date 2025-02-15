#pragma once

#include "srrg_solver/variables_and_factors/types_2d/all_types.h"
#include "keyframe_se2.h"

namespace cad2slam{
  using namespace srrg2_solver;

  class FactorKeyframe2Keyframe : public SE2PosePoseGeodesicErrorFactor{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void _drawImpl(srrg2_core::ViewerCanvasPtr canvas) const override;
    Matrix2f rotationMatrix();
    bool isOuter() const;
  private:
    Vector2f asVector();
  };

  using FactorKeyframe2KeyframePtr = std::shared_ptr<FactorKeyframe2Keyframe>;
}