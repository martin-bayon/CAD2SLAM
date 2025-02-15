#pragma once

#include "srrg_solver/variables_and_factors/types_2d/variable_se2.h"
#include "cad2slam/data_structures/keyframe_data.h"

namespace cad2slam {
  using namespace srrg2_solver;

  class KeyframeSE2 : public VariableSE2Right, public KeyframeData{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseVariableType = VariableSE2Right;

    void setCadPoint(Vector2f point) override;
    Vector2f getCadPoint() override;
    Vector2f getSlamPoint();
    Isometry2f getCadIso();
    Isometry2f getSlamIso();

    void serialize(ObjectData& data, IdContext& context) override;
    void deserialize(ObjectData& data, IdContext& context) override;
    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };

  using KeyframeSE2Ptr = std::shared_ptr<KeyframeSE2>;
}