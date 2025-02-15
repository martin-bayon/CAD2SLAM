
#include "factor_keyframe2anchor.h"

#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"

namespace cad2slam {
  using namespace srrg2_core;

  void FactorKeyframe2Anchor::errorAndJacobian(bool error_only) {
    const Isometry2f& X = _variables.at<0>()->estimate();
    const Vector2f p = _slam_point;

    Vector2f prediction = X.inverse() * p;
    _e                  = prediction - _measurement;
    if (error_only) {
      return;
    }
    _J.block<2, 2>(0, 0) = -1.0f * Matrix2f::Identity();
    _J.block<2, 2>(0, 0) = -1.0f * Matrix2f::Identity();
  }

  void FactorKeyframe2Anchor::_drawImpl(ViewerCanvasPtr canvas) const {
    if(!canvas){
      throw std::runtime_error("FactorKeyframe2Anchor::_drawImpl | Invalid canvas");
    }
    const auto* var = dynamic_cast<const KeyframeSE2*>(variable(0));
    srrg2_core::Isometry2f var_iso = var->estimate();

    Isometry2f measurement_iso = _measurement_iso();
    auto measurement_from_world = var_iso * measurement_iso;

    //Draw the SLAM anchor
    canvas->pushColor();
    canvas->setColor(srrg2_core::ColorPalette::color3fDarkGreen());
    canvas->pushMatrix();
    canvas->multMatrix(geometry3d::get3dFrom2dPose(geometry2d::v2t(Vector3f(_slam_point.x(), _slam_point.y(), 0.f))).matrix());
    canvas->putSphere(0.05);
    canvas->popMatrix();
    canvas->popAttribute();

    //Draw the CAD anchor
    canvas->pushMatrix();
    canvas->setColor(srrg2_core::ColorPalette::color3fRed());
    canvas->multMatrix(geometry3d::get3dFrom2dPose(measurement_from_world).matrix());
    canvas->putSphere(0.05);
    canvas->popMatrix();
    canvas->popAttribute();

    //Draw the line between the CAD anchor and the Keyframe
    Vector3f slam_anchor2cad_anchor_line[2];
    slam_anchor2cad_anchor_line[0] << measurement_from_world.translation().x(), measurement_from_world.translation().y(), 0;
    slam_anchor2cad_anchor_line[1] << geometry2d::t2v(var_iso);
    canvas->pushColor();
    canvas->setColor(srrg2_core::ColorPalette::color3fDarkMagenta());
    canvas->putSegment(2, slam_anchor2cad_anchor_line, 0);

    //Draw the line between both anchors
    Vector3f slam_anchor2keyframe_line[2];
    slam_anchor2keyframe_line[0] << geometry2d::t2v(measurement_from_world);
    slam_anchor2keyframe_line[1] << _slam_point.x(), _slam_point.y(), 0.f;
    canvas->pushColor();
    canvas->setColor(srrg2_core::ColorPalette::color3fGreen());
    canvas->putSegment(2, slam_anchor2keyframe_line, 0);
  }

  void FactorKeyframe2Anchor::serialize(ObjectData &data, IdContext &context) {
    ErrorFactor_::serialize(data, context);
    data.setEigen("_slam_point", _slam_point);
  }

  void FactorKeyframe2Anchor::deserialize(ObjectData &data, IdContext &context) {
    ErrorFactor_::deserialize(data, context);
    _slam_point = data.getEigen<Vector2f>("_slam_point");
  }

  Isometry2f FactorKeyframe2Anchor::_measurement_iso() const {
    return geometry2d::v2t(Vector3f(_measurement.x(), _measurement.y(), 0));
  }

  INSTANTIATE(FactorKeyframe2Anchor);
}

