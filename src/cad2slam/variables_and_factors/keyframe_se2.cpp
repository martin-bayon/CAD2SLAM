#include "keyframe_se2.h"
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"

namespace cad2slam {

  void KeyframeSE2::serialize(ObjectData &data, IdContext &context) {
    BaseVariableType::serialize(data, context);
    KeyframeData::serialize(data,context);
  }

  void KeyframeSE2::deserialize(ObjectData& data, IdContext& context){
    BaseVariableType::deserialize(data, context);
    KeyframeData::deserialize(data,context);
  }

  void KeyframeSE2::_drawImpl(srrg2_core::ViewerCanvasPtr canvas_) const {
    if(!canvas_){
      throw std::runtime_error("[KeyframeSE2::_drawImpl] | Invalid canvas");
    }
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlack());
    if(this->status() == VariableBase::Status::Fixed){
      canvas_->setColor(srrg2_core::ColorPalette::color3fRed());
    }
    canvas_->pushMatrix();
    canvas_->multMatrix(geometry3d::get3dFrom2dPose(estimate()).matrix());
    if(_outer){
      canvas_->putSphere(0.1);
    }else{
      canvas_->putSphere(0.05);
    }

    canvas_->popMatrix();
    canvas_->popAttribute();
  }

  void KeyframeSE2::setCadPoint(Vector2f point) {
    KeyframeData::setCadPoint(point);
    Vector3f normalizedPose(point.x(), point.y(), 0);
    BaseVariableType::setEstimate(geometry2d::v2t(normalizedPose));
  }

  Vector2f KeyframeSE2::getCadPoint() {
    return KeyframeData::getCadPoint();
  }

  Isometry2f KeyframeSE2::getCadIso() {
    auto cad_point = getCadPoint();
    return geometry2d::v2t(Vector3f(cad_point.x(), cad_point.y(), 0));
  }

  Vector2f KeyframeSE2::getSlamPoint() {
    return geometry2d::t2v(estimate()).head<2>();
  }

  Isometry2f KeyframeSE2::getSlamIso(){
    return estimate();
  }

  INSTANTIATE(KeyframeSE2);
}
