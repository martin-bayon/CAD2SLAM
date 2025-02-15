#include "factor_keyframe2keyframe.h"


namespace cad2slam{

  void FactorKeyframe2Keyframe::_drawImpl(srrg2_core::ViewerCanvasPtr canvas) const {
    if(!canvas){
      throw std::runtime_error("FactorRefKeyframe2RefKeyframeAD::_drawImpl | Invalid canvas");
    }
    const auto* var1 = dynamic_cast<const KeyframeSE2*>(variable(0));
    const auto* var2 = dynamic_cast<const KeyframeSE2*>(variable(1));

    Isometry2f var1_iso = var1->estimate();
    Isometry2f var2_iso = var2->estimate();

    Vector3f segments[2];
    segments[0] << var1_iso.translation().x(), var1_iso.translation().y(), 0.f;
    segments[1] << var2_iso.translation().x(), var2_iso.translation().y(), 0.f;

    canvas->pushColor();
    if(isOuter()){
      canvas->setColor(srrg2_core::ColorPalette::color3fBlue());
    }else{
      canvas->setColor(srrg2_core::ColorPalette::color3fGreen());
    }
    if(!enabled()){
      canvas->setColor(srrg2_core::ColorPalette::color3fLightGray());
    }
    canvas->putSegment(2, segments, 0);
  }

  Matrix2f FactorKeyframe2Keyframe::rotationMatrix(){
    auto t_ij = asVector();
    auto t_ij_orthogonal = Vector2f(-t_ij.y(), t_ij.x());
    Matrix2f R_ij;
    R_ij << t_ij, t_ij_orthogonal;
    return R_ij;
  }

  Vector2f FactorKeyframe2Keyframe::asVector(){
    Isometry2f var0_iso = dynamic_cast<const KeyframeSE2*>(variable(0))->estimate();
    Isometry2f var1_iso = dynamic_cast<const KeyframeSE2*>(variable(1))->estimate();
    float d_x = var1_iso.translation().x() - var0_iso.translation().x();
    float d_y = var1_iso.translation().y() - var0_iso.translation().y();
    return Vector2f(d_x, d_y);
  }

  bool FactorKeyframe2Keyframe::isOuter() const {
    const KeyframeSE2* var0 = dynamic_cast<const KeyframeSE2*>(variable(0));
    const KeyframeSE2* var1 = dynamic_cast<const KeyframeSE2*>(variable(1));

    if(!var0){
      throw std::runtime_error("[FactorKeyframe2Keyframe::isOuter] Variable at position 0 is not a KeyframeSE2 variable - Exiting...");
    }
    if(!var1){
      throw std::runtime_error("[FactorKeyframe2Keyframe::isOuter] Variable at position 1 is not a KeyframeSE2 variable - Exiting...");
    }
    return var0->isOuter() && var1->isOuter();
  }
}