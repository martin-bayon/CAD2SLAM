#include "instances.h"

#include "all_types.h"

namespace cad2slam{
  void cad2slam_registerTypes(){
    BOSS_REGISTER_CLASS(KeyframeSE2);
    BOSS_REGISTER_CLASS(FactorKeyframe2Keyframe);
    BOSS_REGISTER_CLASS(FactorKeyframe2Anchor);
  }
}