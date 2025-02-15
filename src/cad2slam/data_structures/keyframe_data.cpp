#include "keyframe_data.h"

#include <utility>


namespace cad2slam{
  using namespace srrg2_core;

  void KeyframeData::setCadPoint(Vector2f point) {
    _cad_point = std::move(point);
  }

  Vector2f KeyframeData::getCadPoint() {
    return _cad_point;
  }

  void KeyframeData::setRoiIndex(Vector2i roi_index) {
    _roi_index = std::move(roi_index);
  }

  Vector2i KeyframeData::getRoiIndex() {
    return _roi_index;
  }

  void KeyframeData::setOuter(bool outer) {
    _outer = outer;
  }

  bool KeyframeData::isOuter() const {
    return _outer;
  }

/////////////////////////////////////////////////////////////////////////////////////////////

  void KeyframeData::serialize(ObjectData &data, IdContext &context) {
    data.setEigen("_cad_point", _cad_point);
    data.setEigen("_roi_index", _roi_index);
    data.setBool("_outer", _outer);
  }

  void KeyframeData::deserialize(ObjectData &data, IdContext &context) {
    _cad_point = data.getEigen<Vector2f>("_cad_point");
    _roi_index = data.getEigen<Vector2i>("_roi_index");
    _outer = data.getBool("_outer");
  }

}
