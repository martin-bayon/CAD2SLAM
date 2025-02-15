#include "correspondence_anchor.h"

#include <utility>

namespace cad2slam {
  CorrespondenceAnchor::CorrespondenceAnchor() = default;

  CorrespondenceAnchor::CorrespondenceAnchor(Vector2f cad_point, Vector2f slam_point) {
    _cad_point = std::move(cad_point);
    _slam_point = std::move(slam_point);
  }

  void CorrespondenceAnchor::serialize(ObjectData &data, IdContext &context) {
    data.setEigen("_cad_point", _cad_point);
    data.setEigen("_slam_point", _slam_point);
  }

  void CorrespondenceAnchor::deserialize(ObjectData &data, IdContext &context) {
    _cad_point = data.getEigen<Vector2f>("_cad_point");
    _slam_point = data.getEigen<Vector2f>("_slam_point");
  }

  void CorrespondenceAnchor::setCadPoint(Vector2f cad_point_) {
    _cad_point = std::move(cad_point_);
  }

  void CorrespondenceAnchor::setSlamPoint(Vector2f slam_point_) {
    _slam_point = std::move(slam_point_);
  }

  Vector2f CorrespondenceAnchor::getCadPoint() {
    return _cad_point;
  }

  Vector2f CorrespondenceAnchor::getSlamPoint() {
    return _slam_point;
  }
}