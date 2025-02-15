#include "path_poses.h"

namespace cad2slam {

  PathPoses::PathPoses() = default;

  PathPoses::PathPoses(Vector2i start, Vector2i end) {
    _start = start;
    _end = end;
  }

  void PathPoses::serialize(ObjectData &data, IdContext &context) {
    data.setEigen("_start", _start);
    data.setEigen("_end", _end);
  }

  void PathPoses::deserialize(ObjectData &data, IdContext &context) {
    _start = data.getEigen<Vector2i>("_start");
    _end = data.getEigen<Vector2i>("_end");
  }
}