#pragma once
#include "srrg_geometry/geometry_defs.h"

#include "srrg_boss/identifiable.h"
#include "srrg_boss/object_data.h"

namespace cad2slam{
  using namespace srrg2_core;

  class KeyframeData{
  public:
    virtual void setCadPoint(Vector2f point);
    virtual Vector2f getCadPoint();

    virtual void setRoiIndex(Vector2i roi_index);
    virtual Vector2i getRoiIndex();

    void serialize(ObjectData &data, IdContext &context);
    void deserialize(ObjectData &data, IdContext &context);

    bool isOuter() const;
    void setOuter(bool outer);

  protected:
    Vector2f _cad_point;
    Vector2i _roi_index;
    bool _outer;
  };
}
