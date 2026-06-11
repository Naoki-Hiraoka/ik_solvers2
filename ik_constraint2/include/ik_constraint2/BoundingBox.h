#pragma once

#include <cnoid/Body>

namespace ik_constraint2 {
  class BoundingBox {
  public:
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity();
    cnoid::LinkPtr parentLink = nullptr;
    cnoid::Vector3 dimensions = cnoid::Vector3::Zero();

    bool isInside(const cnoid::Vector3& p);
    void cacheParentLinkPose();
  protected:
    Eigen::Isometry3d worldPoseinv;
  };

}

