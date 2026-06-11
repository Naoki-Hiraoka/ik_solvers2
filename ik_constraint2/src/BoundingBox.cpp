#include <ik_constraint2/BoundingBox.h>

namespace ik_constraint2 {
  bool BoundingBox::isInside(const cnoid::Vector3& p) {
    cnoid::Vector3 plocal = worldPoseinv * p;
    return
      (plocal[0] < dimensions[0]/2) &&
      (plocal[1] < dimensions[1]/2) &&
      (plocal[2] < dimensions[2]/2) &&
      (plocal[0] > -dimensions[0]/2) &&
      (plocal[1] > -dimensions[1]/2) &&
      (plocal[2] > -dimensions[2]/2);
  }

  void BoundingBox::cacheParentLinkPose(){
    if(parentLink){
      worldPoseinv = (parentLink->T() * localPose).inverse();
    }else{
      worldPoseinv = Eigen::Isometry3d::Identity();
    }
  }

}

