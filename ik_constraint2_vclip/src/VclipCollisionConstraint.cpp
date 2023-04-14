#include <ik_constraint2_vclip/VclipCollisionConstraint.h>
#include <iostream>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <choreonoid_vclip/choreonoid_vclip.h>

namespace ik_constraint2_vclip{
  bool VclipCollisionConstraint::computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) {
    assert(A_link && B_link); // temporary

    if(A_link && A_link != this->A_link_vclipModel_){
      this->A_vclipModel_ = choreonoid_vclip::convertToVClipModel(choreonoid_qhull::convertToConvexHull(A_link->collisionShape()));
      this->A_link_vclipModel_ = A_link;
    }
    if(B_link && B_link != this->B_link_vclipModel_){
      this->B_vclipModel_ = choreonoid_vclip::convertToVClipModel(choreonoid_qhull::convertToConvexHull(B_link->collisionShape()));
      this->B_link_vclipModel_ = B_link;
    }

    cnoid::Vector3 A_localp, B_localp;
    double dist;
    bool solved = choreonoid_vclip::computeDistance(this->A_vclipModel_,
                                                    A_link->p(),
                                                    A_link->R(),
                                                    this->B_vclipModel_,
                                                    B_link->p(),
                                                    B_link->R(),
                                                    dist,
                                                    A_localp,
                                                    B_localp
                                                    );

    if(solved && dist > 1e-6){
      distance = dist;
      direction = (A_link->T()*A_localp - B_link->T()*B_localp).normalized();
      A_v = A_localp;
      B_v = B_localp;

      this->prev_dist_ = dist;
      this->prev_direction_ = direction;
      this->prev_A_localp_ = A_localp;
      this->prev_B_localp_ = B_localp;
    }else{
      // 干渉時は近傍点が正しくない場合があるので、干渉直前の値を使う
      distance = this->prev_dist_;
      direction = this->prev_direction_;
      A_v = this->prev_A_localp_;
      B_v = this->prev_B_localp_;
    }

    return true;
  }
}
