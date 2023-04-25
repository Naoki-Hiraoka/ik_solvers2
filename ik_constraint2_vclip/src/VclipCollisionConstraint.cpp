#include <ik_constraint2_vclip/VclipCollisionConstraint.h>
#include <iostream>
#include <choreonoid_vclip/choreonoid_vclip.h>

namespace ik_constraint2_vclip{
  bool VclipCollisionConstraint::computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) {
    if(A_link == nullptr ||
       B_link == nullptr){
      std::cerr << "[VclipCollisionConstraint::computeDistance] assertion failed" << std::endl;
    }

    if(A_link && A_link != this->A_link_vclipModel_){
      this->A_vclipModel_ = choreonoid_vclip::convertToVClipModel(A_link->collisionShape()); // 内部で凸包を計算しているので、collisionShapeは凸である必要はない
      this->A_link_vclipModel_ = A_link;
    }
    if(B_link && B_link != this->B_link_vclipModel_){
      this->B_vclipModel_ = choreonoid_vclip::convertToVClipModel(B_link->collisionShape()); // 内部で凸包を計算しているので、collisionShapeは凸である必要はない
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

  std::shared_ptr<ik_constraint2::IKConstraint> VclipCollisionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<VclipCollisionConstraint> ret = std::make_shared<VclipCollisionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void VclipCollisionConstraint::copy(std::shared_ptr<VclipCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    CollisionConstraint::copy(ret, modelMap);

    //vclipModelは使いまわす
    if(this->A_link_vclipModel_ && modelMap.find(this->A_link_vclipModel_->body()) != modelMap.end()) ret->A_link_vclipModel() = modelMap.find(this->A_link_vclipModel_->body())->second->link(this->A_link_vclipModel_->index());
    if(this->B_link_vclipModel_ && modelMap.find(this->B_link_vclipModel_->body()) != modelMap.end()) ret->B_link_vclipModel() = modelMap.find(this->B_link_vclipModel_->body())->second->link(this->B_link_vclipModel_->index());
  }

}
