#include <ik_constraint2_bullet/BulletCollisionConstraint.h>
#include <iostream>
#include <choreonoid_bullet/choreonoid_bullet.h>

namespace ik_constraint2_bullet{
  bool BulletCollisionConstraint::computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) {
    if(A_link == nullptr ||
       B_link == nullptr){
      std::cerr << "[BulletCollisionConstraint::computeDistance] assertion failed" << std::endl;
    }

    if(A_link && A_link != this->A_link_bulletModel_){
      if(this->useSingleMesh_) {
        std::shared_ptr<btConvexShape> m = choreonoid_bullet::convertToBulletModel(A_link->collisionShape()); // 内部で凸包を計算しているので、collisionShapeは凸である必要はない
        if(m) this->A_bulletModel_ = std::vector<std::shared_ptr<btConvexShape> >{m};
        else this->A_bulletModel_.clear();
      }else{
        this->A_bulletModel_ = choreonoid_bullet::convertToBulletModels(A_link->collisionShape()); // 内部で凸包を計算しているので、collisionShapeは凸である必要はない
      }
      this->A_link_bulletModel_ = A_link;
    }
    if(B_link && B_link != this->B_link_bulletModel_){
      if(this->useSingleMesh_) {
        std::shared_ptr<btConvexShape> m = choreonoid_bullet::convertToBulletModel(B_link->collisionShape()); // 内部で凸包を計算しているので、collisionShapeは凸である必要はない
        if(m) this->B_bulletModel_ = std::vector<std::shared_ptr<btConvexShape> >{m};
        else this->B_bulletModel_.clear();
      }else{
        this->B_bulletModel_ = choreonoid_bullet::convertToBulletModels(B_link->collisionShape()); // 内部で凸包を計算しているので、collisionShapeは凸である必要はない
      }
      this->B_link_bulletModel_ = B_link;
    }

    cnoid::Vector3 A_localp, B_localp;
    double dist;
    bool solved = choreonoid_bullet::computeDistance(this->A_bulletModel_,
                                                     A_link->p(),
                                                     A_link->R(),
                                                     this->B_bulletModel_,
                                                     B_link->p(),
                                                     B_link->R(),
                                                     dist,
                                                     A_localp,
                                                     B_localp
                                                     );

    if(!solved) return false;

    // bulletは干渉している場合に正しいpenetrationを返す
    distance = dist;
    direction = (A_link->T()*A_localp - B_link->T()*B_localp).normalized();
    if(distance < 0) direction *= -1; // 離れる方向が正
    A_v = A_localp;
    B_v = B_localp;

    this->prev_dist_ = dist;
    this->prev_direction_ = direction;
    this->prev_A_localp_ = A_v;
    this->prev_B_localp_ = B_v;

    return true;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> BulletCollisionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<BulletCollisionConstraint> ret = std::make_shared<BulletCollisionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void BulletCollisionConstraint::copy(std::shared_ptr<BulletCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    CollisionConstraint::copy(ret, modelMap);

    //bulletModelは使いまわす
    if(this->A_link_bulletModel_ && modelMap.find(this->A_link_bulletModel_->body()) != modelMap.end()) ret->A_link_bulletModel() = modelMap.find(this->A_link_bulletModel_->body())->second->link(this->A_link_bulletModel_->index());
    if(this->B_link_bulletModel_ && modelMap.find(this->B_link_bulletModel_->body()) != modelMap.end()) ret->B_link_bulletModel() = modelMap.find(this->B_link_bulletModel_->body())->second->link(this->B_link_bulletModel_->index());
  }

}
