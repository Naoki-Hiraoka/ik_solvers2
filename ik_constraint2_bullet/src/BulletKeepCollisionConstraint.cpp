#include <ik_constraint2_bullet/BulletKeepCollisionConstraint.h>
#include <iostream>
#include <choreonoid_bullet/choreonoid_bullet.h>

namespace ik_constraint2_bullet{
  bool BulletKeepCollisionConstraint::computeCommonPoint(const cnoid::LinkPtr A_link,
                                                         const cnoid::LinkPtr B_link,
                                                         cnoid::Vector3& p, // common point. world frame
                                                         double& distance, // AとBの距離. 負の値はpenetration depth
                                                         Eigen::SparseMatrix<double,Eigen::RowMajor>& A_C, // ? * 3. linkA local frame. pとAが干渉するためのpの条件
                                                         Eigen::VectorXd& A_dl,
                                                         Eigen::VectorXd& A_du,
                                                         Eigen::SparseMatrix<double,Eigen::RowMajor>& B_C, // ? * 3. linkA local frame. pとBが干渉するためのpの条件
                                                         Eigen::VectorXd& B_dl,
                                                         Eigen::VectorXd& B_du
                                                         )
  {
    if(A_link == nullptr ||
       B_link == nullptr){
      std::cerr << "[BulletKeepCollisionConstraint::computeCommonPoint] assertion failed" << std::endl;
      return false;
    }

    if(A_link && A_link != this->A_link_bulletModel_){
      if(this->useSingleMeshA_) {
        std::shared_ptr<btConvexShape> m = choreonoid_bullet::convertToBulletModel(A_link->collisionShape()); // 内部で凸包を計算しているので、collisionShapeは凸である必要はない
        if(m) this->A_bulletModel_ = std::vector<std::shared_ptr<btConvexShape> >{m};
        else this->A_bulletModel_.clear();
      }else{
        this->A_bulletModel_ = choreonoid_bullet::convertToBulletModels(A_link->collisionShape()); // 内部で凸包を計算しているので、collisionShapeは凸である必要はない
      }
      this->A_link_bulletModel_ = A_link;
    }

    if(B_link && B_link != this->B_link_bulletModel_){
      if(this->useSingleMeshB_) {
        std::shared_ptr<btConvexShape> m = choreonoid_bullet::convertToBulletModel(B_link->collisionShape()); // 内部で凸包を計算しているので、collisionShapeは凸である必要はない
        if(m) this->B_bulletModel_ = std::vector<std::shared_ptr<btConvexShape> >{m};
        else this->B_bulletModel_.clear();
      }else{
        this->B_bulletModel_ = choreonoid_bullet::convertToBulletModels(B_link->collisionShape()); // 内部で凸包を計算しているので、collisionShapeは凸である必要はない
      }
      this->B_link_bulletModel_ = B_link;
    }

    if(this->A_bulletModel_.size() != this->A_FACE_C_.size() ||
       this->A_bulletModel_.size() != this->A_FACE_dl_.size() ||
       this->A_bulletModel_.size() != this->A_FACE_du_.size()){
      std::cerr << __FUNCTION__ <<  "model A size mismatch" << this->A_bulletModel_.size() << " " << this->A_FACE_C_.size() << " " << this->A_FACE_dl_.size() << " " << this->A_FACE_du_.size() << std::endl;
      return false;
    }
    if(this->B_bulletModel_.size() != this->B_FACE_C_.size() ||
       this->B_bulletModel_.size() != this->B_FACE_dl_.size() ||
       this->B_bulletModel_.size() != this->B_FACE_du_.size()){
      std::cerr << __FUNCTION__ <<  "model B size mismatch" << this->B_bulletModel_.size() << " " << this->B_FACE_C_.size() << " " << this->B_FACE_dl_.size() << " " << this->B_FACE_du_.size() << std::endl;
      return false;
    }

    double minDist = std::numeric_limits<double>::max();
    int min_i = 0;
    int min_j = 0;
    cnoid::Vector3 A_p, B_p; // world frame
    for(int i=0;i<this->A_bulletModel_.size();i++){
      for(int j=0;j<this->B_bulletModel_.size();j++){
        cnoid::Vector3 A_localp, B_localp; // local frame
        double dist;
        // bulletは干渉している場合に正しいpenetrationを返す
        bool solved = choreonoid_bullet::computeDistance(this->A_bulletModel_[i],
                                                         A_link->p(),
                                                         A_link->R(),
                                                         this->B_bulletModel_[j],
                                                         B_link->p(),
                                                         B_link->R(),
                                                         dist,
                                                         A_localp,
                                                         B_localp
                                                         );
        if(solved && dist < minDist){
          minDist = dist;
          min_i = 0;
          min_j = j;
          A_p = A_link->T() * A_localp;
          B_p = B_link->T() * B_localp;
        }
      }
    }

    if(minDist == std::numeric_limits<double>::max()) return false;

    distance = minDist;
    A_C = this->A_FACE_C_[min_i];
    A_dl = this->A_FACE_dl_[min_i];
    A_du = this->A_FACE_du_[min_i];
    B_C = this->B_FACE_C_[min_j];
    B_dl = this->B_FACE_dl_[min_j];
    B_du = this->B_FACE_du_[min_j];

    // A_p, B_pのどちらかは両方のメッシュの内部にある
    if(std::min({(A_C * (A_link->T().inverse() * A_p) - A_dl).minCoeff(),
            (A_du - A_C * (A_link->T().inverse() * A_p)).minCoeff(),
            (B_C * (B_link->T().inverse() * A_p) - B_dl).minCoeff(),
            (B_du - B_C * (B_link->T().inverse() * A_p)).minCoeff()})
      <
      std::min({(A_C * (A_link->T().inverse() * B_p) - A_dl).minCoeff(),
            (A_du - A_C * (A_link->T().inverse() * B_p)).minCoeff(),
        (B_C * (B_link->T().inverse() * B_p) - B_dl).minCoeff(),
            (B_du - B_C * (B_link->T().inverse() * B_p)).minCoeff()})){
      p = B_p;
    }else{
      p = A_p;
    }

    return true;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> BulletKeepCollisionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<BulletKeepCollisionConstraint> ret = std::make_shared<BulletKeepCollisionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void BulletKeepCollisionConstraint::copy(std::shared_ptr<BulletKeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    KeepCollisionConstraint::copy(ret, modelMap);

    //bulletModelは使いまわす
    if(this->A_link_bulletModel_ && modelMap.find(this->A_link_bulletModel_->body()) != modelMap.end()) ret->A_link_bulletModel() = modelMap.find(this->A_link_bulletModel_->body())->second->link(this->A_link_bulletModel_->index());
    if(this->B_link_bulletModel_ && modelMap.find(this->B_link_bulletModel_->body()) != modelMap.end()) ret->B_link_bulletModel() = modelMap.find(this->B_link_bulletModel_->body())->second->link(this->B_link_bulletModel_->index());
  }

}
