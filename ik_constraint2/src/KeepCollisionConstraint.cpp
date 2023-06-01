#include <iostream>

#include <ik_constraint2/KeepCollisionConstraint.h>
#include <ik_constraint2/Jacobian.h>
#include <cnoid/EigenUtil>

namespace ik_constraint2{
  void KeepCollisionConstraint::updateBounds () {
    // minIneq/maxIneqの計算

    if(this->A_link_ == nullptr ||
       this->B_link_ == nullptr ||
       !this->computeCommonPoint(this->A_link_,
                                 this->B_link_,
                                 this->currentp_,
                                 this->currentDistance_,
                                 this->A_currentC_,
                                 this->A_currentdl_,
                                 this->A_currentdu_,
                                 this->B_currentC_,
                                 this->B_currentdl_,
                                 this->B_currentdu_
                                 )){
      this->currentDistance_ = - this->ignorePenetration_;
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
    }else{
      if(this->currentDistance_ > - this->ignorePenetration_){
        int numConstraint = this->A_currentC_.rows() + this->B_currentC_.rows();
        if(this->minIneq_.rows()!=numConstraint) this->minIneq_ = Eigen::VectorXd::Zero(numConstraint);
        if(this->maxIneq_.rows()!=numConstraint) this->maxIneq_ = Eigen::VectorXd::Zero(numConstraint);

        Eigen::VectorXd currentA = this->A_currentC_ * (this->A_link_->T().inverse() * this->currentp_);
        Eigen::VectorXd currentB = this->B_currentC_ * (this->B_link_->T().inverse() * this->currentp_);
        this->minIneq_.head(this->A_currentC_.rows()) = (this->A_currentdl_ - currentA).array().max(-this->maxError_).min(this->maxError_) * this->weight_;
        this->maxIneq_.head(this->A_currentC_.rows()) = (this->A_currentdu_ - currentA).array().max(-this->maxError_).min(this->maxError_) * this->weight_;
        this->minIneq_.tail(this->B_currentC_.rows()) = (this->B_currentdl_ - currentB).array().max(-this->maxError_).min(this->maxError_) * this->weight_;
        this->maxIneq_.tail(this->B_currentC_.rows()) = (this->B_currentdu_ - currentA).array().max(-this->maxError_).min(this->maxError_) * this->weight_;
      }else{
        this->minIneq_.resize(0);
        this->maxIneq_.resize(0);
      }
    }

    if(this->debugLevel_>=1){
      std::cerr << "KeepCollisionConstraint " << (this->A_link_ ? this->A_link_->name() : "world") << " - " << (this->B_link_ ? this->B_link_->name() : "world") << std::endl;
      std::cerr << "distance: " << this->currentDistance_ << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }
  }

  void KeepCollisionConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {

    // jacobianIneq_の計算
    // 行列の初期化. 前回とcol形状が変わっていないなら再利用
    if(!IKConstraint::isJointsSame(joints,this->jacobian_joints_)
       || this->A_link_ != this->jacobian_A_link_
       || this->B_link_ != this->jacobian_B_link_){
      this->jacobian_joints_ = joints;
      this->jacobian_A_link_ = this->A_link_;
      this->jacobian_B_link_ = this->B_link_;

      ik_constraint2::calc6DofJacobianShape(this->jacobian_joints_,//input
                                            this->jacobian_A_link_,//input
                                            this->jacobian_A_full_,
                                            this->jacobianColMap_,
                                            this->path_A_joints_
                                            );
      ik_constraint2::calc6DofJacobianShape(this->jacobian_joints_,//input
                                            this->jacobian_B_link_,//input
                                            this->jacobian_B_full_,
                                            this->jacobianColMap_,
                                            this->path_B_joints_
                                            );
    }

    if(this->currentDistance_ <= - this->ignorePenetration_){
      // this->jacobian_, this->jacobianIneq_のサイズだけそろえる
      this->jacobian_.resize(0,this->jacobian_A_full_.cols());
      this->jacobianIneq_.resize(0,this->jacobian_A_full_.cols());
      this->jacobianExt_.resize(0,0);
      this->jacobianIneqExt_.resize(0,0);
    }else{
      this->jacobian_.resize(0,this->jacobian_A_full_.cols());
      this->jacobianIneq_.resize(this->A_currentC_.rows() + this->B_currentC_.rows(),this->jacobian_A_full_.cols());
      this->jacobianExt_.resize(0,3);
      this->jacobianIneqExt_.resize(this->A_currentC_.rows() + this->B_currentC_.rows(),3);

      Eigen::SparseMatrix<double,Eigen::RowMajor> A_R_sparse(3,3);
      for(int i=0;i<3;i++) for(int j=0;j<3;j++) A_R_sparse.insert(i,j) = this->A_link_->R()(i,j);
      this->jacobian_A_local_ = - A_R_sparse.transpose() * this->jacobian_A_full_.topRows<3>();
      this->jacobian_A_ext_local_ = A_R_sparse.transpose();
      this->jacobian_A_local_ += IKConstraint::cross(this->A_link_->T().inverse() * this->currentp_) * A_R_sparse.transpose() * this->jacobian_A_full_.bottomRows<3>();

      Eigen::SparseMatrix<double,Eigen::RowMajor> B_R_sparse(3,3);
      for(int i=0;i<3;i++) for(int j=0;j<3;j++) B_R_sparse.insert(i,j) = this->B_link_->R()(i,j);
      this->jacobian_B_local_ = - B_R_sparse.transpose() * this->jacobian_B_full_.topRows<3>();
      this->jacobian_B_ext_local_ = B_R_sparse.transpose();
      this->jacobian_B_local_ += IKConstraint::cross(this->B_link_->T().inverse() * this->currentp_) * B_R_sparse.transpose() * this->jacobian_B_full_.bottomRows<3>();

      this->jacobianIneq_.topRows(this->A_currentC_.rows()) = this->weight_ * this->A_currentC_ * this->jacobian_A_local_;
      this->jacobianIneq_.bottomRows(this->B_currentC_.rows()) = this->weight_ * this->B_currentC_ * this->jacobian_B_local_;
      this->jacobianIneqExt_.topRows(this->A_currentC_.rows()) = this->weight_ * this->jacobian_A_ext_local_;
      this->jacobianIneqExt_.bottomRows(this->B_currentC_.rows()) = this->weight_ * this->jacobian_B_ext_local_;

    }

    if(this->debugLevel_>=1){
      std::cerr << "KeepCollisionConstraint " << (this->A_link_ ? this->A_link_->name() : "world") << " - " << (this->B_link_ ? this->B_link_->name() : "world") << std::endl;
      std::cerr << "jacobianineq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
      std::cerr << "jacobianineqExt" << std::endl;
      std::cerr << this->jacobianIneqExt_ << std::endl;
    }

    return;
  }

  bool KeepCollisionConstraint::isSatisfied() const{
    return this->currentDistance_+this->tolerance_ < this->precision_;
  }

  double KeepCollisionConstraint::distance() const{
    return std::abs(std::max(this->currentDistance_+this->tolerance_, 0.0)) * this->weight_;
  }

  double KeepCollisionConstraint::margin() const{
    return - (this->currentDistance_+this->tolerance_) * this->weight_;
  }

  std::vector<cnoid::SgNodePtr>& KeepCollisionConstraint::getDrawOnObjects(){
    return this->drawOnObjects_;
  }

  void KeepCollisionConstraint::copy(std::shared_ptr<KeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(this->A_link_ && modelMap.find(this->A_link_->body()) != modelMap.end()) ret->A_link() = modelMap.find(this->A_link_->body())->second->link(this->A_link_->index());
    if(this->B_link_ && modelMap.find(this->B_link_->body()) != modelMap.end()) ret->B_link() = modelMap.find(this->B_link_->body())->second->link(this->B_link_->index());
  }

}
