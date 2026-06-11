#include <ik_constraint2/PoseKeepCollisionConstraint.h>
#include <ik_constraint2/Jacobian.h>
#include <iostream>
#include <cnoid/TimeMeasure>

namespace ik_constraint2{
  void PoseKeepCollisionConstraint::updateBounds () {
    cnoid::TimeMeasure timer;
    if(this->debugLevel_>=1) timer.begin();

    // minIneq/maxIneqの計算

    if(this->A_link_ == nullptr ||
       this->B_POSE_.size() == 0){
      std::cerr << "[PoseKeepCollisionConstraint::computeCommonPose] assertion failed" << std::endl;
      return;
    }

    if(this->A_FACE_C_.size() != this->A_FACE_dl_.size() ||
       this->A_FACE_C_.size() != this->A_FACE_du_.size()){
      std::cerr << __FUNCTION__ <<  "model A size mismatch" << this->A_FACE_C_.size() << " " << this->A_FACE_dl_.size() << " " << this->A_FACE_du_.size() << std::endl;
      return;
    }

    const Eigen::Isometry3d A_pose = this->A_link_->T() * this->A_localpos_; // world frame
    const Eigen::Isometry3d A_poseInv = A_pose.inverse();
    const Eigen::Isometry3d AtoB = A_poseInv;

    double minDist = std::numeric_limits<double>::max();
    int min_i = 0;
    int min_j = 0;
    for(int i=0;i<this->A_FACE_C_.size();i++){
      if(this->A_FACE_C_[i].rows()!=0 && this->A_FACE_C_[i].cols()!=6){
        std::cerr << __FUNCTION__ <<  "model A matrix size mismatch" << this->A_FACE_C_[i].rows() << "x" << this->A_FACE_C_[i].cols() << std::endl;
      }
      if(this->A_FACE_C_[i].rows()!= this->A_FACE_dl_[i].rows() ||
         this->A_FACE_C_[i].rows()!= this->A_FACE_du_[i].rows()){
        std::cerr << __FUNCTION__ <<  "model A matrix size mismatch" << this->A_FACE_C_[i].rows() << " " << this->A_FACE_dl_[i].rows() << " " << this->A_FACE_du_[i].rows() << std::endl;
      }
      for(int j=0;j<this->B_POSE_.size();j++){
        const cnoid::Isometry3 b_pose = AtoB * this->B_POSE_[j]; // linkA local frame
        cnoid::Vector6 b;
        b.head<3>() = b_pose.translation();
        cnoid::AngleAxisd bR(b_pose.linear());
        b.tail<3>() = bR.angle() * bR.axis();
        const Eigen::VectorXd value = this->A_FACE_C_[i] * b;
        double dist = - std::min((this->A_FACE_du_[i] - value).minCoeff()-this->shrinkA_, (value - this->A_FACE_dl_[i]).minCoeff()-this->shrinkA_);
        if(dist < minDist){
          minDist = dist;
          min_i = i;
          min_j = j;
        }
      }
    }

    this->currentDistance_ = minDist;
    this->A_currentC_ = this->A_FACE_C_[min_i];
    this->A_currentdl_ = this->A_FACE_dl_[min_i].array() + this->shrinkA_;
    this->A_currentdu_ = this->A_FACE_du_[min_i].array() - this->shrinkA_;
    this->currentp_ = this->B_POSE_[min_j];

    if(this->currentDistance_ > - this->ignorePenetration_){
      cnoid::Isometry3 currentpose = A_poseInv * this->currentp_;
      cnoid::Vector6 currentp;
      currentp.head<3>() = currentpose.translation();
      cnoid::AngleAxisd currentR(currentpose.linear());
      currentp.tail<3>() = currentR.angle() * currentR.axis();
      Eigen::VectorXd currentA = this->A_currentC_ * currentp;
      this->minIneq_ = (this->A_currentdl_ - currentA).array().min(this->maxError_) * this->weight_;
      this->maxIneq_ = (this->A_currentdu_ - currentA).array().max(-this->maxError_) * this->weight_;
    }else{
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
    }

    if(this->debugLevel_>=1) {
      double time = timer.measure();
      std::cerr << "PoseKeepCollisionConstraint::updateBounds time: " << time << "[s]." << std::endl;
    }
    if(this->debugLevel_>=2){
      std::cerr << "PoseKeepCollisionConstraint " << (this->A_link_ ? this->A_link_->name() : "world") << std::endl;
      std::cerr << "distance: " << this->currentDistance_ << std::endl;
      std::cerr << "currentp" << std::endl;
      std::cerr << this->currentp_.translation().transpose() << std::endl;
      std::cerr << this->currentp_.linear() << std::endl;
      std::cerr << "A_currentC" << std::endl;
      std::cerr << this->A_currentC_ << std::endl;
      std::cerr << "A_currentdl" << std::endl;
      std::cerr << this->A_currentdl_ << std::endl;
      std::cerr << "A_currentdu" << std::endl;
      std::cerr << this->A_currentdu_ << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }
  }

  void PoseKeepCollisionConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
    cnoid::TimeMeasure timer;
    if(this->debugLevel_>=1) timer.begin();

    // jacobianIneq_の計算
    // 行列の初期化. 前回とcol形状が変わっていないなら再利用
    if(!IKConstraint::isJointsSame(joints,this->jacobian_joints_)
       || this->A_link_ != this->jacobian_A_link_){
      this->jacobian_joints_ = joints;
      this->jacobian_A_link_ = this->A_link_;

      ik_constraint2::calc6DofJacobianShape(this->jacobian_joints_,//input
                                            this->jacobian_A_link_,//input
                                            this->jacobian_A_full_,
                                            this->jacobianColMap_,
                                            this->path_A_joints_
                                            );
    }

    if(this->currentDistance_ <= - this->ignorePenetration_){
      // this->jacobian_, this->jacobianIneq_のサイズだけそろえる
      this->jacobian_.resize(0,this->jacobian_A_full_.cols());
      this->jacobianIneq_.resize(0,this->jacobian_A_full_.cols());
    }else{
      ik_constraint2::calc6DofJacobianCoef(this->jacobian_joints_,//input
                                           this->jacobian_A_link_,//input
                                           this->A_localpos_.translation(),//input
                                           this->jacobianColMap_,//input
                                           this->path_A_joints_,//input
                                           this->jacobian_A_full_
                                           );

      this->jacobian_.resize(0,this->jacobian_A_full_.cols());
      this->jacobian_A_local_.resize(6, this->jacobian_A_full_.cols());

      const Eigen::Isometry3d A_pose = this->A_link_->T() * this->A_localpos_; // world frame
      const Eigen::Isometry3d A_poseInv = A_pose.inverse();

      Eigen::SparseMatrix<double,Eigen::RowMajor> A_R_sparse(3,3);
      for(int i=0;i<3;i++) for(int j=0;j<3;j++) A_R_sparse.insert(i,j) = A_pose.linear()(i,j);
      this->jacobian_A_local_.topRows<3>() = - A_R_sparse.transpose() * this->jacobian_A_full_.topRows<3>();
      this->jacobian_A_local_.topRows<3>() += IKConstraint::cross(A_poseInv * this->currentp_.translation()) * A_R_sparse.transpose() * this->jacobian_A_full_.bottomRows<3>();
      this->jacobian_A_local_.bottomRows<3>() = - A_R_sparse.transpose() * this->jacobian_A_full_.bottomRows<3>();

      this->jacobianIneq_ = this->weight_ * this->A_currentC_ * this->jacobian_A_local_;

    }

    if(this->debugLevel_>=1) {
      double time = timer.measure();
      std::cerr << "PoseKeepCollisionConstraint::updateJacobian time: " << time << "[s]." << std::endl;
    }
    if(this->debugLevel_>=2){
      std::cerr << "PoseKeepCollisionConstraint " << (this->A_link_ ? this->A_link_->name() : "world") << std::endl;
      std::cerr << "jacobianineq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool PoseKeepCollisionConstraint::isSatisfied() const{
    return this->currentDistance_ <= this->precision_;
  }

  double PoseKeepCollisionConstraint::distance() const{
    return std::abs(std::max(this->currentDistance_, 0.0)) * this->weight_;
  }

  double PoseKeepCollisionConstraint::margin() const{
    return - (this->currentDistance_) * this->weight_;
  }

  std::vector<cnoid::SgNodePtr>& PoseKeepCollisionConstraint::getDrawOnObjects(){
    if(this->points_ == nullptr){
      this->points_ = new cnoid::SgPointSet;
      this->points_->setPointSize(20.0);
      this->points_->getOrCreateColors()->resize(1);
      this->points_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.5,0.0,0.0);
      this->points_->getOrCreateVertices()->resize(1);
      this->points_->colorIndices().resize(1);
      this->points_->colorIndices()[0] = 0.0;

      this->drawOnObjects_ = std::vector<cnoid::SgNodePtr>{this->points_};
    }

    if(this->currentDistance_ <= -this->ignorePenetration_) return this->dummyDrawOnObjects_;

    this->points_->getOrCreateVertices()->at(0) = this->currentp_.translation().cast<cnoid::Vector3f::Scalar>();

    return this->drawOnObjects_;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> PoseKeepCollisionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<PoseKeepCollisionConstraint> ret = std::make_shared<PoseKeepCollisionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void PoseKeepCollisionConstraint::copy(std::shared_ptr<PoseKeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    ret->A_link() = applyModelMap(this->A_link_, modelMap);
    ret->points_ = nullptr;
  }

}
