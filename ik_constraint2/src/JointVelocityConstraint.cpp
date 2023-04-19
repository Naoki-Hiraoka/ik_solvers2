#include <ik_constraint2/JointVelocityConstraint.h>
#include <iostream>

namespace ik_constraint2{
  void JointVelocityConstraint::updateBounds () {
    if(!this->joint_) {
      std::cerr << "[JointVelocityConstraint::update] !this->joint_" << std::endl;
      return;
    }


    // this->minIneq_, maxIneq_
    if (this->joint_->isRotationalJoint() || this->joint_->isPrismaticJoint()) {
      double lower = (this->joint_->dq_lower() - this->joint_->dq()) * dt_;
      double upper = (this->joint_->dq_upper() - this->joint_->dq()) * dt_;

      if(this->minIneq_.rows() != 1) this->minIneq_ = Eigen::VectorXd(1);
      this->minIneq_[0] = std::min(lower, this->maxError_) * this->weight_;
      if(this->maxIneq_.rows() != 1) this->maxIneq_ = Eigen::VectorXd(1);
      this->maxIneq_[0] = std::max(upper, -this->maxError_) * this->weight_;

      // distance計算用
      this->current_lower_ = lower;
      this->current_upper_ = upper;
    }else if (this->joint_->isFreeJoint()){
      cnoid::Vector6 lower, upper;
      lower.head<3>() = (cnoid::Vector3::Ones() * this->joint_->dq_lower() - this->joint_->v()) * dt_;
      lower.tail<3>() = (cnoid::Vector3::Ones() * this->joint_->dq_lower() - this->joint_->w()) * dt_;
      upper.head<3>() = (cnoid::Vector3::Ones() * this->joint_->dq_upper() - this->joint_->v()) * dt_;
      upper.tail<3>() = (cnoid::Vector3::Ones() * this->joint_->dq_upper() - this->joint_->w()) * dt_;

      if(this->minIneq_.rows() != 6) this->minIneq_ = Eigen::VectorXd(6);
      for(size_t i=0;i<6;i++) this->minIneq_[i] = std::min(lower[i], this->maxError_) * this->weight_;
      if(this->maxIneq_.rows() != 6) this->maxIneq_ = Eigen::VectorXd(6);
      for(size_t i=0;i<6;i++) this->maxIneq_[i] = std::max(upper[i], -this->maxError_) * this->weight_;

      // distance計算用
      this->current_lower6_ = lower;
      this->current_upper6_ = upper;
    }

    if(this->debugLevel_>=1){
      std::cerr << "JointVelocityConstraint" << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }

  }

  void JointVelocityConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
    if(!this->joint_) {
      std::cerr << "[JointVelocityConstraint::update] !this->joint_" << std::endl;
      return;
    }

    // this->jacobianIneq_
    if(!IKConstraint::isJointsSame(joints,this->jacobianineq_joints_) ||
       this->joint_ != this->jacobianineq_joint_){
      this->jacobianineq_joints_ = joints;
      this->jacobianineq_joint_ = this->joint_;
      this->jacobianineqColMap_.clear();
      int cols = 0;
      for(size_t i=0; i < this->jacobianineq_joints_.size(); i++){
        this->jacobianineqColMap_[this->jacobianineq_joints_[i]] = cols;
        cols += this->getJointDOF(this->jacobianineq_joints_[i]);
      }

      int rows;
      if (this->joint_->isRotationalJoint() || this->joint_->isPrismaticJoint()) rows=1;
      else if (this->joint_->isFreeJoint()) rows = 6;
      else rows = 0;

      this->jacobianIneq_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(rows,cols);

      if(this->jacobianineqColMap_.find(this->jacobianineq_joint_) != this->jacobianineqColMap_.end()){
        for(size_t i=0;i<rows;i++){
          this->jacobianIneq_.insert(rows,this->jacobianineqColMap_[this->jacobianineq_joint_]+i) = 1;
        }
      }

    }

    if(this->jacobianineqColMap_.find(this->jacobianineq_joint_) != this->jacobianineqColMap_.end()){
      int rows;
      if (this->jacobianineq_joint_->isRotationalJoint() || this->jacobianineq_joint_->isPrismaticJoint()) rows=1;
      else if (this->jacobianineq_joint_->isFreeJoint()) rows = 6;
      else rows = 0;

      for(size_t i=0;i<rows;i++){
        this->jacobianIneq_.coeffRef(i,this->jacobianineqColMap_[this->jacobianineq_joint_]+i) = this->weight_;
      }
    }

    // this->jacobian_のサイズだけそろえる
    this->jacobian_.resize(0,this->jacobianIneq_.cols());


    if(this->debugLevel_>=1){
      std::cerr << "JointVelocityConstraint" << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool JointVelocityConstraint::isSatisfied() const{
    double cost2=0.0;
    for(int i=0;i<this->minIneq_.size();i++){
      if(this->minIneq_[i] > 0.0) cost2 += std::pow(this->minIneq_[i], 2);
    }
    for(int i=0;i<this->maxIneq_.size();i++){
      if(this->maxIneq_[i] < 0.0) cost2 += std::pow(this->maxIneq_[i], 2);
    }
    return cost2 < std::pow(this->precision_,2);
  }

  double JointVelocityConstraint::distance() const{
    if(!this->joint_) return 0.0;
    if (this->joint_->isRotationalJoint() || this->joint_->isPrismaticJoint()) {
      std::sqrt(std::pow(std::max(this->current_lower_,0.0), 2) + std::pow(std::min(this->current_upper_,0.0), 2))*this->weight_;
    }else if (this->joint_->isFreeJoint()){
      std::sqrt(this->current_lower6_.cwiseMax(Eigen::VectorXd::Zero(this->current_lower6_.size())).squaredNorm() + this->current_upper6_.cwiseMin(Eigen::VectorXd::Zero(this->current_upper6_.size())).squaredNorm())*this->weight_;
    }
    return 0.0;
  }

  std::shared_ptr<IKConstraint> JointVelocityConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<JointVelocityConstraint> ret = std::make_shared<JointVelocityConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void JointVelocityConstraint::copy(std::shared_ptr<JointVelocityConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(this->joint_ && modelMap.find(this->joint_->body()) != modelMap.end()) ret->joint() = modelMap.find(this->joint_->body())->second->link(this->joint_->index());
  }

}
