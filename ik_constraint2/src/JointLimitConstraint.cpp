#include <ik_constraint2/JointLimitConstraint.h>
#include <iostream>

namespace ik_constraint2{
  void JointLimitConstraint::updateBounds () {
    if(!this->joint_ || !(this->joint_->isRotationalJoint() || this->joint_->isPrismaticJoint())) {
      std::cerr << "[JointLimitConstraint::update] !this->joint_ || !(this->joint_->isRotationalJoint() || this->joint_->isPrismaticJoint())" << std::endl;
      return;
    }

    this->calcMinMaxIneq(this->maxIneq_, this->minIneq_);

    if(this->debugLevel_>=1){
      std::cerr << "JointLimitConstraint" << std::endl;
      std::cerr << "q" << std::endl;
      std::cerr << this->joint_->q() << std::endl;
      std::cerr << "q_upper" << std::endl;
      std::cerr << this->joint_->q_upper() << std::endl;
      std::cerr << "q_lower" << std::endl;
      std::cerr << this->joint_->q_lower() << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }

  }

  void JointLimitConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
    if(!this->joint_ || !(this->joint_->isRotationalJoint() || this->joint_->isPrismaticJoint())) {
      std::cerr << "[JointLimitConstraint::update] !this->joint_ || !(this->joint_->isRotationalJoint() || this->joint_->isPrismaticJoint())" << std::endl;
      return;
    }

    // this->jacobianIneq_を作る
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

      this->jacobianIneq_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,cols);

      if(this->jacobianineqColMap_.find(this->jacobianineq_joint_) != this->jacobianineqColMap_.end()){
        if(this->jacobianineq_joint_->isRotationalJoint() || this->jacobianineq_joint_->isPrismaticJoint()){
          this->jacobianIneq_.insert(0,this->jacobianineqColMap_[this->jacobianineq_joint_]) = 1;
        }
      }

    }

    if(this->jacobianineqColMap_.find(this->jacobianineq_joint_) != this->jacobianineqColMap_.end()){
      if(this->jacobianineq_joint_->isRotationalJoint() || this->jacobianineq_joint_->isPrismaticJoint()){
        this->jacobianIneq_.coeffRef(0,this->jacobianineqColMap_[this->jacobianineq_joint_]) = this->weight_;
      }
    }

    // this->jacobian_のサイズだけそろえる
    this->jacobian_.resize(0,this->jacobianIneq_.cols());

    if(this->debugLevel_>=1){
      std::cerr << "JointLimitConstraint" << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool JointLimitConstraint::isSatisfied() const{
    double cost2=0.0;
    for(int i=0;i<this->minIneq_.size();i++){
      if(this->minIneq_[i] > 0.0) cost2 += std::pow(this->minIneq_[i], 2);
    }
    for(int i=0;i<this->maxIneq_.size();i++){
      if(this->maxIneq_[i] < 0.0) cost2 += std::pow(this->maxIneq_[i], 2);
    }
    return cost2 < std::pow(this->precision_,2);
  }

  double JointLimitConstraint::distance() const{
    return std::sqrt(std::pow(std::max(this->current_lower_,0.0), 2) + std::pow(std::min(this->current_upper_,0.0), 2))*this->weight_;
  }

  void JointLimitConstraint::calcMinMaxIneq(Eigen::VectorXd& maxIneq, Eigen::VectorXd& minIneq){
    double lower = this->joint_->q_lower() - this->joint_->q();
    double upper = this->joint_->q_upper() - this->joint_->q();

    // this->minIneq_, this->maxIneq_を作る
    if(minIneq.rows() != 1) minIneq = Eigen::VectorXd(1);
    minIneq[0] = std::min(lower, this->maxError_) * this->weight_;
    if(maxIneq.rows() != 1) maxIneq = Eigen::VectorXd(1);
    maxIneq[0] = std::max(upper, -this->maxError_) * this->weight_;

    // distance計算用
    this->current_lower_ = lower;
    this->current_upper_ = upper;

    return;
  }

}
