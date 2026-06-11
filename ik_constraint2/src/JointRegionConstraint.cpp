#include <ik_constraint2/JointRegionConstraint.h>
#include <iostream>

namespace ik_constraint2{
  void JointRegionConstraint::updateBounds () {
    double A_q = (this->A_joint_) ? this->A_joint_->q() + this->A_q_: this->A_q_;
    double B_q = (this->B_joint_) ? this->B_joint_->q() + this->B_q_: this->B_q_;

    double error = A_q - B_q; // A - B
    double upper = this->region_ - error;
    double lower = -this->region_ - error;

    // this->Ineqの計算 A - B
    if(this->maxIneq_.rows() != 1) this->maxIneq_ = Eigen::VectorXd(1);
    this->maxIneq_[0] = this->weight_ * std::max(upper,-this->maxError_);
    if(this->minIneq_.rows() != 1) this->minIneq_ = Eigen::VectorXd(1);
    this->minIneq_[0] = this->weight_ * std::min(lower,this->maxError_);

    // distance計算用
    this->current_lower_ = lower;
    this->current_upper_ = upper;

    if(this->debugLevel_>=1){
      std::cerr << "JointRegionConstraint " << (this->A_joint_?this->A_joint_->name(): std::string("abs")) << " : " << (this->B_joint_?this->B_joint_->name(): std::string("abs")) << std::endl;
      std::cerr << "A_q" << std::endl;
      std::cerr << A_q << std::endl;
      std::cerr << "B_q" << std::endl;
      std::cerr << B_q << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
    }
  }

  void JointRegionConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
    // this->jacobianIneq_の計算
    if(!IKConstraint::isJointsSame(joints,this->jacobian_joints_)
       || this->A_joint_ != this->jacobian_A_joint_
       || this->B_joint_ != this->jacobian_B_joint_){
      this->jacobian_joints_ = joints;
      this->jacobian_A_joint_ = this->A_joint_;
      this->jacobian_B_joint_ = this->B_joint_;
      this->jacobianColMap_.clear();
      int cols = 0;
      for(size_t i=0; i < this->jacobian_joints_.size(); i++){
        this->jacobianColMap_[this->jacobian_joints_[i]] = cols;
        cols += IKConstraint::getJointDOF(this->jacobian_joints_[i]);
      }

      this->jacobianIneq_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,cols);

      if(this->jacobianColMap_.find(this->jacobian_A_joint_) != this->jacobianColMap_.end()){
        if(this->jacobian_A_joint_->isRevoluteJoint() || this->jacobian_A_joint_->isPrismaticJoint()){
          this->jacobianIneq_.insert(0,this->jacobianColMap_[this->jacobian_A_joint_]) = 1;
        }
      }
      if(this->jacobianColMap_.find(this->jacobian_B_joint_) != this->jacobianColMap_.end()){
        if(this->jacobian_B_joint_->isRevoluteJoint() || this->jacobian_B_joint_->isPrismaticJoint()){
          this->jacobianIneq_.insert(0,this->jacobianColMap_[this->jacobian_B_joint_]) = 1;
        }
      }

    }

    if(this->jacobianColMap_.find(this->jacobian_A_joint_) != this->jacobianColMap_.end()){
      if(this->jacobian_A_joint_->isRevoluteJoint() || this->jacobian_A_joint_->isPrismaticJoint()){
        this->jacobianIneq_.coeffRef(0,this->jacobianColMap_[this->jacobian_A_joint_]) = this->weight_;
      }
    }
    if(this->jacobianColMap_.find(this->jacobian_B_joint_) != this->jacobianColMap_.end()){
      if(this->jacobian_B_joint_->isRevoluteJoint() || this->jacobian_B_joint_->isPrismaticJoint()){
        this->jacobianIneq_.coeffRef(0,this->jacobianColMap_[this->jacobian_B_joint_]) = -this->weight_;
      }
    }

    // this->jacobian_のサイズだけそろえる
    this->jacobian_.resize(0,this->jacobian_.cols());

    if(this->debugLevel_>=1){
      std::cerr << "JointRegionConstraint" << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool JointRegionConstraint::isSatisfied() const{
    double cost2=0.0;
    for(int i=0;i<this->minIneq_.size();i++){
      if(this->minIneq_[i] > 0.0) cost2 += std::pow(this->minIneq_[i], 2);
    }
    for(int i=0;i<this->maxIneq_.size();i++){
      if(this->maxIneq_[i] < 0.0) cost2 += std::pow(this->maxIneq_[i], 2);
    }
    return cost2 <= std::pow(this->precision_,2);
  }

  double JointRegionConstraint::distance() const{
    return std::sqrt(std::pow(std::max(this->current_lower_,0.0), 2) + std::pow(std::min(this->current_upper_,0.0), 2))*this->weight_;
  }

  std::shared_ptr<IKConstraint> JointRegionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<JointRegionConstraint> ret = std::make_shared<JointRegionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void JointRegionConstraint::copy(std::shared_ptr<JointRegionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    ret->A_joint() = applyModelMap(this->A_joint_, modelMap);
    ret->B_joint() = applyModelMap(this->B_joint_, modelMap);
  }

}
