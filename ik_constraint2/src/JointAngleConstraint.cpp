#include <ik_constraint2/JointAngleConstraint.h>
#include <iostream>

namespace ik_constraint2{
  void JointAngleConstraint::updateBounds () {
    if(!this->joint_) {
      std::cerr << "[JointAngleConstraint::update] !this->joint_" << std::endl;
      return;
    }

    double error = this->targetq_ - this->joint_->q(); // target - joint

    // this->eqの計算 target - joint
    if(this->eq_.rows() != 1) this->eq_ = Eigen::VectorXd(1);
    this->eq_[0] = this->weight_ * std::min(std::max(error,-this->maxError_), this->maxError_);

    // distance計算用
    this->current_error_ = error;

    if(this->debugLevel_>=1){
      std::cerr << "JointAngleConstraint" << std::endl;
      std::cerr << "q" << std::endl;
      std::cerr << this->joint_->q() << std::endl;
      std::cerr << "targetq" << std::endl;
      std::cerr << this->targetq_ << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
    }

  }

  void JointAngleConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
    if(!this->joint_) {
      std::cerr << "[JointAngleConstraint::update] !this->joint_" << std::endl;
      return;
    }

    // this->jacobian_の計算
    if(!IKConstraint::isJointsSame(joints,this->jacobian_joints_) ||
       this->joint_ != this->jacobian_joint_){
      this->jacobian_joints_ = joints;
      this->jacobian_joint_ = this->joint_;
      this->jacobianColMap_.clear();
      int cols = 0;
      for(size_t i=0; i < this->jacobian_joints_.size(); i++){
        this->jacobianColMap_[this->jacobian_joints_[i]] = cols;
        cols += IKConstraint::getJointDOF(this->jacobian_joints_[i]);
      }

      this->jacobian_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,cols);

      if(this->jacobianColMap_.find(this->jacobian_joint_) != this->jacobianColMap_.end()){
        if(this->jacobian_joint_->isRotationalJoint() || this->jacobian_joint_->isPrismaticJoint()){
          this->jacobian_.insert(0,this->jacobianColMap_[this->jacobian_joint_]) = 1;
        }
      }

    }

    if(this->jacobianColMap_.find(this->jacobian_joint_) != this->jacobianColMap_.end()){
      if(this->jacobian_joint_->isRotationalJoint() || this->jacobian_joint_->isPrismaticJoint()){
        this->jacobian_.coeffRef(0,this->jacobianColMap_[this->jacobian_joint_]) = this->weight_;
      }
    }

    // this->jacobianIneq_のサイズだけそろえる
    this->jacobianIneq_.resize(0,this->jacobian_.cols());

    if(this->debugLevel_>=1){
      std::cerr << "JointAngleConstraint" << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }

    return;
  }

  bool JointAngleConstraint::isSatisfied() const{
    return this->eq_.norm() <= this->precision_;
  }

  double JointAngleConstraint::distance() const{
    return std::abs(this->current_error_ * this->weight_);
  }

  std::shared_ptr<IKConstraint> JointAngleConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<JointAngleConstraint> ret = std::make_shared<JointAngleConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void JointAngleConstraint::copy(std::shared_ptr<JointAngleConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(this->joint_ && modelMap.find(this->joint_->body()) != modelMap.end()) ret->joint() = modelMap.find(this->joint_->body())->second->link(this->joint_->index());
  }

}
