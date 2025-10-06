#include <ik_constraint2/JointDisplacementConstraint.h>
#include <iostream>

namespace ik_constraint2{
  void JointDisplacementConstraint::updateBounds () {
    if(!this->joint_) {
      std::cerr << "[JointDisplacementConstraint::update] !this->joint_" << std::endl;
      return;
    }

    // this->minIneq_, maxIneq_
    if (this->joint_->isRevoluteJoint() || this->joint_->isPrismaticJoint()) {

      if(this->minIneq_.rows() != 1) this->minIneq_ = Eigen::VectorXd(1);
      this->minIneq_[0] = - this->limit_ * this->weight_;
      if(this->maxIneq_.rows() != 1) this->maxIneq_ = Eigen::VectorXd(1);
      this->maxIneq_[0] = this->limit_ * this->weight_;

    }else if (this->joint_->isFreeJoint()){
      if(this->minIneq_.rows() != 6) this->minIneq_ = Eigen::VectorXd(6);
      for(size_t i=0;i<6;i++) this->minIneq_[i] = - this->limit_ * this->weight_;
      if(this->maxIneq_.rows() != 6) this->maxIneq_ = Eigen::VectorXd(6);
      for(size_t i=0;i<6;i++) this->maxIneq_[i] = this->limit_ * this->weight_;

    }else{
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
    }

    if(this->debugLevel_>=1){
      std::cerr << "JointDisplacementConstraint" << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }

  }

  void JointDisplacementConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
    if(!this->joint_) {
      std::cerr << "[JointDisplacementConstraint::update] !this->joint_" << std::endl;
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
      if (this->joint_->isRevoluteJoint() || this->joint_->isPrismaticJoint()) rows=1;
      else if (this->joint_->isFreeJoint()) rows = 6;
      else rows = 0;

      this->jacobianIneq_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(rows,cols);

      if(this->jacobianineqColMap_.find(this->jacobianineq_joint_) != this->jacobianineqColMap_.end()){
        const int idx = this->jacobianineqColMap_[this->jacobianineq_joint_];
        for(size_t i=0;i<rows;i++){
          this->jacobianIneq_.insert(i,idx+i) = 1;
        }
      }

    }

    if(this->jacobianineqColMap_.find(this->jacobianineq_joint_) != this->jacobianineqColMap_.end()){
      int rows;
      if (this->jacobianineq_joint_->isRevoluteJoint() || this->jacobianineq_joint_->isPrismaticJoint()) rows=1;
      else if (this->jacobianineq_joint_->isFreeJoint()) rows = 6;
      else rows = 0;

      const int idx = this->jacobianineqColMap_[this->jacobianineq_joint_];
      for(size_t i=0;i<rows;i++){
        this->jacobianIneq_.coeffRef(i,idx+i) = this->weight_;
      }
    }

    // this->jacobian_のサイズだけそろえる
    this->jacobian_.resize(0,this->jacobianIneq_.cols());

    if(this->debugLevel_>=1){
      std::cerr << "JointDisplacementConstraint" << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool JointDisplacementConstraint::isSatisfied() const{
    return true;
  }

  double JointDisplacementConstraint::distance() const{
    return 0.0;
  }

  std::shared_ptr<IKConstraint> JointDisplacementConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<JointDisplacementConstraint> ret = std::make_shared<JointDisplacementConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void JointDisplacementConstraint::copy(std::shared_ptr<JointDisplacementConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(this->joint_ && modelMap.find(this->joint_->body()) != modelMap.end()) ret->joint() = modelMap.find(this->joint_->body())->second->link(this->joint_->index());
  }

}
