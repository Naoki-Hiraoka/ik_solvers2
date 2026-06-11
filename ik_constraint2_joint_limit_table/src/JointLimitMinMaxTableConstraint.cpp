#include <ik_constraint2_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <iostream>

namespace ik_constraint2_joint_limit_table{
  void JointLimitMinMaxTableConstraint::calcMinMaxIneq(Eigen::VectorXd& maxIneq, Eigen::VectorXd& minIneq){
    if(minIneq.rows() != 1) minIneq = Eigen::VectorXd::Zero(1);
    if(maxIneq.rows() != 1) maxIneq = Eigen::VectorXd::Zero(1);

    if(!this->joint_ || !(this->joint_->isRevoluteJoint() || this->joint_->isPrismaticJoint())) {
      return;
    }

    double lower = this->joint_->q_lower();
    double upper = this->joint_->q_upper();
    for(size_t i=0;i<this->jointLimitTables_.size();i++){
      if(this->jointLimitTables_[i]->getSelfJoint() == this->joint_){
        lower = std::max(lower, this->jointLimitTables_[i]->getLlimit());
        upper = std::min(upper, this->jointLimitTables_[i]->getUlimit());
      }
    }
    lower += std::min(this->tolerance_, (upper - lower) / 2.0);
    upper -= std::min(this->tolerance_, (upper - lower) / 2.0);
    lower -= this->joint_->q();
    upper -= this->joint_->q();

    minIneq[0] = std::min(this->weight_ * lower, this->maxError_);
    maxIneq[0] = std::max(this->weight_ * upper, -this->maxError_);

    // distance計算用
    this->current_lower_ = lower;
    this->current_upper_ = upper;

    return;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> JointLimitMinMaxTableConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<JointLimitMinMaxTableConstraint> ret = std::make_shared<JointLimitMinMaxTableConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void JointLimitMinMaxTableConstraint::copy(std::shared_ptr<JointLimitMinMaxTableConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    JointLimitConstraint::copy(ret, modelMap);
    for(int i=0;i<ret->jointLimitTables().size();i++){
      ret->jointLimitTables()[i] = ret->jointLimitTables()[i]->clone(modelMap);
    }
  }


}
