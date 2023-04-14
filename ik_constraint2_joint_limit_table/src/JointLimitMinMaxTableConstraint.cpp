#include <ik_constraint2_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <iostream>

namespace ik_constraint2_joint_limit_table{
  void JointLimitMinMaxTableConstraint::calcMinMaxIneq(Eigen::VectorXd& maxIneq, Eigen::VectorXd& minIneq){
    if(minIneq.rows() != 1) minIneq = Eigen::VectorXd::Zero(1);
    if(maxIneq.rows() != 1) maxIneq = Eigen::VectorXd::Zero(1);

    if(!this->joint_ || !(this->joint_->isRotationalJoint() || this->joint_->isPrismaticJoint())) {
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
    lower -= this->joint_->q();
    upper -= this->joint_->q();

    minIneq[0] = std::min(this->weight_ * lower, this->maxError_);
    maxIneq[0] = std::max(this->weight_ * upper, -this->maxError_);

    return;
  }

}
