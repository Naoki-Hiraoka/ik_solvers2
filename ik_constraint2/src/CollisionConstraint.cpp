#include <iostream>

#include <ik_constraint2/CollisionConstraint.h>
#include <ik_constraint2/Jacobian.h>
#include <cnoid/EigenUtil>

namespace ik_constraint2{

  CollisionConstraint::CollisionConstraint()
  {
  }

  void CollisionConstraint::update (const std::vector<cnoid::LinkPtr>& joints) {
    // minIneq/maxIneqの計算
    if(this->minIneq_.rows()!=1) this->minIneq_ = Eigen::VectorXd::Zero(1);
    if(this->maxIneq_.rows()!=1) this->maxIneq_ = Eigen::VectorXd::Zero(1);

    if(!this->computeDistance(this->A_link_, this->B_link_,
                              this->currentDistance_, this->currentDirection_, this->A_currentLocalp_,this->B_currentLocalp_)){
      this->minIneq_[0] = -1e10;
      this->maxIneq_[0] = 1e10;
    }else{
      this->minIneq_[0] = std::min((this->tolerance_ - this->currentDistance_) / this->velocityDamper_, this->maxError_) * this->weight_;
      this->maxIneq_[0] = 1e10;
    }

    // jacobianIneq_の計算
    // 行列の初期化. 前回とcol形状が変わっていないなら再利用
    if(!IKConstraint::isJointsSame(joints,this->jacobianineq_joints_)
       || this->A_link_ != this->jacobianineq_A_link_
       || this->B_link_ != this->jacobianineq_B_link_){
      this->jacobianineq_joints_ = joints;
      this->jacobianineq_A_link_ = this->A_link_;
      this->jacobianineq_B_link_ = this->B_link_;

      ik_constraint2::calc6DofJacobianShape(this->jacobianineq_joints_,//input
                                this->jacobianineq_A_link_,//input
                                this->jacobianineq_B_link_,//input
                                this->jacobianineq_full_,
                                this->jacobianineqColMap_,
                                this->path_A_joints_,
                                this->path_B_joints_,
                                this->path_BA_joints_,
                                this->path_BA_joints_numUpwardConnections_
                                );
    }

    cnoid::Position A_localpos = cnoid::Position::Identity();
    A_localpos.translation() = this->A_currentLocalp_;
    cnoid::Position B_localpos = cnoid::Position::Identity();
    B_localpos.translation() = this->B_currentLocalp_;
    ik_constraint2::calc6DofJacobianCoef(this->jacobianineq_joints_,//input
                             this->jacobianineq_A_link_,//input
                             A_localpos,//input
                             this->jacobianineq_B_link_,//input
                             B_localpos,//input
                             this->jacobianineqColMap_,//input
                             this->path_A_joints_,//input
                             this->path_B_joints_,//input
                             this->path_BA_joints_,//input
                             this->path_BA_joints_numUpwardConnections_,//input
                             this->jacobianineq_full_
                             );

    Eigen::SparseMatrix<double,Eigen::RowMajor> dir(3,1);
    for(int i=0;i<3;i++) dir.insert(i,0) = this->currentDirection_[i];
    this->jacobianIneq_ = dir.transpose() * this->jacobianineq_full_.topRows<3>() * this->weight_;


    if(this->debugLevel_>=1){
      std::cerr << "CollisionConstraint " << (this->A_link_ ? this->A_link_->name() : "world") << " - " << (this->B_link_ ? this->B_link_->name() : "world") << std::endl;
      std::cerr << "distance: " << this->currentDistance_ << std::endl;
      std::cerr << "direction" << std::endl;
      std::cerr << dir.transpose() << std::endl;
      std::cerr << "jacobianineq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool CollisionConstraint::isSatisfied() const{
    return this->currentDistance_-this->tolerance_ < this->precision_;
  }

}
