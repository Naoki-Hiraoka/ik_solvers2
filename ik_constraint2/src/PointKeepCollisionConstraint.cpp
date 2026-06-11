#include <ik_constraint2/PointKeepCollisionConstraint.h>
#include <ik_constraint2/Jacobian.h>
#include <iostream>
#include <cnoid/TimeMeasure>

namespace ik_constraint2{
  void PointKeepCollisionConstraint::updateBounds () {
    cnoid::TimeMeasure timer;
    if(this->debugLevel_>=1) timer.begin();

    // minIneq/maxIneqの計算

    if(this->A_link_ == nullptr ||
       this->B_POINT_.size() == 0){
      std::cerr << "[PointKeepCollisionConstraint::computeCommonPoint] assertion failed" << std::endl;
      return;
    }

    if(this->A_FACE_C_.size() != this->A_FACE_dl_.size() ||
       this->A_FACE_C_.size() != this->A_FACE_du_.size()){
      std::cerr << __FUNCTION__ <<  "model A size mismatch" << this->A_FACE_C_.size() << " " << this->A_FACE_dl_.size() << " " << this->A_FACE_du_.size() << std::endl;
      return;
    }

    if(this->B_POINT_changed_){
      this->B_POINT_mat_.resize(4,this->B_POINT_.size());
      for(int i=0;i<this->B_POINT_.size();i++){
        this->B_POINT_mat_.block<3,1>(0,i) = this->B_POINT_[i];
        this->B_POINT_mat_(3,i) = 1.0;
      }
      this->B_POINT_changed_ = false;
    }

    const Eigen::Isometry3d A_pose = this->A_link_->T(); // world frame
    const Eigen::Isometry3d A_poseInv = A_pose.inverse();
    const Eigen::Isometry3d AtoB = A_poseInv;
    const Eigen::MatrixXd AtoBmat = AtoB.matrix().topRows(3);

    double minDist = std::numeric_limits<double>::max();
    int min_i = 0;
    int min_j = 0;

    this->value.resize(this->A_FACE_C_.size());
    for(int i=0;i<this->A_FACE_C_.size();i++){
      if(this->A_FACE_C_[i].rows()!=0 && this->A_FACE_C_[i].cols()!=3){
        std::cerr << __FUNCTION__ <<  "model A matrix size mismatch" << this->A_FACE_C_[i].rows() << "x" << this->A_FACE_C_[i].cols() << std::endl;
      }
      if(this->A_FACE_C_[i].rows()!= this->A_FACE_dl_[i].rows() ||
         this->A_FACE_C_[i].rows()!= this->A_FACE_du_[i].rows()){
        std::cerr << __FUNCTION__ <<  "model A matrix size mismatch" << this->A_FACE_C_[i].rows() << " " << this->A_FACE_dl_[i].rows() << " " << this->A_FACE_du_[i].rows() << std::endl;
      }
      const Eigen::MatrixXd A_FACE_C = this->A_FACE_C_[i] * AtoBmat; // world frame
      const Eigen::VectorXd A_FACE_du = this->A_FACE_du_[i].array() - this->shrinkA_;
      const Eigen::VectorXd A_FACE_dl = this->A_FACE_dl_[i].array() + this->shrinkA_;
      this->value[i].noalias() = A_FACE_C * this->B_POINT_mat_;
      Eigen::Index minRow, minCol;
      const double dist =
        - (( - (this->value[i].colwise() - A_FACE_du).colwise().maxCoeff() )
           .cwiseMin( (this->value[i].colwise() - A_FACE_dl).colwise().minCoeff() )
           .maxCoeff(&minRow, &minCol));
      if(dist < minDist){
        minDist = dist;
        min_i = i;
        min_j = minCol;
      }
    }

    this->currentDistance_ = minDist;
    this->A_currentC_ = this->A_FACE_C_[min_i];
    this->A_currentdl_ = this->A_FACE_dl_[min_i].array() + this->shrinkA_;
    this->A_currentdu_ = this->A_FACE_du_[min_i].array() - this->shrinkA_;
    this->currentp_ = this->B_POINT_[min_j];

    if(this->currentDistance_ > - this->ignorePenetration_){
      Eigen::VectorXd currentA = this->A_currentC_ * (A_poseInv * this->currentp_);
      this->minIneq_ = (this->A_currentdl_ - currentA).array().min(this->maxError_) * this->weight_;
      this->maxIneq_ = (this->A_currentdu_ - currentA).array().max(-this->maxError_) * this->weight_;
    }else{
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
    }

    if(this->debugLevel_>=1) {
      double time = timer.measure();
      std::cerr << "PointKeepCollisionConstraint::updateBounds time: " << time << "[s]." << std::endl;
    }
    if(this->debugLevel_>=2){
      std::cerr << "PointKeepCollisionConstraint " << (this->A_link_ ? this->A_link_->name() : "world") << std::endl;
      std::cerr << "distance: " << this->currentDistance_ << std::endl;
      std::cerr << "currentp" << std::endl;
      std::cerr << this->currentp_.transpose() << std::endl;
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

  void PointKeepCollisionConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
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
                                           cnoid::Vector3::Zero(),//input
                                           this->jacobianColMap_,//input
                                           this->path_A_joints_,//input
                                           this->jacobian_A_full_
                                           );

      this->jacobian_.resize(0,this->jacobian_A_full_.cols());

      Eigen::SparseMatrix<double,Eigen::RowMajor> A_R_sparse(3,3);
      for(int i=0;i<3;i++) for(int j=0;j<3;j++) A_R_sparse.insert(i,j) = this->A_link_->R()(i,j);
      this->jacobian_A_local_ = - A_R_sparse.transpose() * this->jacobian_A_full_.topRows<3>();
      this->jacobian_A_local_ += IKConstraint::cross(this->A_link_->T().inverse() * this->currentp_) * A_R_sparse.transpose() * this->jacobian_A_full_.bottomRows<3>();

      this->jacobianIneq_ = this->weight_ * this->A_currentC_ * this->jacobian_A_local_;

    }

    if(this->debugLevel_>=1) {
      double time = timer.measure();
      std::cerr << "PointKeepCollisionConstraint::updateJacobian time: " << time << "[s]." << std::endl;
    }
    if(this->debugLevel_>=2){
      std::cerr << "PointKeepCollisionConstraint " << (this->A_link_ ? this->A_link_->name() : "world") << std::endl;
      std::cerr << "jacobianineq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool PointKeepCollisionConstraint::isSatisfied() const{
    return this->currentDistance_ <= this->precision_;
  }

  double PointKeepCollisionConstraint::distance() const{
    return std::abs(std::max(this->currentDistance_, 0.0)) * this->weight_;
  }

  double PointKeepCollisionConstraint::margin() const{
    return - (this->currentDistance_) * this->weight_;
  }

  std::vector<cnoid::SgNodePtr>& PointKeepCollisionConstraint::getDrawOnObjects(){
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

    this->points_->getOrCreateVertices()->at(0) = this->currentp_.cast<cnoid::Vector3f::Scalar>();

    return this->drawOnObjects_;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> PointKeepCollisionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<PointKeepCollisionConstraint> ret = std::make_shared<PointKeepCollisionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void PointKeepCollisionConstraint::copy(std::shared_ptr<PointKeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    ret->A_link() = applyModelMap(this->A_link_, modelMap);
    ret->points_ = nullptr;
  }

}
