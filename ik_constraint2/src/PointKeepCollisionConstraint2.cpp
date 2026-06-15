#include <ik_constraint2/PointKeepCollisionConstraint2.h>
#include <ik_constraint2/Jacobian.h>
#include <iostream>
#include <cnoid/TimeMeasure>
#include <boost/container_hash/hash.hpp>

namespace ik_constraint2{

  size_t Vector3ihash(const Eigen::Vector3i& v) {
    size_t seed = 0;
    for(int i=0;i<3;i++) boost::hash_combine(seed, v[i]);
    return seed;
  }

  Eigen::Vector3i PointContainer::key(const Eigen::Vector3d& point){
    return ((point - this->origin) / this->resolution).array().floor().cast<int>();
  }

  void PointKeepCollisionConstraint2::updateBounds () {
    cnoid::TimeMeasure timer;
    if(this->debugLevel_>=1) timer.begin();

    // minIneq/maxIneqの計算

    if(this->A_link_ == nullptr ||
       this->B_POINT_ == nullptr ||
       this->B_POINT_->points.size() == 0
       ){
      std::cerr << "[PointKeepCollisionConstraint2::computeCommonPoint] assertion failed" << std::endl;
      return;
    }

    if(this->A_FACE_C_.size() != this->A_FACE_dl_.size() ||
       this->A_FACE_C_.size() != this->A_FACE_du_.size()){
      std::cerr << __FUNCTION__ <<  "model A size mismatch" << this->A_FACE_C_.size() << " " << this->A_FACE_dl_.size() << " " << this->A_FACE_du_.size() << std::endl;
      return;
    }

    const Eigen::Isometry3d A_pose = this->A_link_->T(); // world frame

    this->B_POINT_mat_.clear();
    {
      Eigen::Vector3d center = A_pose * this->A_bounding_sphere_center_;
      Eigen::Vector3i minKey = this->B_POINT_->key(center.array() - this->A_bounding_sphere_radius_-this->giveupDistance_);
      Eigen::Vector3i maxKey = this->B_POINT_->key(center.array() + this->A_bounding_sphere_radius_+this->giveupDistance_);
      for(int x=minKey[0];x<=maxKey[0];x++){
        for(int y=minKey[1];y<=maxKey[1];y++){
          for(int z=minKey[2];z<=maxKey[2];z++){
            Eigen::Vector3i key(x,y,z);
            std::unordered_map<Eigen::Vector3i, std::vector<Eigen::Vector3d> >::iterator it = this->B_POINT_->points.find(key);
            if(it!=this->B_POINT_->points.end()){
              this->B_POINT_mat_.insert(this->B_POINT_mat_.end(), it->second.begin(), it->second.end());
            }
          }
        }
      }
    }

    if(this->B_POINT_mat_.size() == 0){
      this->currentDistance_ = std::numeric_limits<double>::max();
      this->A_currentC_.resize(0,3);
      this->A_currentdl_.resize(0);
      this->A_currentdu_.resize(0);
      this->currentp_.setZero();
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);

    }else{

      Eigen::MatrixXd B_POINT_mat(4,this->B_POINT_mat_.size());
      for(int i=0;i<this->B_POINT_mat_.size();i++){
        B_POINT_mat.block<3,1>(0,i) = this->B_POINT_mat_[i];
        B_POINT_mat(3,i) = 1.0;
      }

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
        this->value[i].noalias() = A_FACE_C * B_POINT_mat;
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
      this->currentp_ = this->B_POINT_mat_[min_j];

      if(this->currentDistance_ > - this->ignorePenetration_){
        Eigen::VectorXd currentA = this->A_currentC_ * (A_poseInv * this->currentp_);
        this->minIneq_ = (this->A_currentdl_ - currentA).array().min(this->maxError_) * this->weight_;
        this->maxIneq_ = (this->A_currentdu_ - currentA).array().max(-this->maxError_) * this->weight_;
      }else{
        this->minIneq_.resize(0);
        this->maxIneq_.resize(0);
      }
    }

    if(this->debugLevel_>=1) {
      double time = timer.measure();
      std::cerr << "PointKeepCollisionConstraint2::updateBounds time: " << time << "[s]." << std::endl;
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

  void PointKeepCollisionConstraint2::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
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

    if(this->currentDistance_ <= - this->ignorePenetration_ ||
       this->currentDistance_ == std::numeric_limits<double>::max()){
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
      std::cerr << "PointKeepCollisionConstraint2::updateJacobian time: " << time << "[s]." << std::endl;
    }
    if(this->debugLevel_>=2){
      std::cerr << "PointKeepCollisionConstraint " << (this->A_link_ ? this->A_link_->name() : "world") << std::endl;
      std::cerr << "jacobianineq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool PointKeepCollisionConstraint2::isSatisfied() const{
    return this->currentDistance_ <= this->precision_;
  }

  double PointKeepCollisionConstraint2::distance() const{
    return std::abs(std::max(this->currentDistance_, 0.0)) * this->weight_;
  }

  double PointKeepCollisionConstraint2::margin() const{
    return - (this->currentDistance_) * this->weight_;
  }

  std::vector<cnoid::SgNodePtr>& PointKeepCollisionConstraint2::getDrawOnObjects(){
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

  std::shared_ptr<ik_constraint2::IKConstraint> PointKeepCollisionConstraint2::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<PointKeepCollisionConstraint2> ret = std::make_shared<PointKeepCollisionConstraint2>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void PointKeepCollisionConstraint2::copy(std::shared_ptr<PointKeepCollisionConstraint2> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    ret->A_link() = applyModelMap(this->A_link_, modelMap);
    ret->points_ = nullptr;
  }

}
