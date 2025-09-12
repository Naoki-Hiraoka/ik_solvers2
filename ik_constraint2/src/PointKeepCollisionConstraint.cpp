#include <ik_constraint2/PointKeepCollisionConstraint.h>
#include <iostream>

namespace ik_constraint2{
  bool PointKeepCollisionConstraint::computeCommonPoint(const cnoid::LinkPtr A_link,
                                                        const cnoid::LinkPtr B_link,
                                                        cnoid::Vector3& p, // common point. world frame
                                                        double& distance, // AとBの距離. 負の値はpenetration depth
                                                        Eigen::SparseMatrix<double,Eigen::RowMajor>& A_C, // ? * 3. linkA local frame. pとAが干渉するためのpの条件
                                                        Eigen::VectorXd& A_dl,
                                                        Eigen::VectorXd& A_du,
                                                        Eigen::SparseMatrix<double,Eigen::RowMajor>& B_C, // ? * 3. linkB local frame. pとBが干渉するためのpの条件
                                                        Eigen::VectorXd& B_dl,
                                                        Eigen::VectorXd& B_du
                                                        )
  {
    if(A_link == nullptr ||
       B_link == nullptr){
      std::cerr << "[PointKeepCollisionConstraint::computeCommonPoint] assertion failed" << std::endl;
      return false;
    }

    if(this->A_FACE_C_.size() != this->A_FACE_dl_.size() ||
       this->A_FACE_C_.size() != this->A_FACE_du_.size()){
      std::cerr << __FUNCTION__ <<  "model A size mismatch" << this->A_FACE_C_.size() << " " << this->A_FACE_dl_.size() << " " << this->A_FACE_du_.size() << std::endl;
      return false;
    }

    const Eigen::Isometry3d A_pose = (A_link) ? A_link->T() : Eigen::Isometry3d::Identity(); // world frame
    const Eigen::Isometry3d A_poseInv = A_pose.inverse();
    const Eigen::Isometry3d B_pose = (B_link) ? B_link->T() : Eigen::Isometry3d::Identity(); // world frame
    const Eigen::Isometry3d AtoB = A_poseInv * B_pose;

    double minDist = std::numeric_limits<double>::max();
    int min_i = 0;
    int min_j = 0;
    for(int i=0;i<this->A_FACE_C_.size();i++){
      if(this->A_FACE_C_[i].rows()!=0 && this->A_FACE_C_[i].cols()!=3){
        std::cerr << __FUNCTION__ <<  "model A matrix size mismatch" << this->A_FACE_C_[i].rows() << "x" << this->A_FACE_C_[i].cols() << std::endl;
      }
      if(this->A_FACE_C_[i].rows()!= this->A_FACE_dl_[i].rows() ||
         this->A_FACE_C_[i].rows()!= this->A_FACE_du_[i].rows()){
        std::cerr << __FUNCTION__ <<  "model A matrix size mismatch" << this->A_FACE_C_[i].rows() << " " << this->A_FACE_dl_[i].rows() << " " << this->A_FACE_du_[i].rows() << std::endl;
      }
      for(int j=0;j<this->B_POINT_.size();j++){
        const Eigen::Vector3d b = AtoB * this->B_POINT_[j]; // linkA local frame
        const Eigen::VectorXd value = this->A_FACE_C_[i] * b;
        double dist = - std::min((this->A_FACE_du_[i] - value).minCoeff()-this->shrinkA_, (value - this->A_FACE_dl_[i]).minCoeff()-this->shrinkA_);
        if(dist < minDist){
          minDist = dist;
          min_i = i;
          min_j = j;
        }
      }
    }

    if(minDist == std::numeric_limits<double>::max()) return false;

    distance = minDist;
    A_C = this->A_FACE_C_[min_i];
    A_dl = this->A_FACE_dl_[min_i].array() + this->shrinkA_;
    A_du = this->A_FACE_du_[min_i].array() - this->shrinkA_;

    if(this->I3_.rows() != 3){
      this->I3_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(3,3);
      for(int i=0;i<3;i++) this->I3_.insert(i,i) = 1.0;
    }
    B_C = this->I3_;
    B_dl = this->B_POINT_[min_j];
    B_du = this->B_POINT_[min_j];
    p = B_pose * this->B_POINT_[min_j];

    return true;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> PointKeepCollisionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<PointKeepCollisionConstraint> ret = std::make_shared<PointKeepCollisionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void PointKeepCollisionConstraint::copy(std::shared_ptr<PointKeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    KeepCollisionConstraint::copy(ret, modelMap);
  }

}
