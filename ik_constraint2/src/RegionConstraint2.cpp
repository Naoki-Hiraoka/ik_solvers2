#include <ik_constraint2/RegionConstraint2.h>
#include <ik_constraint2/Jacobian.h>
#include <iostream>

namespace ik_constraint2{
  void RegionConstraint2::updateBounds () {
    const cnoid::Isometry3& A_pos = (this->A_link_) ? this->A_link_->T() * this->A_localpos_ : this->A_localpos_;
    const cnoid::Isometry3& B_pos = (this->B_link_) ? this->B_link_->T() * this->B_localpos_ : this->B_localpos_;

    cnoid::Vector6 error; // world frame. A-B
    error.head<3>() = A_pos.translation() - B_pos.translation();
    const cnoid::AngleAxis angleAxis = cnoid::AngleAxis(A_pos.linear() * B_pos.linear().transpose());
    error.tail<3>() = angleAxis.angle()*angleAxis.axis();

    // 並進について、A-Bの目標変位を計算し、this->minIneq_, this->maxIneq_に入れる
    if(this->weight_ <= 0.0 ||
       this->C_.rows() == 0) {
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
    }else{
      if(this->C_.rows() != this->dl_.rows() ||
         this->C_.rows() != this->du_.rows() ||
         (this->C_.rows() != 0 && this->C_.cols() != 6)) {
        std::cerr << __FUNCTION__ << "dimension mismatch" << std::endl;
        return;
      }
      Eigen::VectorXd current = this->C_ * error;
      this->minIneq_ = (this->dl_-current).array().min(this->maxError_) * this->weight_;
      this->maxIneq_ = (this->du_-current).array().max(-this->maxError_) * this->weight_;
    }

    // distanceを計算する
    this->current_error_ = error;

    if(this->debugLevel_>=1){
      std::cerr << "RegionConstraint2" << std::endl;
      std::cerr << "A_pos" << std::endl;
      std::cerr << A_pos.translation().transpose() << std::endl;
      std::cerr << A_pos.linear() << std::endl;
      std::cerr << "B_pos" << std::endl;
      std::cerr << B_pos.translation().transpose() << std::endl;
      std::cerr << B_pos.linear() << std::endl;
      std::cerr << "error" << std::endl;
      std::cerr << error.transpose() << std::endl;
      std::cerr << "C" << std::endl;
      std::cerr << this->C_ << std::endl;
      std::cerr << "dl" << std::endl;
      std::cerr << this->dl_ << std::endl;
      std::cerr << "du" << std::endl;
      std::cerr << this->du_ << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }

  }

  void RegionConstraint2::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
    if(!IKConstraint::isJointsSame(joints,this->jacobian_joints_)
       || this->A_link_ != this->jacobian_A_link_
       || this->B_link_ != this->jacobian_B_link_){
      this->jacobian_joints_ = joints;
      this->jacobian_A_link_ = this->A_link_;
      this->jacobian_B_link_ = this->B_link_;

      ik_constraint2::calc6DofJacobianShape(this->jacobian_joints_,//input
                                            this->jacobian_A_link_,//input
                                            this->jacobian_B_link_,//input
                                            this->jacobianineq_full_,
                                            this->jacobian_ColMap_,
                                            this->path_A_joints_,
                                            this->path_B_joints_,
                                            this->path_BA_joints_,
                                            this->path_BA_joints_numUpwardConnections_
                                            );

    }

    ik_constraint2::calc6DofJacobianCoef(this->jacobian_joints_,//input
                                         this->jacobian_A_link_,//input
                                         this->A_localpos_,//input
                                         this->jacobian_B_link_,//input
                                         this->B_localpos_,//input
                                         this->jacobian_ColMap_,//input
                                         this->path_A_joints_,//input
                                         this->path_B_joints_,//input
                                         this->path_BA_joints_,//input
                                         this->path_BA_joints_numUpwardConnections_,//input
                                         this->jacobianineq_full_
                                         );

    if(this->weight_ <= 0.0 ||
       this->C_.rows() == 0) {
      this->jacobianIneq_.resize(0,this->jacobianineq_full_.cols());
    }else{
      this->jacobianIneq_ = this->weight_ * this->C_ * this->jacobianineq_full_;
    }

    if(this->debugLevel_>=1){
      std::cerr << "RegionConstraint2" << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;

  }

  bool RegionConstraint2::isSatisfied () const {
    if(this->C_.rows() == 0) return true;
    Eigen::VectorXd current = this->C_ * this->current_error_;
    return std::sqrt((this->dl_-current).array().max(0.0).matrix().squaredNorm() + (this->du_-current).array().min(0.0).matrix().squaredNorm()) <= this->precision_;
  }

  double RegionConstraint2::distance () const {
    if(this->C_.rows() == 0) return 0.0;
    Eigen::VectorXd current = this->C_ * this->current_error_;
    return std::sqrt(((this->dl_-current)*this->weight_).array().max(0.0).matrix().squaredNorm() + ((this->du_-current)*this->weight_).array().min(0.0).matrix().squaredNorm());
  }

  std::vector<cnoid::SgNodePtr>& RegionConstraint2::getDrawOnObjects(){
    if(!this->lines_){
      this->lines_ = new cnoid::SgLineSet;
      this->lines_->setLineWidth(1.0);
      this->lines_->getOrCreateColors()->resize(4);
      this->lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(1.0,1.0,1.0);
      this->lines_->getOrCreateColors()->at(1) = cnoid::Vector3f(1.0,0.0,0.0);
      this->lines_->getOrCreateColors()->at(2) = cnoid::Vector3f(0.0,1.0,0.0);
      this->lines_->getOrCreateColors()->at(3) = cnoid::Vector3f(0.0,0.0,1.0);
      // A, A_x, A_y, A_z, B, B_x, B_y, B_z
      this->lines_->getOrCreateVertices()->resize(8);
      this->lines_->colorIndices().resize(0);
      this->lines_->addLine(0,1); this->lines_->colorIndices().push_back(1); this->lines_->colorIndices().push_back(1);
      this->lines_->addLine(0,2); this->lines_->colorIndices().push_back(2); this->lines_->colorIndices().push_back(2);
      this->lines_->addLine(0,3); this->lines_->colorIndices().push_back(3); this->lines_->colorIndices().push_back(3);
      this->lines_->addLine(4,5); this->lines_->colorIndices().push_back(1); this->lines_->colorIndices().push_back(1);
      this->lines_->addLine(4,6); this->lines_->colorIndices().push_back(2); this->lines_->colorIndices().push_back(2);
      this->lines_->addLine(4,7); this->lines_->colorIndices().push_back(3); this->lines_->colorIndices().push_back(3);
      this->lines_->addLine(0,4); this->lines_->colorIndices().push_back(0); this->lines_->colorIndices().push_back(0);

      this->drawOnObjects_ = std::vector<cnoid::SgNodePtr>{this->lines_};
    }

    const cnoid::Isometry3& A_pos = (this->A_link_) ? this->A_link_->T() * this->A_localpos_ : this->A_localpos_;
    const cnoid::Isometry3& B_pos = (this->B_link_) ? this->B_link_->T() * this->B_localpos_ : this->B_localpos_;

    this->lines_->getOrCreateVertices()->at(0) = A_pos.translation().cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(1) = (A_pos * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(2) = (A_pos * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(3) = (A_pos * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(4) = B_pos.translation().cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(5) = (B_pos * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(6) = (B_pos * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(7) = (B_pos * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();

    return this->drawOnObjects_;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> RegionConstraint2::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<RegionConstraint2> ret = std::make_shared<RegionConstraint2>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void RegionConstraint2::copy(std::shared_ptr<RegionConstraint2> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(this->A_link_ && modelMap.find(this->A_link_->body()) != modelMap.end()) ret->A_link() = modelMap.find(this->A_link_->body())->second->link(this->A_link_->index());
    if(this->B_link_ && modelMap.find(this->B_link_->body()) != modelMap.end()) ret->B_link() = modelMap.find(this->B_link_->body())->second->link(this->B_link_->index());
  }


}
