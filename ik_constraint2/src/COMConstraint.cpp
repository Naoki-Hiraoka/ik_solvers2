#include <ik_constraint2/COMConstraint.h>
#include <ik_constraint2/Jacobian.h>

namespace ik_constraint2{
  void COMConstraint::updateBounds () {
    // B - A
    cnoid::Vector3 A_p = this->A_robot_ ? this->A_robot_->centerOfMass() + this->A_localp() : this->A_localp();
    cnoid::Vector3 B_p = this->B_robot_ ? this->B_robot_->centerOfMass() + this->B_localp() : this->B_localp();
    cnoid::Vector3 error = A_p - B_p; // world
    cnoid::Vector3 error_eval = (this->eval_R_.transpose() * error).eval(); // evalR

    // A-Bの目標変位を求める. this->eq_に入れる
    if(this->eq_.rows()!=(this->weight_.array() > 0.0).count()) this->eq_ = Eigen::VectorXd((this->weight_.array() > 0.0).count());
    int idx=0;
    for(size_t i=0; i<3; i++){
      if(this->weight_[i]>0.0) {
        this->eq_[idx] = std::min(std::max(-error_eval[i],-this->maxError_[i]),this->maxError_[i]) * this->weight_[i];
        idx++;
      }
    }

    // distance計算用
    this->current_error_eval_ = error_eval;

    // this->maxIneq_とthis->minIneq_を求める
    if(this->C_.cols() != 3 ||
       this->C_.rows() != this->dl_.size() ||
       this->du_.size() != this->dl_.size()){
      std::cerr << "\x1b[31m" << "[COMConstraint::checkConvergence] dimension mismatch" << "\x1b[39m" << std::endl;
      this->C_.resize(0,3);
      this->du_.resize(0);
      this->dl_.resize(0);
    }
    if(this->C_.rows() != this->maxCError_.size() ){
      this->maxCError_ = cnoid::VectorX::Ones(this->C_.rows()) * 0.1;
    }
    if(this->minIneq_.rows() != this->C_.rows()) this->minIneq_ = Eigen::VectorXd(this->C_.rows());
    if(this->maxIneq_.rows() != this->C_.rows()) this->maxIneq_ = Eigen::VectorXd(this->C_.rows());
    cnoid::VectorX Ce = this->C_ * error_eval;
    cnoid::VectorX u = this->du_ - Ce;
    cnoid::VectorX l = this->dl_ - Ce;
    for(size_t i=0; i<u.size(); i++){
      this->maxIneq_[i] = std::max(u[i],-this->maxCError_[i]);
      this->minIneq_[i] = std::min(l[i],this->maxCError_[i]);
    }

    // distance計算用
    this->current_u_ = u;
    this->current_l_ = l;

    if(this->debugLevel_>=1){
      std::cerr << "COMConstraint" << std::endl;
      std::cerr << "A COM "<<A_p.transpose() << std::endl;
      std::cerr << "B COM "<<B_p.transpose() << std::endl;
      std::cerr << "error_eval" << std::endl;
      std::cerr << error_eval.transpose() << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }

  }

  void COMConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {

    // this->jqcobian_を求める
    // 行列の初期化. 前回とcol形状が変わっていないなら再利用
    if(!IKConstraint::isJointsSame(joints,this->jacobian_joints_)
       || this->A_robot_ != this->jacobian_A_robot_
       || this->B_robot_ != this->jacobian_B_robot_){
      this->jacobian_joints_ = joints;
      this->jacobian_A_robot_ = this->A_robot_;
      this->jacobian_B_robot_ = this->B_robot_;

      ik_constraint2::calcCMJacobianShape(this->jacobian_joints_,
                                          this->jacobian_A_robot_,
                                          this->jacobian_B_robot_,
                                          this->jacobian_full_,
                                          this->jacobianColMap_);
    }

    ik_constraint2::calcCMJacobianCoef(this->jacobian_joints_,
                                       this->jacobian_A_robot_,
                                       this->jacobian_B_robot_,
                                       this->jacobianColMap_,
                                       this->jacobian_full_);

    Eigen::SparseMatrix<double,Eigen::RowMajor> eval_R_sparse(3,3);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) eval_R_sparse.insert(i,j) = this->eval_R_(i,j);
    this->jacobian_full_local_= eval_R_sparse.transpose() * this->jacobian_full_;

    this->jacobian_.resize((this->weight_.array() > 0.0).count(),this->jacobian_full_local_.cols());
    for(size_t i=0, idx=0;i<3;i++){
      if(this->weight_[i]>0.0) {
        this->jacobian_.row(idx) = this->weight_[i] * this->jacobian_full_local_.row(i);
        idx++;
      }
    }

    // this->jacobianIneq_を求める
    this->jacobianIneq_ = this->C_ * this->jacobian_full_local_;

    if(this->debugLevel_>=1){
      std::cerr << "COMConstraint" << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool COMConstraint::isSatisfied() const{
    double cost2=0.0;
    for(int i=0;i<this->minIneq_.size();i++){
      if(this->minIneq_[i] > 0.0) cost2 += std::pow(this->minIneq_[i], 2);
    }
    for(int i=0;i<this->maxIneq_.size();i++){
      if(this->maxIneq_[i] < 0.0) cost2 += std::pow(this->maxIneq_[i], 2);
    }
    return this->eq_.norm() < this->precision_ && cost2 < std::pow(this->CPrecision_,2);
  }

  double COMConstraint::distance() const{
    return std::sqrt(this->current_error_eval_.cwiseProduct(this->weight_).squaredNorm() + this->current_l_.cwiseMax(Eigen::VectorXd::Zero(this->current_l_.size())).squaredNorm() + this->current_u_.cwiseMin(Eigen::VectorXd::Zero(this->current_u_.size())).squaredNorm());
  }

  std::vector<cnoid::SgNodePtr>& COMConstraint::getDrawOnObjects(){
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

    const cnoid::Vector3 A_p = this->A_robot_ ? this->A_robot_->centerOfMass() + this->A_localp() : this->A_localp();
    const cnoid::Vector3 B_p = this->B_robot_ ? this->B_robot_->centerOfMass() + this->B_localp() : this->B_localp();

    this->lines_->getOrCreateVertices()->at(0) = A_p.cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(1) = (A_p + (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(2) = (A_p + (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(3) = (A_p + (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(4) = B_p.cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(5) = (B_p + (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(6) = (B_p + (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(7) = (B_p + (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();

    return this->drawOnObjects_;
  }

  std::shared_ptr<IKConstraint> COMConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<COMConstraint> ret = std::make_shared<COMConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void COMConstraint::copy(std::shared_ptr<COMConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(modelMap.find(this->A_robot_) != modelMap.end()) ret->A_robot() = modelMap.find(this->A_robot_)->second;
    if(modelMap.find(this->B_robot_) != modelMap.end()) ret->B_robot() = modelMap.find(this->B_robot_)->second;
  }

}
