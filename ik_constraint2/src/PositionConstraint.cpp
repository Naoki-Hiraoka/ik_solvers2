#include <ik_constraint2/PositionConstraint.h>
#include <ik_constraint2/Jacobian.h>

namespace ik_constraint2{
  void PositionConstraint::updateBounds () {
    const cnoid::Position& A_pos = (this->A_link_) ? this->A_link_->T() * this->A_localpos_ : this->A_localpos_;
    const cnoid::Position& B_pos = (this->B_link_) ? this->B_link_->T() * this->B_localpos_ : this->B_localpos_;

    cnoid::Matrix3d eval_R = (this->eval_link_) ? this->eval_link_->R() * this->eval_localR_ : this->eval_localR_;

    cnoid::Vector6 error; // world frame. A-B
    const cnoid::Vector3 pos_error = A_pos.translation() - B_pos.translation();
    cnoid::Vector3 rot_error = cnoid::Vector3::Zero();
    // 1軸がフリーの場合は、軸と軸がなす角度を見る
    if((this->weight_.tail<3>().array() > 0.0).count() == 2 &&
       ((this->eval_link_ == this->A_link_) || (this->eval_link_ == this->B_link_)) ) {
      cnoid::Vector3 axis; // evalR local
      if(this->weight_[3] == 0.0) axis = cnoid::Vector3::UnitX();
      else if(this->weight_[4] == 0.0) axis = cnoid::Vector3::UnitY();
      else if(this->weight_[5] == 0.0) axis = cnoid::Vector3::UnitZ();

      // A_axisとB_axisを一致させる
      cnoid::Vector3 A_axis; // world frame
      cnoid::Vector3 B_axis; // world frame
      if(this->eval_link_ == this->A_link_){
        A_axis = eval_R * axis;
        B_axis = B_pos.linear() * A_pos.linear().transpose() * A_axis;
      }else{ // this->eval_link_ == this->B_link_
        B_axis = eval_R * axis;
        A_axis = A_pos.linear() * B_pos.linear().transpose() * B_axis;
      }
      Eigen::Vector3d cross = B_axis.cross(A_axis);
      double dot = std::min(1.0,std::max(-1.0,B_axis.dot(A_axis))); // acosは定義域外のときnanを返す
      if(cross.norm()==0){ // 0度 or 180度.
        if(dot == -1){ // 180度
          if(this->weight_[3] == 0.0) rot_error = eval_R * M_PI * cnoid::Vector3::UnitY();
          else if(this->weight_[4] == 0.0) rot_error = eval_R * M_PI * cnoid::Vector3::UnitZ();
          else if(this->weight_[5] == 0.0) rot_error = eval_R * M_PI * cnoid::Vector3::UnitX();
        }else{
          // rot_error.setZero();
        }
      }else{
        double angle = std::acos(dot); // 0~pi
        Eigen::Vector3d axis_ = cross.normalized(); // include sign
        rot_error = angle * axis_;
      }
    }else{
      const cnoid::AngleAxis angleAxis = cnoid::AngleAxis(A_pos.linear() * B_pos.linear().transpose());
      rot_error = angleAxis.angle()*angleAxis.axis();
    }
    error << pos_error , rot_error;

    cnoid::Vector6 error_eval; // eval frame. A-B
    error_eval.head<3>() = eval_R.transpose() * error.head<3>();
    error_eval.tail<3>() = eval_R.transpose() * error.tail<3>();

    // A-Bの目標変位を計算し、this->eq_に入れる
    if(this->eq_.rows()!=(this->weight_.array() > 0.0).count()) this->eq_ = Eigen::VectorXd((this->weight_.array() > 0.0).count());
    int idx=0;
    for(size_t i=0; i<6; i++){
      if(this->weight_[i]>0.0) {
        this->eq_[idx] = std::min(std::max(-error_eval[i],-this->maxError_[i]),this->maxError_[i]) * this->weight_[i];
        idx++;
      }
    }

    // distanceを計算する
    this->current_error_eval_ = error_eval;

    if(this->debugLevel_>=1){
      std::cerr << "PositionConstraint" << std::endl;
      std::cerr << "A_pos" << std::endl;
      std::cerr << A_pos.translation().transpose() << std::endl;
      std::cerr << A_pos.linear() << std::endl;
      std::cerr << "B_pos" << std::endl;
      std::cerr << B_pos.translation().transpose() << std::endl;
      std::cerr << B_pos.linear() << std::endl;
      std::cerr << "error_eval" << std::endl;
      std::cerr << error_eval.transpose() << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
    }

  }

  void PositionConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {

    // this->jacobian_を計算する
    // 行列の初期化. 前回とcol形状が変わっていないなら再利用
    if(!IKConstraint::isJointsSame(joints,this->jacobian_joints_)
       || this->A_link_ != this->jacobian_A_link_
       || this->B_link_ != this->jacobian_B_link_){
      this->jacobian_joints_ = joints;
      this->jacobian_A_link_ = this->A_link_;
      this->jacobian_B_link_ = this->B_link_;

      ik_constraint2::calc6DofJacobianShape(this->jacobian_joints_,//input
                                this->jacobian_A_link_,//input
                                this->jacobian_B_link_,//input
                                this->jacobian_full_,
                                this->jacobianColMap_,
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
                             this->jacobianColMap_,//input
                             this->path_A_joints_,//input
                             this->path_B_joints_,//input
                             this->path_BA_joints_,//input
                             this->path_BA_joints_numUpwardConnections_,//input
                             this->jacobian_full_
                             );

    cnoid::Matrix3d eval_R = (this->eval_link_) ? this->eval_link_->R() * this->eval_localR_ : this->eval_localR_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> eval_R_sparse(3,3);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) eval_R_sparse.insert(i,j) = eval_R(i,j);
    this->jacobian_full_local_.resize(this->jacobian_full_.rows(), this->jacobian_full_.cols());
    this->jacobian_full_local_.topRows<3>() = eval_R_sparse.transpose() * this->jacobian_full_.topRows<3>();
    this->jacobian_full_local_.bottomRows<3>() = eval_R_sparse.transpose() * this->jacobian_full_.bottomRows<3>();

    this->jacobian_.resize((this->weight_.array() > 0.0).count(),this->jacobian_full_local_.cols());
    for(size_t i=0, idx=0;i<6;i++){
      if(this->weight_[i]>0.0) {
        this->jacobian_.row(idx) = this->weight_[i] * this->jacobian_full_local_.row(i);
        idx++;
      }
    }

    // this->jacobianIneq_のサイズだけそろえる
    this->jacobianIneq_.resize(0,this->jacobian_.cols());

    if(this->debugLevel_>=1){
      std::cerr << "PositionConstraint" << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }

    return;
  }

  bool PositionConstraint::isSatisfied () const {
    return this->eq_.norm() < this->precision_;
  }

  double PositionConstraint::distance () const {
    return this->current_error_eval_.cwiseProduct(this->weight_).norm();
  }


  std::vector<cnoid::SgNodePtr>& PositionConstraint::getDrawOnObjects(){
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

    const cnoid::Position& A_pos = (this->A_link_) ? this->A_link_->T() * this->A_localpos_ : this->A_localpos_;
    const cnoid::Position& B_pos = (this->B_link_) ? this->B_link_->T() * this->B_localpos_ : this->B_localpos_;

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

  std::shared_ptr<IKConstraint> PositionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<PositionConstraint> ret = std::make_shared<PositionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void PositionConstraint::copy(std::shared_ptr<PositionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(this->A_link_ && modelMap.find(this->A_link_->body()) != modelMap.end()) ret->A_link() = modelMap.find(this->A_link_->body())->second->link(this->A_link_->index());
    if(this->B_link_ && modelMap.find(this->B_link_->body()) != modelMap.end()) ret->B_link() = modelMap.find(this->B_link_->body())->second->link(this->B_link_->index());
    if(this->eval_link_ && modelMap.find(this->eval_link_->body()) != modelMap.end()) ret->eval_link() = modelMap.find(this->eval_link_->body())->second->link(this->eval_link_->index());
  }
}
