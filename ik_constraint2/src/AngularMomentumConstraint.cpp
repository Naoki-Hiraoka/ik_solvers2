#include <ik_constraint2/AngularMomentumConstraint.h>
#include <cnoid/Jacobian>

namespace ik_constraint2 {
  namespace cnoid18 {
    // choreonoidのrelease1.7の calcAngularMomentumJacobianにはバグがあり、開発版ではhttps://github.com/s-nakaoka/choreonoid/pull/234 で修正されている. 修正された版の関数(https://github.com/choreonoid/choreonoid/blob/master/src/Body/Jacobian.cpp )を使う
    cnoid::Matrix3d D(cnoid::Vector3d r)
    {
      cnoid::Matrix3d r_cross;
      r_cross <<
        0.0,  -r(2), r(1),
        r(2),    0.0,  -r(0),
        -r(1), r(0),    0.0;
      return r_cross.transpose() * r_cross;
    }

    struct SubMass
    {
      double m;
      cnoid::Vector3 mwc;
      cnoid::Matrix3d Iw;
      SubMass& operator+=(const SubMass& rhs){
        m += rhs.m;
        mwc += rhs.mwc;
        Iw += rhs.Iw;
        return *this;
      }
    };

    void calcSubMass(cnoid::Link* link, std::vector<SubMass>& subMasses, bool calcIw)
    {
      cnoid::Matrix3d R = link->R();
      SubMass& sub = subMasses[link->index()];
      sub.m = link->m();
      sub.mwc = link->m() * link->wc();

      for(cnoid::Link* child = link->child(); child; child = child->sibling()){
        calcSubMass(child, subMasses, calcIw);
        SubMass& childSub = subMasses[child->index()];
        sub.m += childSub.m;
        sub.mwc += childSub.mwc;
      }

      if(calcIw){
        if(sub.m != 0.0) sub.Iw = R * link->I() * R.transpose() + link->m() * D( link->wc() - sub.mwc/sub.m );
        else sub.Iw = R * link->I() * R.transpose();
        for(cnoid::Link* child = link->child(); child; child = child->sibling()){
          SubMass& childSub = subMasses[child->index()];
          if(sub.m != 0.0 && childSub.m != 0.0) sub.Iw += childSub.Iw + childSub.m * D( childSub.mwc/childSub.m - sub.mwc/sub.m );
          else sub.Iw += childSub.Iw;
        }
      }
    }
    void calcAngularMomentumJacobian(cnoid::Body* body, cnoid::Link* base, Eigen::MatrixXd& H)
    {

      // prepare subm, submwc

      const int nj = body->numJoints();
      std::vector<SubMass> subMasses(body->numLinks());
      cnoid::Link* rootLink = body->rootLink();
      std::vector<int> sgn(nj, 1);

      cnoid::MatrixXd M;
      calcCMJacobian( body, base, M );
      M.conservativeResize(3, nj);
      M *= body->mass();

      if(!base){
        calcSubMass(rootLink, subMasses, true);
        H.resize(3, nj + 6);

      } else {
        cnoid::JointPath path(rootLink, base);
        cnoid::Link* skip = path.joint(0);
        SubMass& sub = subMasses[skip->index()];
        sub.m = rootLink->m();
        sub.mwc = rootLink->m() * rootLink->wc();

        for(cnoid::Link* child = rootLink->child(); child; child = child->sibling()){
          if(child != skip){
            calcSubMass(child, subMasses, true);
            sub += subMasses[child->index()];
          }
        }

        // assuming there is no branch between base and root
        for(int i=1; i < path.numJoints(); i++){
          cnoid::Link* joint = path.joint(i);
          const cnoid::Link* parent = joint->parent();
          SubMass& sub = subMasses[joint->index()];
          sub.m = parent->m();
          sub.mwc = parent->m() * parent->wc();
          sub += subMasses[parent->index()];
        }

        H.resize(3, nj);

        for(int i=0; i < path.numJoints(); i++){
          sgn[path.joint(i)->jointId()] = -1;
        }
      }

      // compute Jacobian
      for(int i=0; i < nj; ++i){
        cnoid::Link* joint = body->joint(i);
        if(joint->isRotationalJoint()){
          const cnoid::Vector3 omega = sgn[joint->jointId()] * joint->R() * joint->a();
          const SubMass& sub = subMasses[joint->index()];
          const cnoid::Vector3 Mcol = M.col(joint->jointId());
          cnoid::Vector3 dp;
          if(sub.m != 0.0) dp = (sub.mwc/sub.m).cross(Mcol) + sub.Iw * omega;
          else dp = sub.Iw * omega;
          H.col(joint->jointId()) = dp;
        } else {
          std::cerr << "calcAngularMomentumJacobian() : unsupported jointType("
                    << joint->jointType() << std::endl;
        }
      }

      if(!base){
        const int c = nj;
        H.block(0, c, 3, 3).setZero();
        H.block(0, c+3, 3, 3) = subMasses[rootLink->index()].Iw;

        cnoid::Vector3 cm = body->calcCenterOfMass();
        cnoid::Matrix3d cm_cross;
        cm_cross <<
          0.0,  -cm(2), cm(1),
          cm(2),    0.0,  -cm(0),
          -cm(1), cm(0),    0.0;
        H.block(0,0,3,c) -= cm_cross * M;
      }
    }
  }
}

namespace ik_constraint2{
  void AngularMomentumConstraint::updateBounds () {
    if(this->robot_) {
      Eigen::MatrixXd AMJ;
      cnoid18::calcAngularMomentumJacobian(this->robot_,nullptr,AMJ); // [joint root]の順. comまわり
      cnoid::VectorX dq(this->robot_->numJoints()+6);
      for(int i=0;i<this->robot_->numJoints();i++) dq[i] = this->robot_->joint(i)->dq();
      dq.segment<3>(this->robot_->numJoints()) = this->robot_->rootLink()->v();
      dq.tail<3>() = this->robot_->rootLink()->w();
      cnoid::Vector3 error = this->eval_R_.transpose() * (AMJ * dq - this->targetAngularMomentum_) * this->dt_; //eval_R系  [kg m^2]. target - current

      cnoid::Matrix3 I = AMJ.block<3,3>(0,this->robot_->numJoints()+3); //world系
      cnoid::Matrix3 I_evalR = this->eval_R_.transpose() * I * this->eval_R_; //eval_R系
      cnoid::Matrix3 I_evalR_inv;
      I_evalR_inv <<
        1.0/I_evalR(0,0), 0.0, 0.0,
        0.0, 1.0/I_evalR(1,1), 0.0,
        0.0, 0.0, 1.0/I_evalR(2,2);
      cnoid::Vector3 error_scaled = I_evalR_inv * error; //eval_R系  [rad]

      // this->eq_を求める. target - current
      if(this->eq_.rows() != 3) this->eq_ = Eigen::VectorXd(3);
      for(int i=0;i<3;i++) this->eq_[i] = std::max(std::min(-error_scaled[i], this->maxError_[i]), -this->maxError_[i]) * this->weight_[i];

      // キャッシュ及びdistance計算用
      this->current_I_evalR_inv_ = I_evalR_inv;
      this->current_error_eval_ = error_scaled;

      if(this->debugLevel_>=1){
        std::cerr << "AngularMomentumConstraint" << std::endl;
        std::cerr << "AngularMomentum" << std::endl;
        std::cerr << this->eval_R_.transpose() * AMJ * dq * this->dt_ << std::endl;
        std::cerr << "I_evalR" << std::endl;
        std::cerr << I_evalR << std::endl;
        std::cerr << "error" << std::endl;
        std::cerr << error << std::endl;
        std::cerr << "error_scaled" << std::endl;
        std::cerr << error_scaled << std::endl;
        std::cerr << "eq" << std::endl;
        std::cerr << this->eq_.transpose() << std::endl;
      }
    }else{
      std::cerr << "AngularMomentumConstraint::update() : !this->robot_" << std::endl;
    }
  }

  void AngularMomentumConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
    if(this->robot_) {

      // this->jacobian_を求める
      // 行列の初期化. 前回とcol形状が変わっていないなら再利用
      if(!IKConstraint::isJointsSame(joints,this->jacobian_joints_)
         || this->robot_ != this->jacobian_robot_){
        this->jacobian_joints_ = joints;
        this->jacobian_robot_ = this->robot_;

        AngularMomentumConstraint::calcAngularMomentumJacobianShape(this->jacobian_joints_,
                                                                    this->jacobian_robot_,
                                                                    nullptr,
                                                                    this->jacobian_full_,
                                                                    this->jacobianColMap_);
      }

      AngularMomentumConstraint::calcAngularMomentumJacobianCoef(this->jacobian_joints_,
                                                                 this->jacobian_robot_,
                                                                 nullptr,
                                                                 this->jacobianColMap_,
                                                                 this->jacobian_full_);


      Eigen::SparseMatrix<double,Eigen::RowMajor> eval_R_sparse(3,3);
      for(int i=0;i<3;i++) for(int j=0;j<3;j++) eval_R_sparse.insert(i,j) = this->eval_R_(i,j);
      Eigen::SparseMatrix<double,Eigen::RowMajor> I_evalR_inv_sparse(3,3);
      for(int i=0;i<3;i++) I_evalR_inv_sparse.insert(i,i) = this->current_I_evalR_inv_(i,i);

      this->jacobian_ = I_evalR_inv_sparse * eval_R_sparse.transpose() * this->jacobian_full_;
      for(size_t i=0;i<3;i++) this->jacobian_.row(i) *= this->weight_[i];

      // this->jacobianIneq_のサイズだけそろえる
      this->jacobianIneq_.resize(0,this->jacobian_.cols());

      if(this->debugLevel_>=1){
        std::cerr << "AngularMomentumConstraint" << std::endl;
        std::cerr << "jacobian" << std::endl;
        std::cerr << this->jacobian_ << std::endl;
      }

    }else{
      std::cerr << "AngularMomentumConstraint::update() : !this->robot_" << std::endl;
    }

    return;
  }

  bool AngularMomentumConstraint::isSatisfied() const{
    return this->eq_.norm() <= this->precision_;
  }

  double AngularMomentumConstraint::distance() const{
    return this->current_error_eval_.cwiseProduct(this->weight_).norm();
  }


  void AngularMomentumConstraint::calcAngularMomentumJacobianShape(const std::vector<cnoid::LinkPtr>& joints,
                                                                   const cnoid::BodyPtr& A_robot,
                                                                   const cnoid::BodyPtr& B_robot,
                                                                   Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian,
                                                                   std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap){
    jacobianColMap.clear();
    int cols = 0;
    for(size_t i=0;i<joints.size();i++){
      jacobianColMap[joints[i]] = cols;
      cols += getJointDOF(joints[i]);
    }
    jacobian = Eigen::SparseMatrix<double,Eigen::RowMajor>(3,cols);

    std::vector<Eigen::Triplet<double> > tripletList;
    tripletList.reserve(100);//適当

    if(A_robot != B_robot){
      for(int i=0;i<2;i++){
        cnoid::BodyPtr robot = (i==0) ? A_robot : B_robot;
        if(!robot) continue;
        if(jacobianColMap.find(robot->rootLink()) != jacobianColMap.end()){
          int idx = jacobianColMap[robot->rootLink()];
          for(size_t row=0;row<3;row++){
            if(robot->rootLink()->isFreeJoint()){
              for(size_t j=3;j<getJointDOF(robot->rootLink());j++){//並進は無視
                tripletList.push_back(Eigen::Triplet<double>(row,idx+j,1));
              }
            }else{
              for(size_t j=0;j<getJointDOF(robot->rootLink());j++){
                tripletList.push_back(Eigen::Triplet<double>(row,idx+j,1));
              }
            }
          }
        }
        for(size_t j=0;j<robot->numJoints();j++){
          if(jacobianColMap.find(robot->joint(j)) != jacobianColMap.end()){
            int idx = jacobianColMap[robot->joint(j)];
            for(size_t row=0;row<3;row++){
              for(size_t k=0;k<getJointDOF(robot->joint(j));k++){
                tripletList.push_back(Eigen::Triplet<double>(row,idx+k,1));
              }
            }
          }
        }
      }
    }
    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());
  }

  void AngularMomentumConstraint::calcAngularMomentumJacobianCoef(const std::vector<cnoid::LinkPtr>& joints,//input
                                                                  const cnoid::BodyPtr& A_robot,//input
                                                                  const cnoid::BodyPtr& B_robot,//input
                                                                  std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap, //input
                                                                  Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian//output
                                                                  ) {
    if(A_robot != B_robot){
      for(size_t i=0;i<2; i++){
        cnoid::BodyPtr robot = (i==0) ? A_robot : B_robot;
        if(!robot) continue;
        int sign = (i==0) ? 1 : -1;

        Eigen::MatrixXd AMJ;
        cnoid18::calcAngularMomentumJacobian(robot,nullptr,AMJ); // [joint root]の順. comまわり

        if(jacobianColMap.find(robot->rootLink()) != jacobianColMap.end()){
          int col_idx = jacobianColMap[robot->rootLink()];
          for(size_t j=0;j<3;j++){
            if(robot->rootLink()->isFreeJoint()){
              for(size_t k=3;k<getJointDOF(robot->rootLink());k++){ // 並進は無視
                jacobian.coeffRef(j,col_idx+k) = sign * AMJ(j,robot->numJoints()+k);
              }
            }else{
              for(size_t k=0;k<getJointDOF(robot->rootLink());k++){
                jacobian.coeffRef(j,col_idx+k) = sign * AMJ(j,robot->numJoints()+k);
              }
            }
          }
        }
        for(size_t j=0;j<robot->numJoints();j++){
          if(jacobianColMap.find(robot->joint(j)) != jacobianColMap.end()){
            int col_idx = jacobianColMap[robot->joint(j)];
            for(size_t k=0;k<3;k++){
              for(size_t d=0;d<getJointDOF(robot->joint(j));d++){
                jacobian.coeffRef(k,col_idx+d) = sign * AMJ(k,j+d);
              }
            }
          }
        }
      }
    }
  }

  std::shared_ptr<IKConstraint> AngularMomentumConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<AngularMomentumConstraint> ret = std::make_shared<AngularMomentumConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void AngularMomentumConstraint::copy(std::shared_ptr<AngularMomentumConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(modelMap.find(this->robot_) != modelMap.end()) ret->robot() = modelMap.find(this->robot_)->second;
  }

}
