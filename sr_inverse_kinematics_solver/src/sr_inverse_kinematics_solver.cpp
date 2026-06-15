#include <sr_inverse_kinematics_solver/sr_inverse_kinematics_solver.h>
#include <Eigen/Sparse>
#include <iostream>
#include <iomanip>
#include <Eigen/SparseCholesky>
#include <cnoid/TimeMeasure>

namespace sr_inverse_kinematics_solver {
  inline Eigen::SparseMatrix<double,Eigen::RowMajor> toSelectionMatrix(const Eigen::VectorXd& in)
  {
    Eigen::SparseMatrix<double,Eigen::RowMajor> ret((in.array() > 0.0).count(), in.size());
    if (ret.rows() != 0 && ret.cols() != 0) {
      for (int row=0, col=0; col< in.size(); ++col) {
        if(in(col) > 0.0){
          ret.insert(row, col) = 1;
          row++;
        }
      }
    }
    return ret;
  }

  inline Eigen::SparseMatrix<double,Eigen::RowMajor> toDiagonalMatrix(const Eigen::VectorXd& in)
  {
    Eigen::SparseMatrix<double,Eigen::RowMajor> ret(in.size(), in.size());
    for (int i=0; i< in.size(); ++i) {
      ret.insert(i, i) = in[i];
    }
    return ret;
  }

  inline void updateConstraints(const std::vector<cnoid::LinkPtr>& variables, const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& ikc_list, const IKParam& param, bool updateJacobian=true){
    cnoid::TimeMeasure timer;
    if(param.debugLevel>0) timer.begin();

    for ( int i=0; i<ikc_list.size(); i++ ) {
      ikc_list[i]->updateBounds();
      if(updateJacobian) ikc_list[i]->updateJacobian(variables);
    }

    if(param.debugLevel>0) {
      double time = timer.measure();
      std::cerr << "[SRInverseIK] updateConstraints time: " << time << "[s]." << std::endl;
    }
  }

  inline bool checkConstraintsSatisfied(const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& ikc_list) {
    bool satisfied = true;
    for ( int i=0; i<ikc_list.size(); i++ ) {
      if (!ikc_list[i]->isSatisfied()) satisfied = false;
    }
    return satisfied;
  }


  // 返り値は、convergeしたかどうか
  inline bool solveIKOnce (const std::vector<cnoid::LinkPtr>& variables,
                           const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& ikc_list,
                           const IKParam& param) {
    // Solvability-unconcerned Inverse Kinematics by Levenberg-Marquardt Method [sugihara:RSJ2009]
    // q = q + S_q^T * H^(-1) * g // S_q: selection matrix
    // g = J^T * We * e
    // H = J^T * We * J + Wn
    // Wn = (e^T * We * e * \bar{we} + \bar{wn}) * Wq // Wq: modify to insert dq weight
    // Weは既にIKConstraintクラスのJ,eに含まれている

    cnoid::TimeMeasure timer;
    if(param.debugLevel>0) timer.begin();

    double dim = 0;
    for(size_t i=0;i<variables.size();i++) dim+=ik_constraint2::IKConstraint::getJointDOF(variables[i]);

    cnoid::VectorX dq_weight_all = cnoid::VectorX::Ones(dim);
    if(param.dqWeight.size() == dim) {
      for(int i=0;i<param.dqWeight.size();i++) dq_weight_all[i] *= param.dqWeight[i];
    }
    const Eigen::SparseMatrix<double,Eigen::RowMajor> S_q = toSelectionMatrix(dq_weight_all);
    const size_t VALID_Q_NUM = S_q.rows();
    if (VALID_Q_NUM == 0) return true;

    Eigen::SparseMatrix<double,Eigen::RowMajor> H(VALID_Q_NUM,VALID_Q_NUM);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(VALID_Q_NUM);
    double sumError = 0.0;

    for(size_t i=0;i<ikc_list.size();i++){
      const Eigen::VectorXd& error = ikc_list[i]->getEq();
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian = ikc_list[i]->getJacobian() * S_q.transpose();

      H += Eigen::SparseMatrix<double,Eigen::RowMajor>(jacobian.transpose() * jacobian);
      g += jacobian.transpose() * error;
      sumError += error.squaredNorm();
    }

    const Eigen::VectorXd dq_weight_all_final = S_q * static_cast<Eigen::VectorXd>(dq_weight_all.array() * dq_weight_all.array());
    const double weight = std::min(param.wmax, (sumError * param.we + param.wn));
    Eigen::SparseMatrix<double,Eigen::RowMajor> Wn = weight * toDiagonalMatrix(dq_weight_all_final);
    H += Wn;

    // solve
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
    Eigen::VectorXd result = solver.compute(Eigen::SparseMatrix<double>(H)).solve(g); // result = H.inverse() * g; is slow

#define dbg(var)  std::cout << #var"= " << (var) << std::endl
#define dbgn(var) std::cout << #var"= " << std::endl <<(var) << std::endl
#define dbgv(var) std::cout << #var"= " << (var.transpose()) << std::endl
    if (param.debugLevel >= 1) {
      dbgv(result);
      dbgn(H);
      dbgv(g);
      dbg(sumError);
      dbgn(Wn);
      std::cout<<std::endl;
    }

    if (!result.allFinite()) {
      std::cerr <<"[SRInverseIK] ERROR nan/inf is found" << std::endl;
      return false;
    }
    cnoid::VectorX dq_all = (S_q.transpose() * result).eval();

    size_t idx = 0;
    for(size_t i=0;i<variables.size();i++){
      if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()){
        // update joint angles
        variables[i]->q() += dq_all[idx];
        if(param.enableJointLimit){
          // 不等式制約無しで関節角度上下限チェックだけすると、逆運動学の結果が不正確になってしまうことに注意
          if(variables[i]->q() > variables[i]->q_upper()) variables[i]->q() = variables[i]->q_upper();
          if(variables[i]->q() < variables[i]->q_lower()) variables[i]->q() = variables[i]->q_lower();
        }
      }else if(variables[i]->isFreeJoint()) {
        // update rootlink pos rot
        variables[i]->p() += dq_all.segment<3>(idx);
        if(dq_all.segment<3>(idx+3).norm() != 0){
          variables[i]->R() = cnoid::Matrix3(cnoid::AngleAxis(dq_all.segment<3>(idx+3).norm(), cnoid::Vector3(dq_all.segment<3>(idx+3).normalized())) * cnoid::AngleAxis(variables[i]->R()));
          // 単純に3x3行列の空間でRを操作していると、だんだん数値誤差によって回転行列でなくなってしまう
          //const cnoid::Matrix3 dR = cnoid::Matrix3(cnoid::AngleAxis(dq_all.segment<3>(idx+3).norm(), cnoid::Vector3(dq_all.segment<3>(idx+3).normalized())));
          //variables[i]->R() = (dR * variables[i]->R()).eval();
        }
        if (!variables[i]->R().isUnitary()) {
          std::cerr <<"[SRInverseIK] WARN robot->rootLink()->R is not Unitary, something wrong !" << std::endl;
        }
      }

      idx += ik_constraint2::IKConstraint::getJointDOF(variables[i]);
    }

    if(param.debugLevel>0) {
      double time = timer.measure();
      std::cerr << "[SRInverseIK] solveIKOnce time: " << time << "[s]. norm: " << result.norm() << std::endl;
    }

    return result.norm() < param.convergeThre;
  }

  class InitialJointState {
  public:
    InitialJointState() {}
    InitialJointState(const cnoid::Isometry3& T_): T(T_) {}
    InitialJointState(double q_): q(q_) {}
    cnoid::Isometry3 T;
    double q;
  };

  bool solveIKLoop (const std::vector<cnoid::LinkPtr>& variables,
                    const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& ikc_list,
                    const IKParam& param
                    ) {
    cnoid::TimeMeasure timer;
    if(param.debugLevel>0) timer.begin();

    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<variables.size();i++){
      if(variables[i]->body()) bodies.insert(variables[i]->body());
    }

    // 開始時の関節角度を記憶. 後で速度の計算に使う
    std::unordered_map<cnoid::LinkPtr, InitialJointState> initialJointStateMap;
    for(size_t i=0;i<variables.size();i++){
      if(variables[i]->isFreeJoint()) initialJointStateMap[variables[i]] = InitialJointState(variables[i]->T());
      else if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()) initialJointStateMap[variables[i]] = InitialJointState(variables[i]->q());
      else initialJointStateMap[variables[i]] = InitialJointState();
    }

    if(param.calcVelocity){
      for(size_t i=0;i<variables.size();i++){
        if(variables[i]->isFreeJoint()) {
          cnoid::Isometry3& initialT = initialJointStateMap[variables[i]].T;
          variables[i]->v() = (variables[i]->p() - initialT.translation()) / param.dt;
          cnoid::AngleAxis angleAxis = cnoid::AngleAxis(variables[i]->R() * initialT.linear().transpose());
          variables[i]->w() = angleAxis.angle()*angleAxis.axis() / param.dt;
        }
        else if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()) {
          double initialq = initialJointStateMap[variables[i]].q;
          variables[i]->dq() = (variables[i]->q() - initialq) / param.dt;
        }
      }
    }
    for(std::set<cnoid::BodyPtr>::iterator it=bodies.begin(); it != bodies.end(); it++){
      (*it)->calcForwardKinematics(param.calcVelocity);
      (*it)->calcCenterOfMass();
    }
    updateConstraints(variables, ikc_list, param);

    int loop;
    for(loop=0; loop < param.maxIteration; loop++) {
      bool converged = solveIKOnce(variables, ikc_list, param);

      if(param.calcVelocity){
        for(size_t i=0;i<variables.size();i++){
          if(variables[i]->isFreeJoint()) {
            cnoid::Isometry3& initialT = initialJointStateMap[variables[i]].T;
            variables[i]->v() = (variables[i]->p() - initialT.translation()) / param.dt;
            cnoid::AngleAxis angleAxis = cnoid::AngleAxis(variables[i]->R() * initialT.linear().transpose());
            variables[i]->w() = angleAxis.angle()*angleAxis.axis() / param.dt;
          }
          else if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()) {
            double initialq = initialJointStateMap[variables[i]].q;
            variables[i]->dq() = (variables[i]->q() - initialq) / param.dt;
          }
        }
      }
      for(std::set<cnoid::BodyPtr>::iterator it=bodies.begin(); it != bodies.end(); it++){
        (*it)->calcForwardKinematics(param.calcVelocity);
        (*it)->calcCenterOfMass();
      }
      updateConstraints(variables, ikc_list, param);

      bool terminate = false;
      bool satisfied = false;
      if(loop+1 >= param.maxIteration){
        terminate = true;
        satisfied = checkConstraintsSatisfied(ikc_list);
      }else{
        if(converged) {
          terminate = true;
          satisfied = checkConstraintsSatisfied(ikc_list);
        }
        if(!terminate && loop >= param.minIteration){
          if (checkConstraintsSatisfied(ikc_list)) {
            terminate = true;
            satisfied = true;
          }
        }
      }
      if(terminate){
        if(param.debugLevel > 0) {
          double time = timer.measure();
          std::cerr << "[SRInverseIK] solveIKLoop loop: " << loop << " time: " << time << "[s]." << std::endl;
        }
        return satisfied;
      }
    }
    return true; // never reach
  }
}
