#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <Eigen/Sparse>
#include <iostream>
#include <iomanip>
#include <set>
#include <unordered_map>
#include <cnoid/TimeMeasure>

namespace prioritized_inverse_kinematics_solver2 {
  inline void updateConstraints(const std::vector<cnoid::LinkPtr>& variables, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& ikc_list, const IKParam& param, bool updateJacobian=true){
    cnoid::TimeMeasure timer;
    if(param.debugLevel>0) timer.begin();

    for ( int i=0; i<ikc_list.size(); i++ ) {
      for(size_t j=0;j<ikc_list[i].size(); j++){
        ikc_list[i][j]->updateBounds();
        if(updateJacobian) ikc_list[i][j]->updateJacobian(variables);
      }
    }

    if(param.debugLevel>0) {
      double time = timer.measure();
      std::cerr << "[PrioritizedIK] updateConstraints time: " << time << "[s]." << std::endl;
    }

  }

  inline bool checkConstraintsSatisfied(const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& ikc_list, int checkLevel = std::numeric_limits<int>::max()) {
    bool satisfied = true;
    for ( int i=0; i<ikc_list.size() && (checkLevel>=0 ? i<=checkLevel : true); i++ ) {
      for(size_t j=0;j<ikc_list[i].size(); j++){
        if (!ikc_list[i][j]->isSatisfied()) satisfied = false;
      }
    }
    return satisfied;
  }

  inline bool checkRejectionsSatisfied(const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections) {
    bool satisfied = true;
    for ( int i=0; i<rejections.size(); i++ ) {
      if (!rejections[i]->isSatisfied()) satisfied = false;
    }
    return satisfied;
  }

  // 返り値は、convergeしたかどうか
  inline bool solveIKOnce (const std::vector<cnoid::LinkPtr>& variables,
                           const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& ikc_list,
                           std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
                           const IKParam& param,
                           std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc) {
    // Solvability-unconcerned Inverse Kinematics by Levenberg-Marquardt Method [sugihara:RSJ2009]
    // H = J^T * We * J + Wn
    // Wn = (e^T * We * e + \bar{wn}) * Wq // Wq: modify to insert dq weight
    // Weは既にIKConstraintクラスのJ,eに含まれている

    cnoid::TimeMeasure timer;
    if(param.debugLevel>0) timer.begin();

    double dim = 0;
    for(size_t i=0;i<variables.size();i++) dim+=ik_constraint2::IKConstraint::getJointDOF(variables[i]);

    if(prevTasks.size() != ikc_list.size()) {
      prevTasks.clear();
      prevTasks.resize(ikc_list.size(),nullptr);
    }
    for(size_t i=0;i<ikc_list.size();i++){
      taskGeneratorFunc(prevTasks[i],param.debugLevel);

      if(i!=0) prevTasks[i]->toSolve() = true;
      else prevTasks[i]->toSolve() = false;

      int num_eqs = 0;
      int num_ineqs = 0;
      int num_exts = 0;
      std::vector<std::reference_wrapper<const Eigen::VectorXd> > errors;errors.reserve(ikc_list[i].size());
      std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobians;jacobians.reserve(ikc_list[i].size());
      std::vector<std::reference_wrapper <const Eigen::VectorXd> > minineqs;minineqs.reserve(ikc_list[i].size());
      std::vector<std::reference_wrapper<const Eigen::VectorXd> > maxineqs;maxineqs.reserve(ikc_list[i].size());
      std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobianineqs;jacobianineqs.reserve(ikc_list[i].size());
      std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobianexts;jacobianexts.reserve(ikc_list[i].size());
      std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobianineqexts;jacobianineqexts.reserve(ikc_list[i].size());

      for(size_t j=0; j<ikc_list[i].size(); j++){
        errors.emplace_back(ikc_list[i][j]->getEq());
        jacobians.emplace_back(ikc_list[i][j]->getJacobian());
        jacobianineqs.emplace_back(ikc_list[i][j]->getJacobianIneq());
        minineqs.emplace_back(ikc_list[i][j]->getMinIneq());
        maxineqs.emplace_back(ikc_list[i][j]->getMaxIneq());
        jacobianexts.emplace_back(ikc_list[i][j]->getJacobianExt());
        jacobianineqexts.emplace_back(ikc_list[i][j]->getJacobianIneqExt());

        num_eqs += errors[j].get().rows();
        num_ineqs += minineqs[j].get().rows();
        num_exts += std::max(jacobianexts[j].get().cols(), jacobianineqexts[j].get().cols());
      }

      prevTasks[i]->A().resize(num_eqs, dim);
      prevTasks[i]->b().resize(num_eqs);
      prevTasks[i]->C().resize(num_ineqs, dim);
      prevTasks[i]->dl().resize(num_ineqs);
      prevTasks[i]->du().resize(num_ineqs);
      prevTasks[i]->wa() = cnoid::VectorXd::Ones(num_eqs);
      prevTasks[i]->wc() = cnoid::VectorXd::Ones(num_ineqs);
      prevTasks[i]->A_ext().resize(num_eqs, num_exts);
      prevTasks[i]->C_ext().resize(num_ineqs, num_exts);

      int idx_eq = 0;
      int idx_ineq = 0;
      int idx_ext = 0;
      for(size_t j=0;j<ikc_list[i].size(); j++){
        prevTasks[i]->A().middleRows(idx_eq,errors[j].get().rows()) = jacobians[j].get();
        prevTasks[i]->b().segment(idx_eq,errors[j].get().rows()) = errors[j].get();

        prevTasks[i]->C().middleRows(idx_ineq,minineqs[j].get().rows()) = jacobianineqs[j].get();
        prevTasks[i]->dl().segment(idx_ineq,minineqs[j].get().rows()) = minineqs[j].get();
        prevTasks[i]->du().segment(idx_ineq,minineqs[j].get().rows()) = maxineqs[j].get();

        Eigen::SparseMatrix<double, Eigen::ColMajor> A_ext_ColMajor(jacobianexts[j].get().rows(),num_exts);
        A_ext_ColMajor.middleCols(idx_ext,jacobianexts[j].get().cols()) = jacobianexts[j].get();
        prevTasks[i]->A_ext().middleRows(idx_eq,A_ext_ColMajor.rows()) = A_ext_ColMajor;
        Eigen::SparseMatrix<double, Eigen::ColMajor> C_ext_ColMajor(jacobianineqexts[j].get().rows(),num_exts);
        C_ext_ColMajor.middleCols(idx_ext,jacobianineqexts[j].get().cols()) = jacobianineqexts[j].get();
        prevTasks[i]->C_ext().middleRows(idx_ineq,C_ext_ColMajor.rows()) = C_ext_ColMajor;

        idx_eq += errors[j].get().rows();
        idx_ineq += minineqs[j].get().rows();
        idx_ext += std::max(jacobianexts[j].get().cols(), jacobianineqexts[j].get().cols());
      }

      double sumError = 0;
      sumError += prevTasks[i]->b().squaredNorm();
      for(size_t j=0;j<prevTasks[i]->dl().size(); j++) {
        if(prevTasks[i]->dl()[j]>0) sumError += std::pow(prevTasks[i]->dl()[j],2);
        if(prevTasks[i]->du()[j]<0) sumError += std::pow(prevTasks[i]->du()[j],2);
      }
      double weight = std::min(((param.wmaxVec.size()==ikc_list.size())?param.wmaxVec[i]:param.wmax), (sumError * ((param.weVec.size()==ikc_list.size())?param.weVec[i]:param.we)+ ((param.wnVec.size()==ikc_list.size())?param.wnVec[i]:param.wn)));
      prevTasks[i]->w() = cnoid::VectorXd::Ones(dim) * weight;
      prevTasks[i]->w_ext() = cnoid::VectorXd::Ones(num_exts) * weight;
      if(param.dqWeight.size() == dim) {
        for(int j=0;j<dim;j++) prevTasks[i]->w()[j] *= param.dqWeight[j];
      }

      if(param.debugLevel>1) prevTasks[i]->name() = std::string("Task") + std::to_string(i);
    }

    // solve
    cnoid::VectorX result;
    if(!prioritized_qp_base::solve(prevTasks, result, param.debugLevel)){
      std::cerr <<"[PrioritizedIK] prioritized_qp_base::solve failed" << std::endl;
      return true;
    }

    if (!result.allFinite()) {
      std::cerr <<"[PrioritizedIK] ERROR nan/inf is found" << std::endl;
      return true;
    }

    size_t idx = 0;
    for(size_t i=0;i<variables.size();i++){
      if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()){
        // update joint angles
        variables[i]->q() += result[idx];
         // 関節角度上下限チェックはしない. JointLimitConstraintを使うこと. JointLimitConstraint無しで関節角度上下限チェックだけすると、逆運動学の結果が不正確になってしまう
        //if(variables[i]->q() > variables[i]->q_upper()) variables[i]->q() = variables[i]->q_upper();
        //if(variables[i]->q() < variables[i]->q_lower()) variables[i]->q() = variables[i]->q_lower();
      }else if(variables[i]->isFreeJoint()) {
        // update rootlink pos rot
        variables[i]->p() += result.segment<3>(idx);
        if(result.segment<3>(idx+3).norm() != 0){
          variables[i]->R() = cnoid::Matrix3(cnoid::AngleAxis(result.segment<3>(idx+3).norm(), cnoid::Vector3(result.segment<3>(idx+3).normalized())) * cnoid::AngleAxis(variables[i]->R()));
          // 単純に3x3行列の空間でRを操作していると、だんだん数値誤差によって回転行列でなくなってしまう
          //const cnoid::Matrix3 dR = cnoid::Matrix3(cnoid::AngleAxis(result.segment<3>(idx+3).norm(), cnoid::Vector3(result.segment<3>(idx+3).normalized())));
          //variables[i]->R() = (dR * variables[i]->R()).eval();
        }
        if (!variables[i]->R().isUnitary()) {
          std::cerr <<"[PrioritizedIK] WARN robot->rootLink()->R is not Unitary, something wrong !" << std::endl;
        }
      }

      idx += ik_constraint2::IKConstraint::getJointDOF(variables[i]);
    }

    if(param.debugLevel>0) {
      double time = timer.measure();
      std::cerr << "[PrioritizedIK] solveIKOnce time: " << time << "[s]. norm: " << result.norm() << std::endl;
    }

    return result.norm() < param.convergeThre;
  }

  class InitialJointState {
  public:
    InitialJointState() {}
    InitialJointState(const cnoid::Position& T_): T(T_) {}
    InitialJointState(double q_): q(q_) {}
    cnoid::Position T;
    double q;
  };

  inline void link2Frame(const std::vector<cnoid::LinkPtr>& links, std::vector<double>& frame){
    frame.clear();
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        frame.push_back(links[l]->q());
      }else if(links[l]->isFreeJoint()) {
        frame.push_back(links[l]->p()[0]);
        frame.push_back(links[l]->p()[1]);
        frame.push_back(links[l]->p()[2]);
        cnoid::Quaternion q(links[l]->R());
        frame.push_back(q.x());
        frame.push_back(q.y());
        frame.push_back(q.z());
        frame.push_back(q.w());
      }
    }
  }

  inline void frame2Link(std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& links){
    int idx = 0;
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        links[l]->q() = frame[idx];
        idx++;
      }else if(links[l]->isFreeJoint()) {
        links[l]->p()[0] = frame[idx+0];
        links[l]->p()[1] = frame[idx+1];
        links[l]->p()[2] = frame[idx+2];
        cnoid::Quaternion q(frame[idx+6],frame[idx+3],frame[idx+4],frame[idx+5]);
        links[l]->R() = q.toRotationMatrix();
        idx+=7;
      }
    }
  }

  bool solveIKLoop (const std::vector<cnoid::LinkPtr>& variables,
                    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& ikc_list,
                    std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
                    const IKParam& param,
                    std::shared_ptr<std::vector<std::vector<double> > > path,
                    std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc){
    return solveIKLoop(variables, ikc_list, std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(), prevTasks, param, path, taskGeneratorFunc);
  }

  bool solveIKLoop (const std::vector<cnoid::LinkPtr>& variables,
                    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& ikc_list,
                    const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections, // これをsatisfyしなくなる直前のstateを返す
                    std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
                    const IKParam& param,
                    std::shared_ptr<std::vector<std::vector<double> > > path,
                    std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc) {

    cnoid::TimeMeasure timer;
    if(param.debugLevel>0) timer.begin();

    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<variables.size();i++){
      if(variables[i]->body()) bodies.insert(variables[i]->body());
    }

    std::unordered_map<cnoid::LinkPtr, InitialJointState> initialJointStateMap;
    for(size_t i=0;i<variables.size();i++){
      if(variables[i]->isFreeJoint()) initialJointStateMap[variables[i]] = InitialJointState(variables[i]->T());
      else if(variables[i]->isRotationalJoint() || variables[i]->isPrismaticJoint()) initialJointStateMap[variables[i]] = InitialJointState(variables[i]->q());
      else initialJointStateMap[variables[i]] = InitialJointState();
    }

    std::vector<double> prevFrame;
    link2Frame(variables, prevFrame);
    if(path != nullptr) {
      path->resize(1);
      path->at(0) = prevFrame;
    }

    if(param.calcVelocity){
      for(size_t i=0;i<variables.size();i++){
        if(variables[i]->isFreeJoint()) {
          cnoid::Position& initialT = initialJointStateMap[variables[i]].T;
          variables[i]->v() = (variables[i]->p() - initialT.translation()) / param.dt;
          cnoid::AngleAxis angleAxis = cnoid::AngleAxis(variables[i]->R() * initialT.linear().transpose());
          variables[i]->w() = angleAxis.angle()*angleAxis.axis() / param.dt;
        }
        else if(variables[i]->isRotationalJoint() || variables[i]->isPrismaticJoint()) {
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

    if(!checkRejectionsSatisfied(rejections)) return false;

    int loop;
    for(loop=0; loop < param.maxIteration; loop++) {
      bool converged = solveIKOnce(variables, ikc_list, prevTasks, param, taskGeneratorFunc);

      if(param.calcVelocity){
        for(size_t i=0;i<variables.size();i++){
          if(variables[i]->isFreeJoint()) {
            cnoid::Position& initialT = initialJointStateMap[variables[i]].T;
            variables[i]->v() = (variables[i]->p() - initialT.translation()) / param.dt;
            cnoid::AngleAxis angleAxis = cnoid::AngleAxis(variables[i]->R() * initialT.linear().transpose());
            variables[i]->w() = angleAxis.angle()*angleAxis.axis() / param.dt;
          }
          else if(variables[i]->isRotationalJoint() || variables[i]->isPrismaticJoint()) {
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

      if(path != nullptr && (loop + 1) % param.pathOutputLoop == 0) {
        path->resize(path->size() + 1);
        link2Frame(variables, path->back());
      }

      bool terminate = false;
      bool satisfied = false;
      if(!checkRejectionsSatisfied(rejections)){
        terminate = true;
        // 一つ前に戻る
        frame2Link(prevFrame, variables); // 速度無視している
        for(std::set<cnoid::BodyPtr>::iterator it=bodies.begin(); it != bodies.end(); it++){
          (*it)->calcForwardKinematics(param.calcVelocity);
          (*it)->calcCenterOfMass();
        }
        if(path != nullptr && (loop + 1) % param.pathOutputLoop == 0) {
          path->pop_back();
        }
        updateConstraints(variables, ikc_list, param);
        satisfied = checkConstraintsSatisfied(ikc_list);
      }else if(loop+1 >= param.maxIteration){
        terminate = true;
        satisfied = checkConstraintsSatisfied(ikc_list);
      }else{
        if(converged) {
          if(checkConstraintsSatisfied(ikc_list, param.satisfiedConvergeLevel)){
            terminate = true;
            satisfied = checkConstraintsSatisfied(ikc_list);
          }
        }
        if(!terminate && loop >= param.minIteration){
          if (checkConstraintsSatisfied(ikc_list)) {
            terminate = true;
            satisfied = true;
          }
        }
      }
      if(terminate){
        if(path != nullptr) {
          path->resize(path->size() + 1);
          link2Frame(variables, path->back());
        }
        if(param.debugLevel > 0) {
          double time = timer.measure();
          std::cerr << "[PrioritizedIK] solveIKLoop loop: " << loop << " time: " << time << "[s]." << std::endl;
        }
        return satisfied;
      }
    }
  }
}
