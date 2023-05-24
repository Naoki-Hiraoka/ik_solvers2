#include <ik_constraint2/ANDConstraint.h>

namespace ik_constraint2{
  void ANDConstraint::updateBounds () {
    std::vector<std::reference_wrapper<const Eigen::VectorXd> > eqs;eqs.reserve(this->children_.size());
    std::vector<std::reference_wrapper <const Eigen::VectorXd> > minIneqs;minIneqs.reserve(this->children_.size());
    std::vector<std::reference_wrapper<const Eigen::VectorXd> > maxIneqs;maxIneqs.reserve(this->children_.size());

    int num_eqs = 0;
    int num_ineqs = 0;
    for(int i=0;i<this->children_.size();i++) {
      this->children_[i]->updateBounds();
      eqs.emplace_back(this->children_[i]->getEq());
      minIneqs.emplace_back(this->children_[i]->getMinIneq());
      maxIneqs.emplace_back(this->children_[i]->getMaxIneq());

      num_eqs += eqs[i].get().rows();
      num_ineqs += minIneqs[i].get().rows();
    }

    this->eq_.resize(num_eqs);
    this->minIneq_.resize(num_ineqs);
    this->maxIneq_.resize(num_ineqs);

    int idx_eq = 0;
    int idx_ineq = 0;
    for(size_t i=0;i<this->children_.size(); i++){
      this->eq_.segment(idx_eq,eqs[i].get().rows()) = eqs[i].get();
      idx_eq += eqs[i].get().rows();
      this->minIneq_.segment(idx_ineq,minIneqs[i].get().rows()) = minIneqs[i].get();
      this->maxIneq_.segment(idx_ineq,minIneqs[i].get().rows()) = maxIneqs[i].get();
      idx_ineq += minIneqs[i].get().rows();
    }

    if(this->debugLevel_>=2){
      std::cerr << "ANDConstraint" << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }
  }

  void ANDConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
    std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobians;jacobians.reserve(this->children_.size());
    std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobianIneqs;jacobianIneqs.reserve(this->children_.size());

    double dim = 0;
    for(size_t i=0;i<joints.size();i++) dim+=IKConstraint::getJointDOF(joints[i]);

    int num_eqs = 0;
    int num_ineqs = 0;
    for(int i=0;i<this->children_.size();i++) {
      this->children_[i]->updateJacobian(joints);
      jacobians.emplace_back(this->children_[i]->getJacobian());
      jacobianIneqs.emplace_back(this->children_[i]->getJacobianIneq());

      num_eqs += jacobians[i].get().rows();
      num_ineqs += jacobianIneqs[i].get().rows();
    }

    this->jacobian_.resize(num_eqs, dim);
    this->jacobianIneq_.resize(num_ineqs, dim);

    int idx_eq = 0;
    int idx_ineq = 0;
    for(size_t i=0;i<this->children_.size(); i++){
      this->jacobian_.middleRows(idx_eq,jacobians[i].get().rows()) = jacobians[i].get();
      idx_eq += jacobians[i].get().rows();
      this->jacobianIneq_.middleRows(idx_ineq,jacobianIneqs[i].get().rows()) = jacobianIneqs[i].get();
      idx_ineq += jacobianIneqs[i].get().rows();
    }

    if(this->debugLevel_>=2){
      std::cerr << "ANDConstraint" << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool ANDConstraint::isSatisfied () const {
    for(int i=0;i<this->children_.size();i++) {
      if(!this->children_[i]->isSatisfied()) return false;
    }
    return true;
  }

  double ANDConstraint::distance () const {
    double squaredDist = 0;
    for(int i=0;i<this->children_.size();i++) {
      squaredDist += std::pow(this->children_[i]->distance(), 2);
    }
    return std::sqrt(squaredDist);
  }

  double ANDConstraint::margin () const {
    if(!this->isSatisfied()) {
      return -this->distance();
    }else{
      double minMargin = std::numeric_limits<double>::max();
      for(int i=0;i<this->children_.size();i++) {
        minMargin = std::min(minMargin, this->children_[i]->margin());
      }
      return minMargin;
    }
  }

  std::vector<cnoid::SgNodePtr>& ANDConstraint::getDrawOnObjects(){
    this->drawOnObjects_.clear();
    for(int i=0;i<this->children_.size();i++) {
      std::vector<cnoid::SgNodePtr>& objs = this->children_[i]->getDrawOnObjects();
      std::copy(objs.begin(), objs.end(), std::back_inserter(this->drawOnObjects_));
    }
    return this->drawOnObjects_;
  }

  std::shared_ptr<IKConstraint> ANDConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<ANDConstraint> ret = std::make_shared<ANDConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void ANDConstraint::copy(std::shared_ptr<ANDConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    ret->children().clear();
    for(int i=0;i<this->children_.size();i++) {
      ret->children().push_back(this->children_[i]->clone(modelMap));
    }
  }
}
