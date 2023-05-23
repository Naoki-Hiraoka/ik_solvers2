#include <ik_constraint2/ORConstraint.h>

namespace ik_constraint2{
  void ORConstraint::updateBounds () {
    this->activeIdx_ = -1;
    double maxMargin = -std::numeric_limits<double>::max();

    for(int i=0;i<this->children_.size();i++) {
      this->children_[i]->updateBounds();
      double mgn = this->children_[i]->margin();
      if(mgn > maxMargin) {
        maxMargin = mgn;
        this->activeIdx_ = i;
      }
    }

    if(this->activeIdx_ == -1) {
      this->eq_.resize(0);
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
    }else{
      this->eq_ = this->children_[this->activeIdx_]->getEq();
      this->minIneq_ = this->children_[this->activeIdx_]->getMinIneq();
      this->maxIneq_ = this->children_[this->activeIdx_]->getMaxIneq();
    }

    if(this->debugLevel_>=2){
      std::cerr << "ORConstraint " << this->activeIdx_ <<  std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }
  }

  void ORConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {

    this->jacobian_ = this->children_[this->activeIdx_]->getJacobian();
    this->jacobianIneq_ = this->children_[this->activeIdx_]->getJacobianIneq();

    if(this->debugLevel_>=2){
      std::cerr << "ORConstraint " << this->activeIdx_ << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool ORConstraint::isSatisfied () const {
    if(this->children_.size() == 0) return true;
    for(int i=0;i<this->children_.size();i++) {
      if(this->children_[i]->isSatisfied()) return true;
    }
    return false;
  }

  double ORConstraint::distance () const {
    if(this->activeIdx_ == -1) return 0.0;
    else return this->children_[this->activeIdx_]->distance();
  }

  double ORConstraint::margin () const {
    if(this->activeIdx_ == -1) return 0.0;
    else return this->children_[this->activeIdx_]->margin();
  }

  std::vector<cnoid::SgNodePtr>& ORConstraint::getDrawOnObjects(){
    this->drawOnObjects_.clear();
    for(int i=0;i<this->children_.size();i++) {
      std::vector<cnoid::SgNodePtr>& objs = this->children_[i]->getDrawOnObjects();
      std::copy(objs.begin(), objs.end(), std::back_inserter(this->drawOnObjects_));
    }
    return this->drawOnObjects_;
  }

  std::shared_ptr<IKConstraint> ORConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<ORConstraint> ret = std::make_shared<ORConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void ORConstraint::copy(std::shared_ptr<ORConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    ret->children().clear();
    for(int i=0;i<this->children_.size();i++) {
      ret->children().push_back(this->children_[i]->clone(modelMap));
    }
  }
}
