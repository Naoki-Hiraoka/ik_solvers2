#include <ik_constraint2_region_cdd/CddRegionConstraint.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>
#include <iostream>

namespace ik_constraint2_region_cdd{

  bool CddRegionConstraint::setVertices(const Eigen::MatrixXd V){
    if(V.rows() != 3) {
      std::cerr << __FUNCTION__ << "dimention mismatch" << std::endl;
      return false;
    }
    this->V_ = V;

    return choreonoid_cddlib::convertToFACEExpression(V,
                                                      this->C_,
                                                      this->dl_,
                                                      this->du_);
  }


  std::vector<cnoid::SgNodePtr>& CddRegionConstraint::getDrawOnObjects(){
    RegionConstraint::getDrawOnObjects();

    if(!this->points_){
      this->points_ = new cnoid::SgPointSet;
      this->points_->setPointSize(3.0);
      this->points_->getOrCreateColors()->resize(1);
      this->points_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.0,1.0,1.0);
    }

    cnoid::Position B_pos = (this->B_link_) ? this->B_link_->T() * this->B_localpos_ : this->B_localpos_;
    B_pos.linear() = (this->eval_link_) ? this->eval_link_->R() * this->eval_localR_ : this->eval_localR_;

    const Eigen::Matrix<double, 3, Eigen::Dynamic> V_world = B_pos * this->V_;

    this->points_->getOrCreateVertices()->resize(V_world.cols());
    this->points_->colorIndices().resize(V_world.cols());
    for(int i=0;i<V_world.cols();i++){
      this->points_->vertices()->at(i) = V_world.col(i).cast<cnoid::Vector3f::Scalar>();
      this->points_->colorIndices().at(i) = 0;
    }

    this->drawOnObjects_ = std::vector<cnoid::SgNodePtr>{this->lines_, this->points_};
    return this->drawOnObjects_;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> CddRegionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<CddRegionConstraint> ret = std::make_shared<CddRegionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void CddRegionConstraint::copy(std::shared_ptr<CddRegionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    RegionConstraint::copy(ret, modelMap);
  }

}
