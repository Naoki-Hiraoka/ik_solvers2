#include <ik_constraint2_esdf/EsdfCollisionConstraint.h>
#include <ik_constraint2_esdf/util.h>
#include <iostream>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/MeshFilter>

namespace ik_constraint2_esdf{

  bool EsdfCollisionConstraint::computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) {
    if(A_link == nullptr ||
       B_link != nullptr ||
       this->field_ == nullptr){
      std::cerr << "[EsdfCollisionConstraint::computeDistance] assertion failed" << std::endl;
    }

    // 別スレッドで上書きされてもいいようにコピー
    std::shared_ptr<voxblox::EsdfMap> field = this->field_;
    cnoid::Isometry3 fieldOrigin = this->fieldOrigin_;
    cnoid::Isometry3 fieldOriginInv = fieldOrigin.inverse();

    if(A_link && A_link != this->A_link_vertices_){
      this->A_vertices_ = getSurfaceVertices(A_link, this->resolution_);
      this->A_link_vertices_ = A_link;
    }

    // update ignore bounding box
    for(int i=0;i<this->ignoreBoundingBox_.size();i++) this->ignoreBoundingBox_[i].cacheParentLinkPose();

    Eigen::Isometry3d linkT = A_link->T();

    double min_dist = this->maxDistance_; // minDistance以上離れていて、勾配が0でない点のうち、最近傍の距離.
    cnoid::Vector3 closest_v = cnoid::Vector3::Zero(); // link local
    cnoid::Vector3 closest_point_fieldLocal = cnoid::Vector3::Zero(); // field local
    cnoid::Vector3 closest_direction_fieldLocal = cnoid::Vector3::UnitX(); // field local. fieldからlinkへの方向

    double min_dist_grad_invalid = min_dist; // 最近傍の距離
    cnoid::Vector3 closest_v_grad_invalid = closest_v; // link local

    for(int j=0;j<this->A_vertices_.size();j++){
      cnoid::Vector3 v = linkT * this->A_vertices_[j];

      bool ignore = false;
      for(int k=0;k<this->ignoreBoundingBox_.size();k++){
        if(this->ignoreBoundingBox_[k].isInside(v)) {
          ignore = true;
          break;
        }
      }
      if(ignore) continue;

      cnoid::Vector3 v_fieldLocal = fieldOriginInv * v;

      cnoid::Vector3 grad;
      double dist;
      bool success = field->getDistanceAndGradientAtPosition(v_fieldLocal,&dist,&grad);
      if(!success) {
        // fieldの外部にある or 未観測. ロボットの周囲がESDFに含まれかつ観測済みになるようにしておくこと
        continue;
      }
      if(dist < min_dist_grad_invalid) {
          min_dist_grad_invalid = dist;
          closest_v_grad_invalid = this->A_vertices_[j];
      }
      if(grad.norm() > 0 && dist >= this->minDistance_){
        if(dist < min_dist){
          closest_direction_fieldLocal = grad/grad.norm();
          closest_point_fieldLocal = v_fieldLocal-closest_direction_fieldLocal*dist;
          min_dist = dist;
          closest_v = this->A_vertices_[j];
        }
      }
    }

    if(min_dist_grad_invalid >= this->maxDistance_/*初期値*/){
      // 障害物と遠すぎて近傍点が計算できていない. ignoreDistanceをmaxDistance以下にして無視せよ
      distance = min_dist_grad_invalid;
      direction = cnoid::Vector3::UnitX(); // てきとう
      A_v = cnoid::Vector3::Zero(); // てきとう
      B_v = cnoid::Vector3::Zero(); // てきとう
      if(this->ignoreDistance_ >= this->maxDistance_){
        std::cerr << "[EsdfCollisionConstraint::computeDistance] ignoreDistance >= maxDistance" << std::endl;
      }
    }else if (min_dist >= this->maxDistance_/*初期値*/ ||
              min_dist_grad_invalid < min_dist) {
      // 障害物と近すぎて近傍点が計算できていない
      // 干渉時は近傍点が正しくない場合があるので、干渉直前の値を使う
      direction = this->prev_direction_;
      A_v = this->prev_A_localp_;
      B_v = this->prev_B_localp_;
      distance = std::min(this->minDistance_, ((A_link->T() * this->prev_A_localp_) - this->prev_B_localp_).dot(this->prev_direction_));  // 最大でthis->minDistance_の値になりうることに注意. tolerance - precision > voxel_sizeとせよ.

      if((this->tolerance_ - this->precision_) < double(field->voxel_size())){
        std::cerr << "[EsdfCollisionConstraint::computeDistance] tolerance - precision < voxel_size" << std::endl;
      }

    }else{
      cnoid::Vector3 closest_point = fieldOrigin * closest_point_fieldLocal;
      cnoid::Vector3 closest_direction = fieldOrigin.linear() * closest_direction_fieldLocal;

      distance = min_dist;
      direction = closest_direction;
      A_v = closest_v;
      B_v = closest_point;
    }

    this->prev_dist_ = distance;
    this->prev_direction_ = direction;
    this->prev_A_localp_ = A_v;
    this->prev_B_localp_ = B_v;

    return true;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> EsdfCollisionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<EsdfCollisionConstraint> ret = std::make_shared<EsdfCollisionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void EsdfCollisionConstraint::copy(std::shared_ptr<EsdfCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    CollisionConstraint::copy(ret, modelMap);

    //verticesは使いまわす
    if(this->A_link_vertices_ && modelMap.find(this->A_link_vertices_->body()) != modelMap.end()) ret->A_link_vertices() = modelMap.find(this->A_link_vertices_->body())->second->link(this->A_link_vertices_->index());
    for(int i=0;i<ret->ignoreBoundingBox().size();i++){
      if(ret->ignoreBoundingBox()[i].parentLink && modelMap.find(ret->ignoreBoundingBox()[i].parentLink->body()) != modelMap.end()) ret->ignoreBoundingBox()[i].parentLink = modelMap.find(ret->ignoreBoundingBox()[i].parentLink->body())->second->link(ret->ignoreBoundingBox()[i].parentLink->index());
    }
  }


}
