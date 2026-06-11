#include <ik_constraint2_esdf/EsdfMultiCollisionConstraint.h>
#include <ik_constraint2_esdf/util.h>
#include <iostream>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/MeshFilter>

namespace ik_constraint2_esdf{

  class EsdfMultiCollisionConstraintResult {
  public:
    cnoid::Vector3 A_v;
    cnoid::Vector3 v_fieldLocal;
    cnoid::Vector3 grad;
    double dist;
  };

  bool EsdfMultiCollisionConstraint::computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, std::vector<double>& distances, std::vector<cnoid::Vector3>& direction/*B->A*/, std::vector<cnoid::Vector3>& A_v, std::vector<cnoid::Vector3>& B_v) {
    if(A_link == nullptr ||
       B_link != nullptr ||
       this->field_ == nullptr){
      std::cerr << "[EsdfMultiCollisionConstraint::computeDistance] assertion failed" << std::endl; // 起こりえない. for debug
    }

    distances.clear();
    direction.clear();
    A_v.clear();
    B_v.clear();

    // 別スレッドで上書きされてもいいようにコピー
    std::shared_ptr<voxblox::EsdfMap> field = this->field_;
    cnoid::Isometry3 fieldOrigin = this->fieldOrigin_;
    cnoid::Isometry3 fieldOriginInv = fieldOrigin.inverse();

    if(A_link && A_link != this->A_link_vertices_){
      this->A_vertices_ = getSurfaceVertices(A_link, this->resolution_);
      this->A_link_vertices_ = A_link;

      cnoid::Vector3 A_max = - cnoid::Vector3::Ones() * std::numeric_limits<double>::max();
      this->A_min_ = cnoid::Vector3::Ones() * std::numeric_limits<double>::max();
      for(int i=0;i<this->A_vertices_.size();i++){
        A_max = A_max.cwiseMax(this->A_vertices_[i]);
        this->A_min_ = this->A_min_.cwiseMin(this->A_vertices_[i]);
      }

      if(this->A_vertices_.size() ==0) {
        this->A_bin_.clear();
      }else{
        cnoid::Vector3 bbxSize = A_max - this->A_min_;
        this->A_bin_.resize(int(bbxSize[0]/this->resolution2_)+1);
        for(int x=0;x<this->A_bin_.size();x++){
          this->A_bin_[x].resize(int(bbxSize[1]/this->resolution2_)+1);
          for(int y=0;y<this->A_bin_[x].size();y++){
            this->A_bin_[x][y].resize(int(bbxSize[2]/this->resolution2_)+1,false);
          }
        }
      }
    }

    // update ignore bounding box
    for(int i=0;i<this->ignoreBoundingBox_.size();i++) this->ignoreBoundingBox_[i].cacheParentLinkPose();

    Eigen::Isometry3d linkT = A_link->T();

    double min_dist = this->maxDistance_; // minDistance以上離れていて、勾配が0でない点のうち、最近傍の距離.
    double min_dist_grad_invalid = min_dist; // 最近傍の距離

    std::vector<EsdfMultiCollisionConstraintResult> results; // minDistance以上離れていて、maxDistance未満であり、勾配が0でない点
    results.reserve(this->A_vertices_.size());
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
      }
      if(grad.norm() > 0 && dist >= this->minDistance_ && dist < this->maxDistance_){
        EsdfMultiCollisionConstraintResult result;
        result.A_v = this->A_vertices_[j];
        result.v_fieldLocal = v_fieldLocal;
        result.grad = grad;
        result.dist = dist;
        results.push_back(result);
        if(dist < min_dist){
          min_dist = dist;
        }
      }
    }

    if(min_dist_grad_invalid >= this->maxDistance_/*初期値*/){
      // 障害物と遠すぎて近傍点が計算できていない.
      distance = min_dist_grad_invalid;
    }else if (min_dist >= this->maxDistance_/*初期値*/ ||
              min_dist_grad_invalid < min_dist) {
      // 障害物と近すぎて近傍点が計算できていない
      distance = this->minDistance_;
    }else{
      distance = min_dist;

      for(int x=0;x<this->A_bin_.size();x++){
        for(int y=0;y<this->A_bin_[x].size();y++){
          for(int z=0;z<this->A_bin_[x][y].size();z++){
            this->A_bin_[x][y][z] = false;
          }
        }
      }

      std::sort(results.begin(), results.end(),
                [](const EsdfMultiCollisionConstraintResult& a, const EsdfMultiCollisionConstraintResult& b) {
                  return a.dist < b.dist;
                });

      for(int i=0;i<results.size();i++){
        if(results[i].dist > min_dist + this->epsilon_) break;;

        int x = int((results[i].A_v[0] - this->A_min_[0])/this->resolution2_);
        int y = int((results[i].A_v[1] - this->A_min_[1])/this->resolution2_);
        int z = int((results[i].A_v[2] - this->A_min_[2])/this->resolution2_);
        if(this->A_bin_[x][y][z]) continue;
        this->A_bin_[x][y][z] = true;

        cnoid::Vector3 direction_fieldLocal = results[i].grad/results[i].grad.norm();
        cnoid::Vector3 point_fieldLocal = results[i].v_fieldLocal-direction_fieldLocal*results[i].dist;
        cnoid::Vector3 point = fieldOrigin * point_fieldLocal;
        cnoid::Vector3 dir = fieldOrigin.linear() * direction_fieldLocal;

        distances.push_back(results[i].dist);
        direction.push_back(dir);
        A_v.push_back(results[i].A_v);
        B_v.push_back(point);
      }
    }

    return true;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> EsdfMultiCollisionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<EsdfMultiCollisionConstraint> ret = std::make_shared<EsdfMultiCollisionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void EsdfMultiCollisionConstraint::copy(std::shared_ptr<EsdfMultiCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    MultiCollisionConstraint::copy(ret, modelMap);

    //verticesは使いまわす
    ret->A_link_vertices() = applyModelMap(this->A_link_vertices_, modelMap);
    for(int i=0;i<ret->ignoreBoundingBox().size();i++){
      ret->ignoreBoundingBox()[i].parentLink = applyModelMap(ret->ignoreBoundingBox()[i].parentLink, modelMap);
    }
  }


}
