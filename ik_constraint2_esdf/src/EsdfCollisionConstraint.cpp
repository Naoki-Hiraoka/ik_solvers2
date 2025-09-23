#include <ik_constraint2_esdf/EsdfCollisionConstraint.h>
#include <iostream>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/MeshFilter>

namespace ik_constraint2_esdf{

  std::vector<cnoid::Vector3> getSurfaceVertices(cnoid::LinkPtr link, float resolution){
    // 1つのvertexを取得したら、resolutionのサイズの同じ立方体の中にある他のvertexは取得しない
    // faceが巨大な場合、faceの内部の点をresolutionの間隔でサンプリングして取得する

    cnoid::MeshExtractor meshExtractor;

    std::vector<cnoid::Vector3> vertices;
    cnoid::SgMeshPtr mesh = meshExtractor.integrate(link->collisionShape());
    if(mesh && (mesh->numTriangles() != 0)) {
      mesh->updateBoundingBox();
      cnoid::BoundingBoxf bbx = mesh->boundingBox();
      cnoid::Vector3f bbxSize = bbx.max() - bbx.min();
      std::vector<std::vector<std::vector<bool> > > bin;
      bin.resize(int(bbxSize[0]/resolution)+1);
      for(int x=0;x<bin.size();x++){
        bin[x].resize(int(bbxSize[1]/resolution)+1);
        for(int y=0;y<bin[x].size();y++){
          bin[x][y].resize(int(bbxSize[2]/resolution)+1,false);
        }
      }

      for(int j=0;j<mesh->numTriangles();j++){
        cnoid::Vector3f v0 = mesh->vertices()->at(mesh->triangle(j)[0]);
        cnoid::Vector3f v1 = mesh->vertices()->at(mesh->triangle(j)[1]);
        cnoid::Vector3f v2 = mesh->vertices()->at(mesh->triangle(j)[2]);
        float l1 = (v1 - v0).norm();
        float l2 = (v2 - v0).norm();
        cnoid::Vector3f d1 = (v1 - v0).normalized();
        cnoid::Vector3f d2 = (v2 - v0).normalized();

        for(float m=0;;){
          float n_max = (l1==0)? l2 : l2*(1-m/l1);
          for(float n=0;;){
            cnoid::Vector3f v = v0 + d1 * m + d2 * n;
            int x = int((v[0] - bbx.min()[0])/resolution);
            int y = int((v[1] - bbx.min()[1])/resolution);
            int z = int((v[2] - bbx.min()[2])/resolution);
            if(!bin[x][y][z]){
              bin[x][y][z] = true;
              vertices.push_back(v.cast<double>());
            }

            if(n>= n_max) break;
            else n = std::min(n+resolution, n_max);
          }

          if(m>=l1) break;
          else m = std::min(m+resolution, l1);
        }
      }
    }
    return vertices;
  }

  std::vector<std::pair<cnoid::Vector3, cnoid::Vector3> > getSurfaceVerticesAndNormals(cnoid::LinkPtr link, float resolution, float minangle) {
    // 1つのvertexを取得したら、resolutionのサイズの同じ立方体の中にありかつ法線がminangle以下の他のvertexは取得しない
    // faceが巨大な場合、faceの内部の点をresolutionの間隔でサンプリングして取得する

    cnoid::MeshExtractor meshExtractor;
    cnoid::MeshFilter meshFilter;

    std::vector<std::pair<cnoid::Vector3, cnoid::Vector3> > vertices;
    cnoid::SgMeshPtr mesh = meshExtractor.integrate(link->collisionShape());
    if(mesh && (mesh->numTriangles() != 0)) {
      meshFilter.generateNormals(mesh,M_PI,true);
      mesh->updateBoundingBox();
      cnoid::BoundingBoxf bbx = mesh->boundingBox();
      cnoid::Vector3f bbxSize = bbx.max() - bbx.min();
      std::vector<std::vector<std::vector<std::vector<cnoid::Vector3f> > > > bin; // normalを入れる
      bin.resize(int(bbxSize[0]/resolution)+1);
      for(int x=0;x<bin.size();x++){
        bin[x].resize(int(bbxSize[1]/resolution)+1);
        for(int y=0;y<bin[x].size();y++){
          bin[x][y].resize(int(bbxSize[2]/resolution)+1);
        }
      }

      for(int j=0;j<mesh->numTriangles();j++){
        cnoid::Vector3f v0 = mesh->vertices()->at(mesh->triangle(j)[0]);
        cnoid::Vector3f v1 = mesh->vertices()->at(mesh->triangle(j)[1]);
        cnoid::Vector3f v2 = mesh->vertices()->at(mesh->triangle(j)[2]);
        cnoid::Vector3f normal = mesh->normals()->at(mesh->normalIndices()[j*3]); // linkの外側に向かう方向
        float l1 = (v1 - v0).norm();
        float l2 = (v2 - v0).norm();
        cnoid::Vector3f d1 = (v1 - v0).normalized();
        cnoid::Vector3f d2 = (v2 - v0).normalized();

        for(float m=0;;){
          float n_max = (l1==0)? l2 : l2*(1-m/l1);
          for(float n=0;;){
            cnoid::Vector3f v = v0 + d1 * m + d2 * n;
            int x = int((v[0] - bbx.min()[0])/resolution);
            int y = int((v[1] - bbx.min()[1])/resolution);
            int z = int((v[2] - bbx.min()[2])/resolution);

            bool exists = false;
            for(int s=0;s<bin[x][y][z].size();s++){
              if(minangle >= std::acos(std::min(1.0f,(std::max(-1.0f,normal.dot(bin[x][y][z][s])))))){
                exists = true;
                break;
              }
            }
            if(!exists){
              bin[x][y][z].push_back(normal);
              vertices.emplace_back(v.cast<double>(), normal.cast<double>());
            }

            if(n>= n_max) break;
            else n = std::min(n+resolution, n_max);
          }

          if(m>=l1) break;
          else m = std::min(m+resolution, l1);
        }
      }
    }
    return vertices;
  }

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
      // 障害物と遠すぎて近傍点が計算できていない
      distance = min_dist_grad_invalid;
      direction = cnoid::Vector3::UnitX(); // てきとう
      A_v = closest_v_grad_invalid;
      B_v = (A_link->T() * A_v) - direction * distance;
    }else if (min_dist >= this->maxDistance_/*初期値*/ ||
              min_dist_grad_invalid < min_dist) {
      // 障害物と近すぎて近傍点が計算できていない
      // 干渉時は近傍点が正しくない場合があるので、干渉直前の値を使う
      direction = this->prev_direction_;
      A_v = this->prev_A_localp_;
      B_v = this->prev_B_localp_;
      distance = std::min(double(field->voxel_size()), ((A_link->T() * this->prev_A_localp_) - this->prev_B_localp_).dot(this->prev_direction_));  // 最大でfield->voxel_size()の値になりうることに注意. tolerance - precision > voxel_sizeとせよ.

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
