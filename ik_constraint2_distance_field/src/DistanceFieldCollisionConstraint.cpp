#include <ik_constraint2_distance_field/DistanceFieldCollisionConstraint.h>
#include <iostream>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>

namespace ik_constraint2_distance_field{
  inline void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
    cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
    const cnoid::Affine3& T = meshExtractor->currentTransform();

    const int vertexIndexTop = model->getOrCreateVertices()->size();

    const cnoid::SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
      const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
      model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
      cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
      const int v0 = vertexIndexTop + tri[0];
      const int v1 = vertexIndexTop + tri[1];
      const int v2 = vertexIndexTop + tri[2];
      model->addTriangle(v0, v1, v2);
    }
  }

  inline cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){
    if (!collisionshape) return nullptr;

    std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
    cnoid::SgMeshPtr model = new cnoid::SgMesh;
    if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
      model->setName(collisionshape->name());
    }else{
      std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return nullptr;
    }

    return model;
  }

  inline std::vector<cnoid::Vector3f> getVertex(cnoid::LinkPtr link, double resolution){
    // 1つのvertexを取得したら、resolutionのサイズの同じ立方体の中にある他のvertexは取得しない
    // faceが巨大な場合、faceの内部の点をresolutionの間隔でサンプリングして取得する

    std::vector<cnoid::Vector3f> vertices;
    cnoid::SgMeshPtr mesh = convertToSgMesh(link->collisionShape());
    if(mesh) {
      mesh->updateBoundingBox();
      cnoid::BoundingBoxf bbx = mesh->boundingBox();
      cnoid::Vector3f bbxSize = bbx.max() - bbx.min();
      std::vector<std::vector<std::vector<bool> > > bin(int(bbxSize[0]/resolution)+1,
                                                        std::vector<std::vector<bool> >(int(bbxSize[1]/resolution)+1,
                                                                                        std::vector<bool>(int(bbxSize[2]/resolution)+1,
                                                                                                          false)));

      for(int j=0;j<mesh->numTriangles();j++){
        cnoid::Vector3f v0 = mesh->vertices()->at(mesh->triangle(j)[0]);
        cnoid::Vector3f v1 = mesh->vertices()->at(mesh->triangle(j)[1]);
        cnoid::Vector3f v2 = mesh->vertices()->at(mesh->triangle(j)[2]);
        float l1 = (v1 - v0).norm();
        float l2 = (v2 - v0).norm();
        cnoid::Vector3f n1 = (v1 - v0).normalized();
        cnoid::Vector3f n2 = (v2 - v0).normalized();
        for(double m=0;m<l1;m+=resolution){
          for(double n=0;n<l2-(l2/l1*m);n+=resolution){
            cnoid::Vector3f v = v0 + n1 * m + n2 * n;
            int x = int((v[0] - bbx.min()[0])/resolution);
            int y = int((v[1] - bbx.min()[1])/resolution);
            int z = int((v[2] - bbx.min()[2])/resolution);
            if(!bin[x][y][z]){
              bin[x][y][z] = true;
              vertices.push_back(v);
            }
          }
          double n=l2-l2/l1*m;
          cnoid::Vector3f v = v0 + n1 * m + n2 * n;
          int x = int((v[0] - bbx.min()[0])/resolution);
          int y = int((v[1] - bbx.min()[1])/resolution);
          int z = int((v[2] - bbx.min()[2])/resolution);
          if(!bin[x][y][z]){
            bin[x][y][z] = true;
            vertices.push_back(v);
          }
        }
        double m = l1;
        double n= 0;
        cnoid::Vector3f v = v0 + n1 * m + n2 * n;
        int x = int((v[0] - bbx.min()[0])/resolution);
        int y = int((v[1] - bbx.min()[1])/resolution);
        int z = int((v[2] - bbx.min()[2])/resolution);
        if(!bin[x][y][z]){
          bin[x][y][z] = true;
          vertices.push_back(v);
        }
      }
    }
    return vertices;
  }

  bool DistanceFieldCollisionConstraint::computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) {
    if(A_link == nullptr ||
       B_link != nullptr ||
       field_ == nullptr){
      std::cerr << "[DistanceFieldCollisionConstraint::computeDistance] assertion failed" << std::endl;
    }

    // 別スレッドで上書きされてもいいようにコピー
    std::shared_ptr<distance_field::PropagationDistanceField> field = this->field_;
    cnoid::Position fieldOrigin = this->fieldOrigin_;
    Eigen::Affine3f fieldOriginInv = fieldOrigin.inverse().cast<float>();

    if(A_link && A_link != this->A_link_vertices_){
      this->A_vertices_ = getVertex(A_link, this->resolution_);
      this->A_link_vertices_ = A_link;
    }

    // update ignore bounding box
    for(int i=0;i<this->ignoreBoundingBox_.size();i++) this->ignoreBoundingBox_[i].cacheParentLinkPose();

    Eigen::Affine3f linkT = A_link->T().cast<float>();

    double min_dist = this->field_->getUninitializedDistance() + 1; // getUninitializedDistance() returns max_distance
    cnoid::Vector3f closest_v = cnoid::Vector3f::Zero(); // link local
    cnoid::Vector3 closest_point_fieldLocal = cnoid::Vector3::Zero(); // field local
    cnoid::Vector3 closest_direction_fieldLocal = cnoid::Vector3::UnitX(); // field local
    double min_dist_grad_invalid = min_dist;
    cnoid::Vector3f closest_v_grad_invalid = closest_v; // link local

    for(int j=0;j<this->A_vertices_.size();j++){
      cnoid::Vector3f v = linkT * this->A_vertices_[j];

      bool ignore = false;
      for(int k=0;k<this->ignoreBoundingBox_.size();k++){
        if(this->ignoreBoundingBox_[k].isInside(v)) {
          ignore = true;
          break;
        }
      }
      if(ignore) continue;

      cnoid::Vector3f v_fieldLocal = fieldOriginInv * v;

      cnoid::Vector3 grad;
      bool in_bound; // Whether or not the (x,y,z) is valid for gradient purposes.
      double dist = this->field_->getDistanceGradient(v_fieldLocal[0],v_fieldLocal[1],v_fieldLocal[2],grad[0],grad[1],grad[2],in_bound);
      if(dist < min_dist_grad_invalid) {
          min_dist_grad_invalid = dist;
          closest_v_grad_invalid = this->A_vertices_[j];
      }
      if(in_bound && grad.norm() > 0 && dist >= this->minDistance_){
        if(dist < min_dist){
          closest_direction_fieldLocal[0] = (grad[0]/grad.norm());
          closest_direction_fieldLocal[1] = (grad[1]/grad.norm());
          closest_direction_fieldLocal[2] = (grad[2]/grad.norm());
          closest_point_fieldLocal[0] = v_fieldLocal[0]-closest_direction_fieldLocal[0]*dist;
          closest_point_fieldLocal[1] = v_fieldLocal[1]-closest_direction_fieldLocal[1]*dist;
          closest_point_fieldLocal[2] = v_fieldLocal[2]-closest_direction_fieldLocal[2]*dist;
          min_dist = dist;
          closest_v = this->A_vertices_[j];
        }
      }
    }

    if(min_dist_grad_invalid >= this->field_->getUninitializedDistance()){
      // 障害物と遠すぎて近傍点が計算できていない
      distance = min_dist_grad_invalid;
      direction = cnoid::Vector3::UnitX(); // てきとう
      A_v = closest_v_grad_invalid.cast<double>();
      B_v = (A_link->T() * A_v) - direction * distance;
    }else if (min_dist >= this->field_->getUninitializedDistance() ||
              min_dist_grad_invalid < min_dist) {
      // 障害物と近すぎて近傍点が計算できていない
      // 干渉時は近傍点が正しくない場合があるので、干渉直前の値を使う
      direction = this->prev_direction_;
      A_v = this->prev_A_localp_;
      B_v = this->prev_B_localp_;
      distance = ((A_link->T() * this->prev_A_localp_) - this->prev_B_localp_).dot(this->prev_direction_);
    }else{
      cnoid::Vector3 closest_point = this->fieldOrigin_ * closest_point_fieldLocal;
      cnoid::Vector3 closest_direction = this->fieldOrigin_.linear() * closest_direction_fieldLocal;

      distance = min_dist;
      direction = closest_direction;
      A_v = closest_v.cast<double>();
      B_v = closest_point;
    }

    this->prev_dist_ = distance;
    this->prev_direction_ = direction;
    this->prev_A_localp_ = A_v;
    this->prev_B_localp_ = B_v;

    return true;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> DistanceFieldCollisionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<DistanceFieldCollisionConstraint> ret = std::make_shared<DistanceFieldCollisionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void DistanceFieldCollisionConstraint::copy(std::shared_ptr<DistanceFieldCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    CollisionConstraint::copy(ret, modelMap);

    //verticesは使いまわす
    if(this->A_link_vertices_ && modelMap.find(this->A_link_vertices_->body()) != modelMap.end()) ret->A_link_vertices() = modelMap.find(this->A_link_vertices_->body())->second->link(this->A_link_vertices_->index());
    for(int i=0;i<ret->ignoreBoundingBox().size();i++){
      if(ret->ignoreBoundingBox()[i].parentLink && modelMap.find(ret->ignoreBoundingBox()[i].parentLink->body()) != modelMap.end()) ret->ignoreBoundingBox()[i].parentLink = modelMap.find(ret->ignoreBoundingBox()[i].parentLink->body())->second->link(ret->ignoreBoundingBox()[i].parentLink->index());
    }
  }

}
