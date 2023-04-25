#ifndef IK_CONSTRAINT2_DISTANCE_FIELD_DISTANCE_FIELDCOLLISIONCONSTRAINT_H
#define IK_CONSTRAINT2_DISTANCE_FIELD_DISTANCE_FIELDCOLLISIONCONSTRAINT_H

#include <ik_constraint2/CollisionConstraint.h>
#include <moveit/distance_field/propagation_distance_field.h>

namespace ik_constraint2_distance_field{
  // B_linkはnullptrでなければならない. A_linkとdistancefieldの干渉を評価する
  class DistanceFieldCollisionConstraint : public ik_constraint2::CollisionConstraint {
  public:
    class BoundingBox {
    public:
      cnoid::Position localPose = cnoid::Position::Identity();
      cnoid::LinkPtr parentLink;
      cnoid::Vector3 dimensions = cnoid::Vector3::Zero();

      bool isInside(const cnoid::Vector3f& p) {
        cnoid::Vector3f plocal = worldPoseinv * p;
        return
          (plocal[0] < dimensions[0]/2) &&
          (plocal[1] < dimensions[1]/2) &&
          (plocal[2] < dimensions[2]/2) &&
          (plocal[0] > -dimensions[0]/2) &&
          (plocal[1] > -dimensions[1]/2) &&
          (plocal[2] > -dimensions[2]/2);
      }
      void cacheParentLinkPose(){
        if(parentLink){
          worldPoseinv = (parentLink->T() * localPose).inverse().cast<float>();
        }else{
          worldPoseinv = Eigen::Affine3f::Identity();
        }
      }
    protected:
      Eigen::Affine3f worldPoseinv;
    };

    /*
      resolution: linkのメッシュのvertexをチェックする間隔
      field: distance_field
      fieldOrgin: fieldの原点が、world系のどこにあるか.
      minDistance: fieldの裏側に勾配が向かうことを防ぐため、この距離以下の点は無視する
      ignoreBoundingBox: この内部のlinkのvertexは無視する
     */
    double& resolution() { return resolution_; }
    const double& resolution() const { return resolution_; }
    std::shared_ptr<distance_field::PropagationDistanceField>& field() {return this->field_; }
    const std::shared_ptr<distance_field::PropagationDistanceField>& field() const {return this->field_; }
    cnoid::Position& fieldOrigin() { return this->fieldOrigin_; }
    const cnoid::Position& fieldOrigin() const { return this->fieldOrigin_; }
    double& minDistance() { return minDistance_; }
    const double& minDistance() const { return minDistance_; }
    std::vector<BoundingBox >& ignoreBoundingBox() { return this->ignoreBoundingBox_; }
    const std::vector<BoundingBox >& ignoreBoundingBox() const { return this->ignoreBoundingBox_; }

    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<DistanceFieldCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

    // ユーザーは使わない. copy()の中で使われる
    cnoid::LinkPtr& A_link_vertices() { return A_link_vertices_; }
    const cnoid::LinkPtr& A_link_vertices() const { return A_link_vertices_; }

  protected:
    //A_v, B_vはlocal系
    virtual bool computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) override;

    double resolution_ = 0.02;
    std::shared_ptr<distance_field::PropagationDistanceField> field_ = nullptr;
    cnoid::Position fieldOrigin_ = cnoid::Position::Identity();
    double minDistance_ = -0.02;
    std::vector<BoundingBox > ignoreBoundingBox_;

    std::vector<cnoid::Vector3f> A_vertices_; // A_link_のvertices. link local
    cnoid::LinkPtr A_link_vertices_; // A_vertices計算時のA_link_

    cnoid::Vector3 prev_A_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 prev_B_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 prev_direction_ = cnoid::Vector3::UnitX();
    double prev_dist_ = 0.0;

  };

  // link local frame
  std::vector<cnoid::Vector3f> getSurfaceVertices(cnoid::LinkPtr link, double resolution = 0.02);
}

#endif
