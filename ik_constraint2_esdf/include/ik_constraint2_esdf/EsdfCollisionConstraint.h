#ifndef IK_CONSTRAINT2_ESDF_ESDFCOLLISIONCONSTRAINT_H
#define IK_CONSTRAINT2_ESDF_ESDFCOLLISIONCONSTRAINT_H

#include <ik_constraint2/CollisionConstraint.h>
#include <ik_constraint2/BoundingBox.h>
#include <voxblox/core/esdf_map.h>

namespace ik_constraint2_esdf{
  // B_linkはnullptrでなければならない. A_linkとdistancefieldの干渉を評価する
  class EsdfCollisionConstraint : public ik_constraint2::CollisionConstraint {
  public:
    using BoundingBox = ik_constraint2::BoundingBox;

    /*
      resolution: linkのメッシュのvertexをチェックする間隔
      field: distance_field
      fieldOrgin: fieldの原点が、world系のどこにあるか.
      minDistance: fieldの裏側に勾配が向かうことを防ぐため、この距離以下の点は無視する
      maxDistance: この距離以上の点はESDFに勾配が計算されていない. (esdf_integrator_config.max_distance_m)
      ignoreBoundingBox: この内部のlinkのvertexは無視する
     */
    double& resolution() { return resolution_; }
    const double& resolution() const { return resolution_; }
    std::shared_ptr<voxblox::EsdfMap>& field() {return this->field_; }
    const std::shared_ptr<voxblox::EsdfMap>& field() const {return this->field_; }
    cnoid::Isometry3& fieldOrigin() { return this->fieldOrigin_; }
    const cnoid::Isometry3& fieldOrigin() const { return this->fieldOrigin_; }
    double& minDistance() { return minDistance_; }
    const double& minDistance() const { return minDistance_; }
    double& maxDistance() { return maxDistance_; }
    const double& maxDistance() const { return maxDistance_; }
    std::vector<BoundingBox >& ignoreBoundingBox() { return this->ignoreBoundingBox_; }
    const std::vector<BoundingBox >& ignoreBoundingBox() const { return this->ignoreBoundingBox_; }

    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<EsdfCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

    // ユーザーは使わない. copy()の中で使われる
    std::vector<cnoid::Vector3>& A_vertices() { return A_vertices_; }
    const std::vector<cnoid::Vector3>& A_vertices() const { return A_vertices_; }
    cnoid::LinkPtr& A_link_vertices() { return A_link_vertices_; }
    const cnoid::LinkPtr& A_link_vertices() const { return A_link_vertices_; }

  protected:
    //A_v, B_vはlocal系
    virtual bool computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) override;

    double resolution_ = 0.02;
    std::shared_ptr<voxblox::EsdfMap> field_ = nullptr;
    cnoid::Isometry3 fieldOrigin_ = cnoid::Isometry3::Identity();
    double minDistance_ = -0.02;
    double maxDistance_ = 0.5;
    std::vector<BoundingBox > ignoreBoundingBox_;

    std::vector<cnoid::Vector3> A_vertices_; // A_link_のvertices. link local
    cnoid::LinkPtr A_link_vertices_; // A_vertices計算時のA_link_

    cnoid::Vector3 prev_A_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 prev_B_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 prev_direction_ = cnoid::Vector3::UnitX();
    double prev_dist_ = 0.0;

  private:
    void dummy();
  };

}

#endif
