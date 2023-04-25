#ifndef IK_CONSTRAINT2_DISTANCE_FIELD_DISTANCE_FIELDCOLLISIONCONSTRAINT_H
#define IK_CONSTRAINT2_DISTANCE_FIELD_DISTANCE_FIELDCOLLISIONCONSTRAINT_H

#include <ik_constraint2/CollisionConstraint.h>

namespace Vclip{
  class Polyhedron;
}

namespace ik_constraint2_distance_field{
  class VclipCollisionConstraint : public ik_constraint2::CollisionConstraint {
  public:
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<VclipCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

    // ユーザーは使わない. copy()の中で使われる
    cnoid::LinkPtr& A_link_vclipModel() { return A_link_vclipModel_; }
    const cnoid::LinkPtr& A_link_vclipModel() const { return A_link_vclipModel_; }
    cnoid::LinkPtr& B_link_vclipModel() { return B_link_vclipModel_; }
    const cnoid::LinkPtr& B_link_vclipModel() const { return B_link_vclipModel_; }
  protected:
    //A_v, B_vはlocal系
    virtual bool computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) override;

    std::shared_ptr<Vclip::Polyhedron> A_vclipModel_;
    cnoid::LinkPtr A_link_vclipModel_; // A_vclipModel_のA_link
    std::shared_ptr<Vclip::Polyhedron> B_vclipModel_;
    cnoid::LinkPtr B_link_vclipModel_; // B_vclipModel_のB_link

    cnoid::Vector3 prev_A_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 prev_B_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 prev_direction_ = cnoid::Vector3::Zero();
    double prev_dist_ = 0.0;

  };
}

#endif
