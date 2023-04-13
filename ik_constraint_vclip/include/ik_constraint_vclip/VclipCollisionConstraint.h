#ifndef IK_CONSTRAINT_VCLIP_VCLIPCOLLISIONCONSTRAINT_H
#define IK_CONSTRAINT_VCLIP_VCLIPCOLLISIONCONSTRAINT_H

#include <ik_constraint2/CollisionConstraint.h>

namespace Vclip{
  class Polyhedron;
}

namespace ik_constraint_vclip{
  class VclipCollisionConstraint : public ik_constraint2::CollisionConstraint {
  public:
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
