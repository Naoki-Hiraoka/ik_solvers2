#ifndef IK_CONSTRAINT2_CLIENTCOLLISIONCONSTRAINT_H
#define IK_CONSTRAINT2_CLIENTCOLLISIONCONSTRAINT_H

#include <ik_constraint2/CollisionConstraint.h>

namespace ik_constraint2{
  class ClientCollisionConstraint : public CollisionConstraint
  {
  public:

    // A_linkとB_linkの干渉を回避する
    //  最近傍点と、方向ベクトルはgiven. 距離のみ計算
    //  direction: B->A

    const cnoid::Vector3& A_localp() const { return A_localp_;}
    cnoid::Vector3& A_localp() { return A_localp_;}
    const cnoid::Vector3& B_localp() const { return B_localp_;}
    cnoid::Vector3& B_localp() { return B_localp_;}
    const cnoid::Vector3& direction() const { return direction_;}
    cnoid::Vector3& direction() { return direction_;}

    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<ClientCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

  protected:
    //A_v, B_vはlocal系
    virtual bool computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) override;

    cnoid::Vector3 A_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 B_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 direction_ = cnoid::Vector3::UnitX();
  };


}

#endif
