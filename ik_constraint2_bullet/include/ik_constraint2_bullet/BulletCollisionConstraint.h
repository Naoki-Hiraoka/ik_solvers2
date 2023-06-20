#ifndef IK_CONSTRAINT2_BULLET_BULLETCOLLISIONCONSTRAINT_H
#define IK_CONSTRAINT2_BULLET_BULLETCOLLISIONCONSTRAINT_H

#include <ik_constraint2/CollisionConstraint.h>

class btConvexShape;

namespace ik_constraint2_bullet{
  class BulletCollisionConstraint : public ik_constraint2::CollisionConstraint {
  public:
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<BulletCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

    // meshの点が少なく次元が縮退しているとqhullがエラーになるので、特にuseSingleMesh=falseのときは注意.
    bool& useSingleMesh() { return useSingleMesh_; }
    const bool& useSingleMesh() const { return useSingleMesh_; }

    // 各リンクのbullet model. *_link_bulletModelと*_linkが一致していなければ、自動で作成される.
    std::vector<std::shared_ptr<btConvexShape> >& A_bulletModel() { return A_bulletModel_; }
    const std::vector<std::shared_ptr<btConvexShape> >& A_bulletModel() const { return A_bulletModel_; }
    std::vector<std::shared_ptr<btConvexShape> >& B_bulletModel() { return B_bulletModel_; }
    const std::vector<std::shared_ptr<btConvexShape> >& B_bulletModel() const { return B_bulletModel_; }
    // *_bulletModel_作成時のの*_link
    cnoid::LinkPtr& A_link_bulletModel() { return A_link_bulletModel_; }
    const cnoid::LinkPtr& A_link_bulletModel() const { return A_link_bulletModel_; }
    cnoid::LinkPtr& B_link_bulletModel() { return B_link_bulletModel_; }
    const cnoid::LinkPtr& B_link_bulletModel() const { return B_link_bulletModel_; }
  protected:
    //A_v, B_vはlocal系
    virtual bool computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) override;

    bool useSingleMesh_ = true;

    std::vector<std::shared_ptr<btConvexShape> > A_bulletModel_;
    cnoid::LinkPtr A_link_bulletModel_; // A_bulletModel_のA_link
    std::vector<std::shared_ptr<btConvexShape> > B_bulletModel_;
    cnoid::LinkPtr B_link_bulletModel_; // B_bulletModel_のB_link

    cnoid::Vector3 prev_A_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 prev_B_localp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 prev_direction_ = cnoid::Vector3::Zero();
    double prev_dist_ = 0.0;

  };
}

#endif
