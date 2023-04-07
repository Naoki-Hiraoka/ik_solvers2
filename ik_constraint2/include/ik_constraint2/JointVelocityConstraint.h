#ifndef IK_CONSTRAINT2_JOINTVELOCITYCONSTRAINT_H
#define IK_CONSTRAINT2_JOINTVELOCITYCONSTRAINT_H

#include <ik_constraint2/IKConstraint.h>
#include <cnoid/EigenUtil>

namespace ik_constraint2{
  class JointVelocityConstraint : public IKConstraint
  {
  public:
    //jointのdqを上下限以下にする. radの次元で評価する
    //  dt: [s]
    //  maxError: エラーの頭打ち[rad]
    //  weight: コスト関数の重み. error * weight^2 * error.
    //  precision: 収束判定の閾値[rad]. error * weightのノルムと比べる

    const cnoid::LinkPtr& joint() const { return joint_;}
    cnoid::LinkPtr& joint() { return joint_;}
    const double& dt() const { return dt_;}
    double& dt() { return dt_;}

    const double& maxError() const { return maxError_;}
    double& maxError() { return maxError_;}
    const double& precision() const { return precision_;}
    double& precision() { return precision_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}

    //内部状態更新
    virtual void update (const std::vector<cnoid::LinkPtr>& joints) override;
    // 達成判定
    virtual bool isSatisfied () const override;

  private:
    cnoid::LinkPtr joint_ = nullptr;
    double dt_ = 0.1;
    double precision_ = 1e10;
    double maxError_ = 1e-2;
    double weight_ = 1.0;

    cnoid::LinkPtr jacobianineq_joint_ = nullptr; //前回jacobian_を計算した時のjoint

    std::vector<cnoid::LinkPtr> jacobianineq_joints_; // 前回のjacobianineq計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianineqColMap_;

  };
}

#endif
