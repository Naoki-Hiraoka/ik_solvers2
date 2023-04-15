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
    //  precision: 収束判定の閾値[rad].
    //  maxError: エラーの頭打ち[rad]
    //  weight: コスト関数の重み. error * weight^2 * error.

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

    // 内部状態更新. eq, minIneq, maxIneqを生成
    virtual void updateBounds () override;
    // 内部状態更新. jacobian, jacobianIneqを生成
    virtual void updateJacobian (const std::vector<cnoid::LinkPtr>& joints) override;
    // 達成判定
    virtual bool isSatisfied () const override;
    // 達成までの距離. getEqなどは、エラーの頭打ちを行うが、distanceは行わないので、より純粋なisSatisfiedまでの距離を表す.
    virtual double distance() const override;

  private:
    cnoid::LinkPtr joint_ = nullptr;
    double dt_ = 0.1;
    double precision_ = 1e-3;
    double maxError_ = 1e-2;
    double weight_ = 1.0;

    double current_lower_ = 0.0;
    double current_upper_ = 0.0;
    cnoid::Vector6 current_lower6_ = cnoid::Vector6::Zero();
    cnoid::Vector6 current_upper6_ = cnoid::Vector6::Zero();

    cnoid::LinkPtr jacobianineq_joint_ = nullptr; //前回jacobian_を計算した時のjoint

    std::vector<cnoid::LinkPtr> jacobianineq_joints_; // 前回のjacobianineq計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianineqColMap_;

  };
}

#endif
