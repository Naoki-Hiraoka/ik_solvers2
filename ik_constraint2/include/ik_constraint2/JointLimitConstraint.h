#ifndef IK_CONSTRAINT2_JOINTLIMITCONSTRAINT_H
#define IK_CONSTRAINT2_JOINTLIMITCONSTRAINT_H

#include <ik_constraint2/IKConstraint.h>
#include <cnoid/EigenUtil>

namespace ik_constraint2{
  class JointLimitConstraint : public IKConstraint
  {
  public:
    //jointのqをq_upperとq_lowerの間にさせる.
    //  maxError: エラーの頭打ち
    //  weight: コスト関数の重み. error * weight^2 * error. maxErrorの適用後に適用する
    //  precision: 収束判定の閾値. error * weightと比べる

    const cnoid::LinkPtr& joint() const { return joint_;}
    cnoid::LinkPtr& joint() { return joint_;}
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

  protected:
    virtual void calcMinMaxIneq(Eigen::VectorXd& maxIneq, Eigen::VectorXd& minIneq);

    cnoid::LinkPtr joint_ = nullptr;
    double precision_ = 1e-3;
    double maxError_ = 0.05;
    double weight_ = 1.0;

    cnoid::LinkPtr jacobianineq_joint_ = nullptr; //前回jacobian_を計算した時のjoint

    std::vector<cnoid::LinkPtr> jacobianineq_joints_; // 前回のjacobianineq計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianineqColMap_;

  };
}

#endif
