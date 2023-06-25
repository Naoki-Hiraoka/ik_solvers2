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
    //  tolerance: limitからこの値以上離す[m, rad]

    const cnoid::LinkPtr& joint() const { return joint_;}
    cnoid::LinkPtr& joint() { return joint_;}
    const double& maxError() const { return maxError_;}
    double& maxError() { return maxError_;}
    const double& precision() const { return precision_;}
    double& precision() { return precision_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}
    const double& tolerance() const { return tolerance_;}
    double& tolerance() { return tolerance_;}

    // 内部状態更新. eq, minIneq, maxIneqを生成
    virtual void updateBounds () override;
    // 内部状態更新. jacobian, jacobianIneqを生成
    virtual void updateJacobian (const std::vector<cnoid::LinkPtr>& joints) override;
    // 達成判定
    virtual bool isSatisfied () const override;
    // 達成までの距離. getEqなどは、エラーの頭打ちを行うが、distanceは行わないので、より純粋なisSatisfiedまでの距離を表す.
    virtual double distance() const override;
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<JointLimitConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

  protected:
    virtual void calcMinMaxIneq(Eigen::VectorXd& maxIneq, Eigen::VectorXd& minIneq);

    cnoid::LinkPtr joint_ = nullptr;
    double precision_ = 1e-3;
    double maxError_ = 0.05;
    double weight_ = 1.0;
    double tolerance_ = 1e-3;

    double current_lower_ = 0.0;
    double current_upper_ = 0.0;

    cnoid::LinkPtr jacobianineq_joint_ = nullptr; //前回jacobian_を計算した時のjoint

    std::vector<cnoid::LinkPtr> jacobianineq_joints_; // 前回のjacobianineq計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianineqColMap_;

  };
}

#endif
