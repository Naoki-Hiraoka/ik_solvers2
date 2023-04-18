#ifndef IK_CONSTRAINT2_COLLISIONCONSTRAINT_H
#define IK_CONSTRAINT2_COLLISIONCONSTRAINT_H

#include <ik_constraint2/IKConstraint.h>

namespace ik_constraint2{
  class CollisionConstraint : public IKConstraint
  {
  public:
    // A_linkとB_linkの干渉を回避する
    //  tolerance: この値以上離す[m]
    //  precision: 収束判定の閾値. distance - torelanceと比べる
    //  maxError: エラーの頭打ち
    //  weight: コスト関数の重み. error * weight^2 * error. maxErrorの適用後に適用する
    //  velocityDamper: 不等式制約の差分をこの値分の1にする. maxErrorの適用前に適用する.
    //状態が更新される度に, 手動でcalcForwardKinematics()を呼ぶ必要が有る.

    const cnoid::LinkPtr& A_link() const { return A_link_;}
    cnoid::LinkPtr& A_link() { return A_link_;}
    const cnoid::LinkPtr& B_link() const { return B_link_;}
    cnoid::LinkPtr& B_link() { return B_link_;}
    const double& tolerance() const { return tolerance_;}
    double& tolerance() { return tolerance_;}
    const double& maxError() const { return maxError_;}
    double& maxError() { return maxError_;}
    const double& precision() const { return precision_;}
    double& precision() { return precision_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}
    const double& velocityDamper() const { return velocityDamper_;}
    double& velocityDamper() { return velocityDamper_;}

    // 内部状態更新. eq, minIneq, maxIneqを生成
    virtual void updateBounds () override;
    // 内部状態更新. jacobian, jacobianIneqを生成
    virtual void updateJacobian (const std::vector<cnoid::LinkPtr>& joints) override;
    // 達成判定
    virtual bool isSatisfied () const override;
    // 達成までの距離. getEqなどは、エラーの頭打ちを行うが、distanceは行わないので、より純粋なisSatisfiedまでの距離を表す.
    virtual double distance() const override;

    // for debug view
    virtual std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;

  protected:
    //A_v, B_vはlocal系
    virtual bool computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v)=0;

  private:
    cnoid::LinkPtr A_link_ = nullptr;
    cnoid::LinkPtr B_link_ = nullptr;
    double tolerance_ = 0.01;
    double maxError_ = 0.05;
    double precision_ = 1e-3;
    double weight_ = 1.0;
    double velocityDamper_ = 1.0;

    cnoid::SgLineSetPtr lines_;

    cnoid::Vector3 A_currentLocalp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 B_currentLocalp_ = cnoid::Vector3::Zero();
    cnoid::Vector3 currentDirection_ = cnoid::Vector3::UnitX();
    double currentDistance_ = 0.0;

    std::vector<cnoid::LinkPtr> path_A_joints_;
    std::vector<cnoid::LinkPtr> path_B_joints_;
    std::vector<cnoid::LinkPtr> path_BA_joints_;
    int path_BA_joints_numUpwardConnections_ = 0;
    cnoid::LinkPtr jacobianineq_A_link_ = nullptr;// 前回のjacobian計算時のA_link
    cnoid::LinkPtr jacobianineq_B_link_ = nullptr;// 前回のjacobian計算時のB_link
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianineq_full_;

    std::vector<cnoid::LinkPtr> jacobianineq_joints_; // 前回のjacobianineq計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianineqColMap_;

  };


}

#endif
