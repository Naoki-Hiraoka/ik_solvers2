#ifndef IK_CONSTRAINT2_POSITIONCONSTRAINT_H
#define IK_CONSTRAINT2_POSITIONCONSTRAINT_H

#include <ik_constraint2/IKConstraint.h>
#include <cnoid/EigenUtil>
#include <cnoid/LinkPath>
#include <iostream>

namespace ik_constraint2{
  class PositionConstraint : public IKConstraint
  {
  public:
    //A_link中のA_localposの部位とB_link中のB_localposの部位を一致させる.
    //リンクがnullptrならworld座標系を意味する
    //  maxError: エラーの頭打ち eval系
    //  weight: コスト関数の重み. error * weight^2 * error. 0の成分はjacobianやeqに含まれない. eval系
    //  precision: 収束判定の閾値 error * weightのノルムとこの値を比べる
    //状態が更新される度に, 手動でcalcForwardKinematics()を呼ぶ必要が有る.
    const cnoid::LinkPtr& A_link() const { return A_link_;}
    cnoid::LinkPtr& A_link() { return A_link_;}
    const cnoid::Position& A_localpos() const { return A_localpos_;}
    cnoid::Position& A_localpos() { return A_localpos_;}
    const cnoid::LinkPtr& B_link() const { return B_link_;}
    cnoid::LinkPtr& B_link() { return B_link_;}
    const cnoid::Position& B_localpos() const { return B_localpos_;}
    cnoid::Position& B_localpos() { return B_localpos_;}
    const cnoid::Vector6& maxError() const { return maxError_;}
    cnoid::Vector6& maxError() { return maxError_;}
    const double& precision() const { return precision_;}
    double& precision() { return precision_;}
    const cnoid::Vector6& weight() const { return weight_;}
    cnoid::Vector6& weight() { return weight_;}
    const cnoid::LinkPtr& eval_link() const { return eval_link_;}
    cnoid::LinkPtr& eval_link() { return eval_link_;}
    const cnoid::Matrix3d& eval_localR() const { return eval_localR_;}
    cnoid::Matrix3d& eval_localR() { return eval_localR_;}

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

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    cnoid::LinkPtr A_link_ = nullptr;
    cnoid::Position A_localpos_ = cnoid::Position::Identity();
    cnoid::LinkPtr B_link_ = nullptr;
    cnoid::Position B_localpos_ = cnoid::Position::Identity();
    cnoid::Vector6 maxError_ = (cnoid::Vector6()<<0.05,0.05,0.05,0.05,0.05,0.05).finished();
    double precision_ = 1e-4;
    cnoid::Vector6 weight_ = cnoid::Vector6::Ones();
    cnoid::LinkPtr eval_link_ = nullptr;
    cnoid::Matrix3d eval_localR_ = cnoid::Matrix3d::Identity();

    cnoid::SgLineSetPtr lines_;
    cnoid::Vector6 current_error_eval_ = cnoid::Vector6::Zero();

    std::vector<cnoid::LinkPtr> path_A_joints_;
    std::vector<cnoid::LinkPtr> path_B_joints_;
    std::vector<cnoid::LinkPtr> path_BA_joints_;
    int path_BA_joints_numUpwardConnections_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_local_;
    cnoid::LinkPtr jacobian_A_link_ = nullptr;// 前回のjacobian計算時のA_link
    cnoid::LinkPtr jacobian_B_link_ = nullptr;// 前回のjacobian計算時のB_link

    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;

  };
}

#endif
