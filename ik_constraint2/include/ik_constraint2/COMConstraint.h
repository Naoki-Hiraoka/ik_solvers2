#ifndef IK_CONSTRAINT2_COMCONSTRAINT_H
#define IK_CONSTRAINT2_COMCONSTRAINT_H

#include <ik_constraint2/IKConstraint.h>
#include <cnoid/EigenUtil>
#include <cnoid/SceneMarkers>
#include <iostream>

namespace ik_constraint2{
  class COMConstraint : public IKConstraint
  {
  public:
    COMConstraint()
      :
      C_(Eigen::SparseMatrix<double,Eigen::RowMajor>(0,3)) // 下の定義の箇所で初期化するとUbuntu16でエラーになった
    {}
    //robotの重心をworld座標系のtargetPosに位置させる.
    //  maxError: エラーの頭打ち
    //  weight: コスト関数の重み. error * weight^2 * error. 0の成分はjacobianやerrorに含まれない
    //  precision: 収束判定の閾値. error * weightのノルムとこれを比べる
    //状態が更新される度に, 手動でcalcForwardKinematics()とcalcCenterOfMass()を呼ぶ必要が有る.
    const cnoid::BodyPtr& A_robot() const { return A_robot_;}
    cnoid::BodyPtr& A_robot() { return A_robot_;}
    const cnoid::Vector3& A_localp() const { return A_localp_;}
    cnoid::Vector3& A_localp() { return A_localp_;}
    const cnoid::BodyPtr& B_robot() const { return B_robot_;}
    cnoid::BodyPtr& B_robot() { return B_robot_;}
    const cnoid::Vector3& B_localp() const { return B_localp_;}
    cnoid::Vector3& B_localp() { return B_localp_;}
    const cnoid::Matrix3d& eval_R() const { return eval_R_;}
    cnoid::Matrix3d& eval_R() { return eval_R_;}

    // for equality
    const cnoid::Vector3& maxError() const { return maxError_;}
    cnoid::Vector3& maxError() { return maxError_;}
    const double& precision() const { return precision_;}
    double& precision() { return precision_;}
    const cnoid::Vector3& weight() const { return weight_;}
    cnoid::Vector3& weight() { return weight_;}

    // for inequality.  c * 重心位置A-B(eval_R) をdlとduの範囲内にする.
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& C() const { return C_;}
    Eigen::SparseMatrix<double,Eigen::RowMajor>& C() { return C_;}
    const cnoid::VectorX& dl() const { return dl_;}
    cnoid::VectorX& dl() { return dl_;}
    const cnoid::VectorX& du() const { return du_;}
    cnoid::VectorX& du() { return du_;}
    const cnoid::VectorX& maxCErrorVec() const { return maxCErrorVec_;}
    cnoid::VectorX& maxCErrorVec() { return maxCErrorVec_;}
    const double& maxCError() const { return maxCError_;}
    double& maxCError() { return maxCError_;}
    const double& CPrecision() const { return CPrecision_;}
    double& CPrecision() { return CPrecision_;}

    // 内部状態更新. eq, minIneq, maxIneqを生成
    virtual void updateBounds () override;
    // 内部状態更新. jacobian, jacobianIneqを生成
    virtual void updateJacobian (const std::vector<cnoid::LinkPtr>& joints) override;
    // 収束判定
    virtual bool isSatisfied() const override;
    // 達成までの距離. getEqなどは、エラーの頭打ちを行うが、distanceは行わないので、より純粋なisSatisfiedまでの距離を表す.
    virtual double distance() const override;
    // for debug view
    virtual std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<COMConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

  protected:
    cnoid::BodyPtr A_robot_ = nullptr;
    cnoid::Vector3 A_localp_ = cnoid::Vector3::Zero();
    cnoid::BodyPtr B_robot_ = nullptr;
    cnoid::Vector3 B_localp_ = cnoid::Vector3::Zero();
    cnoid::Matrix3d eval_R_ = cnoid::Matrix3d::Identity();

    cnoid::Vector3 maxError_ = 0.05 * cnoid::Vector3::Ones();
    double precision_ = 1e-3;
    cnoid::Vector3 weight_ = cnoid::Vector3::Ones();

    Eigen::SparseMatrix<double,Eigen::RowMajor> C_{0,3};
    cnoid::VectorX dl_;
    cnoid::VectorX du_;
    cnoid::VectorX maxCErrorVec_;
    double maxCError_ = 0.05;
    double CPrecision_ = 1e-3;

    cnoid::SgLineSetPtr lines_;
    cnoid::Vector3 current_error_eval_ = cnoid::Vector3::Zero();
    cnoid::VectorX current_u_;
    cnoid::VectorX current_l_;

    cnoid::BodyPtr jacobian_A_robot_ = nullptr;// 前回のjacobian計算時のrobot
    cnoid::BodyPtr jacobian_B_robot_ = nullptr;// 前回のjacobian計算時のrobot
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_local_;

    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;

  };
}

#endif
