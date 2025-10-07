#ifndef IK_CONSTRAINT2_REGIONCONSTRAINT2_H
#define IK_CONSTRAINT2_REGIONCONSTRAINT2_H

#include <ik_constraint2/IKConstraint.h>

namespace ik_constraint2{
  class RegionConstraint2 : public ik_constraint2::IKConstraint {
  public:
    //A_link中のA_localposの部位とB_link中のB_localposの部位を一致させる.
    //  このとき、world系で見たA-Bの位置姿勢エラーが、region内にあるようにする.
    //  maxError: エラーの頭打ち eval系
    //  weight: コスト関数の重み. error * weight^2 * error. 0の成分はjacobianやeqに含まれない. eval系
    //  precision: 収束判定の閾値 error * weightのノルムとこの値を比べる

    const cnoid::LinkPtr& A_link() const { return A_link_;}
    cnoid::LinkPtr& A_link() { return A_link_;}
    const cnoid::Isometry3& A_localpos() const { return A_localpos_;}
    cnoid::Isometry3& A_localpos() { return A_localpos_;}
    const cnoid::LinkPtr& B_link() const { return B_link_;}
    cnoid::LinkPtr& B_link() { return B_link_;}
    const cnoid::Isometry3& B_localpos() const { return B_localpos_;}
    cnoid::Isometry3& B_localpos() { return B_localpos_;}
    const double& maxError() const { return maxError_;}
    double& maxError() { return maxError_;}
    const double& precision() const { return precision_;}
    double& precision() { return precision_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& C() const { return C_;}
    Eigen::SparseMatrix<double,Eigen::RowMajor>& C() { return C_;}
    const cnoid::VectorX& dl() const { return dl_;}
    cnoid::VectorX& dl() { return dl_;}
    const cnoid::VectorX& du() const { return du_;}
    cnoid::VectorX& du() { return du_;}

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


    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<RegionConstraint2> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;


  protected:
    cnoid::LinkPtr A_link_ = nullptr;
    cnoid::Isometry3 A_localpos_ = cnoid::Isometry3::Identity();
    cnoid::LinkPtr B_link_ = nullptr;
    cnoid::Isometry3 B_localpos_ = cnoid::Isometry3::Identity();
    double maxError_ = 0.05;
    double precision_ = 1e-3;
    double weight_ = 1.0;
    Eigen::SparseMatrix<double,Eigen::RowMajor> C_{0,6}; // ? x 6
    Eigen::VectorXd dl_;
    Eigen::VectorXd du_;

    cnoid::SgLineSetPtr lines_;
    cnoid::Vector6 current_error_ = cnoid::Vector6::Zero();

    std::vector<cnoid::LinkPtr> path_A_joints_;
    std::vector<cnoid::LinkPtr> path_B_joints_;
    std::vector<cnoid::LinkPtr> path_BA_joints_;
    int path_BA_joints_numUpwardConnections_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianineq_full_;
    cnoid::LinkPtr jacobian_A_link_ = nullptr;// 前回のjacobian計算時のA_link
    cnoid::LinkPtr jacobian_B_link_ = nullptr;// 前回のjacobian計算時のB_link
    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobian_ColMap_;

  };
}

#endif
