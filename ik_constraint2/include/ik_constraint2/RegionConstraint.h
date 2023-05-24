#ifndef IK_CONSTRAINT2_REGIONCONSTRAINT_H
#define IK_CONSTRAINT2_REGIONCONSTRAINT_H

#include <ik_constraint2/IKConstraint.h>

namespace ik_constraint2{
  class RegionConstraint : public ik_constraint2::IKConstraint {
  public:
    //A_link中のA_localposの部位とB_link中のB_localposの部位を一致させる.
    //  このとき、eval_R系で見たA-Bの位置エラーが、region内にあるようにする.
    //リンクがnullptrならworld座標系を意味する
    //  maxError: エラーの頭打ち eval系
    //  weight: コスト関数の重み. error * weight^2 * error. 0の成分はjacobianやeqに含まれない. eval系
    //  precision: 収束判定の閾値 error * weightのノルムとこの値を比べる

    const cnoid::LinkPtr& A_link() const { return A_link_;}
    cnoid::LinkPtr& A_link() { return A_link_;}
    const cnoid::Position& A_localpos() const { return A_localpos_;}
    cnoid::Position& A_localpos() { return A_localpos_;}
    const cnoid::LinkPtr& B_link() const { return B_link_;}
    cnoid::LinkPtr& B_link() { return B_link_;}
    const cnoid::Position& B_localpos() const { return B_localpos_;}
    cnoid::Position& B_localpos() { return B_localpos_;}
    const double& maxError() const { return maxError_;}
    double& maxError() { return maxError_;}
    const cnoid::Vector3& maxRError() const { return maxRError_;}
    cnoid::Vector3& maxRError() { return maxRError_;}
    const double& precision() const { return precision_;}
    double& precision() { return precision_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}
    const cnoid::Vector3& weightR() const { return weightR_;}
    cnoid::Vector3& weightR() { return weightR_;}
    const cnoid::LinkPtr& eval_link() const { return eval_link_;}
    cnoid::LinkPtr& eval_link() { return eval_link_;}
    const cnoid::Matrix3d& eval_localR() const { return eval_localR_;}
    cnoid::Matrix3d& eval_localR() { return eval_localR_;}

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
    void copy(std::shared_ptr<RegionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;


  protected:
    cnoid::LinkPtr A_link_ = nullptr;
    cnoid::Position A_localpos_ = cnoid::Position::Identity();
    cnoid::LinkPtr B_link_ = nullptr;
    cnoid::Position B_localpos_ = cnoid::Position::Identity();
    double maxError_ = 0.05;
    cnoid::Vector3 maxRError_ = 0.05 * cnoid::Vector3::Ones();
    double precision_ = 1e-3;
    double weight_ = 1.0;
    cnoid::Vector3 weightR_ = cnoid::Vector3::Ones();
    cnoid::LinkPtr eval_link_ = nullptr;
    cnoid::Matrix3d eval_localR_ = cnoid::Matrix3d::Identity();
    Eigen::SparseMatrix<double,Eigen::RowMajor> C_{0,3}; // ? x 3
    Eigen::VectorXd dl_;
    Eigen::VectorXd du_;

    cnoid::SgLineSetPtr lines_;
    cnoid::Vector6 current_error_eval_ = cnoid::Vector6::Zero();

    std::vector<cnoid::LinkPtr> path_A_joints_;
    std::vector<cnoid::LinkPtr> path_B_joints_;
    std::vector<cnoid::LinkPtr> path_eval_joints_;
    int path_BA_joints_numUpwardConnections_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_A_full_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_B_full_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_eval_full_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_local_;
    cnoid::LinkPtr jacobian_A_link_ = nullptr;// 前回のjacobian計算時のA_link
    cnoid::LinkPtr jacobian_B_link_ = nullptr;// 前回のjacobian計算時のB_link
    cnoid::LinkPtr jacobian_eval_link_ = nullptr;// 前回のjacobian計算時のeval_link

    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;

  };
}

#endif
