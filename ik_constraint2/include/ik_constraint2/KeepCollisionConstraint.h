#ifndef IK_CONSTRAINT2_KEEPCOLLISIONCONSTRAINT_H
#define IK_CONSTRAINT2_KEEPCOLLISIONCONSTRAINT_H

#include <ik_constraint2/IKConstraint.h>

namespace ik_constraint2{
  class KeepCollisionConstraint : public IKConstraint {
  public:
    // A_linkとB_linkを干渉させる.
    //  tolerance: この値以上めりこませる[m]
    //  precision: 収束判定の閾値. distance - torelanceと比べる
    //  maxError: エラーの頭打ち
    //  weight: コスト関数の重み. error * weight^2 * error. maxErrorの適用後に適用する
    //  velocityDamper: 不等式制約の差分をこの値分の1にする. maxErrorの適用前に適用する.
    //  ignorePenetration: この距離以上めりこんでいる場合、制約をフリーにすることで、最適化計算を高速化する. (シュミットトリガー方式の方が行列の形状が変わりにくいのでSQPが利用できていいかも)

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
    const double& ignorePenetration() const { return ignorePenetration_;}
    double& ignorePenetration() { return ignorePenetration_;}

    // 内部状態更新. eq, minIneq, maxIneqを生成
    virtual void updateBounds () override;
    // 内部状態更新. jacobian, jacobianIneqを生成
    virtual void updateJacobian (const std::vector<cnoid::LinkPtr>& joints) override;
    // 達成判定
    virtual bool isSatisfied () const override;
    // 達成までの距離. getEqなどは、エラーの頭打ちを行うが、distanceは行わないので、より純粋なisSatisfiedまでの距離を表す.
    virtual double distance() const override;
    // 制約を満たさなくなるまでの最短距離. 現在満たしていない場合は-distanceと同じ. getEqなどは、エラーの頭打ちを行うが、marginは行わないので、より純粋な距離を表す.
    virtual double margin() const override;

    // for debug view
    virtual std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    void copy(std::shared_ptr<KeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

  protected:
    //pはworld
    virtual bool computeCommonPoint(const cnoid::LinkPtr A_link,
                                    const cnoid::LinkPtr B_link,
                                    cnoid::Vector3& p, // common point. world frame
                                    double& distance, // AとBの距離. 負の値はpenetration depth
                                    Eigen::SparseMatrix<double,Eigen::RowMajor>& A_C, // ? * 3. linkA local frame. pとAが干渉するためのpの条件
                                    Eigen::VectorXd& A_dl,
                                    Eigen::VectorXd& A_du,
                                    Eigen::SparseMatrix<double,Eigen::RowMajor>& B_C, // ? * 3. linkA local frame. pとBが干渉するためのpの条件
                                    Eigen::VectorXd& B_dl,
                                    Eigen::VectorXd& B_du
                                    )=0;

  private:
    cnoid::LinkPtr A_link_ = nullptr;
    cnoid::LinkPtr B_link_ = nullptr;
    double tolerance_ = 0.0;
    double maxError_ = 0.05;
    double precision_ = 1e-3;
    double weight_ = 1.0;
    double velocityDamper_ = 1.0;
    double ignorePenetration_ = 0.1;

    Eigen::SparseMatrix<double,Eigen::RowMajor> A_currentC_{0,3}; // ? x 3. linkA local frame
    Eigen::VectorXd A_currentdl_;
    Eigen::VectorXd A_currentdu_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> B_currentC_{0,3}; // ? x 3. linkB local frame
    Eigen::VectorXd B_currentdl_;
    Eigen::VectorXd B_currentdu_;

    cnoid::Vector3 currentp_ = cnoid::Vector3::Zero(); //world frame
    double currentDistance_ = 0.0;

    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianineq_full_;

    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のupdateJacobian時のjoints
    cnoid::LinkPtr jacobian_A_link_ = nullptr;// 前回のjacobian計算時のA_link
    cnoid::LinkPtr jacobian_B_link_ = nullptr;// 前回のjacobian計算時のB_link
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;
    std::vector<cnoid::LinkPtr> path_A_joints_;
    std::vector<cnoid::LinkPtr> path_B_joints_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_A_full_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_B_full_;

    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_A_local_; // p-Aのヤコビアン(A local)
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_B_local_; // p-Bのヤコビアン(A local)
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_A_ext_local_; // p-Aのヤコビアン(A local)
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_B_ext_local_; // p-Bのヤコビアン(A local)

  };
}

#endif
