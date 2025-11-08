#ifndef IK_CONSTRAINT2_POINTKEEPCOLLISIONCONSTRAINT_H
#define IK_CONSTRAINT2_POINTKEEPCOLLISIONCONSTRAINT_H

#include <ik_constraint2/KeepCollisionConstraint.h>

namespace ik_constraint2{
  class PointKeepCollisionConstraint : public ik_constraint2::IKConstraint {
  public:
    // A_linkと点群Bを干渉させる. A_linkはnullptrであってはならない.
    //  precision: 収束判定の閾値. distance - torelanceと比べる
    //  maxError: エラーの頭打ち
    //  weight: コスト関数の重み. error * weight^2 * error. maxErrorの適用後に適用する
    //  velocityDamper: 不等式制約の差分をこの値分の1にする. maxErrorの適用前に適用する.
    //  ignorePenetration: この距離以上めりこんでいる場合、制約をフリーにすることで、最適化計算を高速化する. (シュミットトリガー方式の方が行列の形状が変わりにくいのでSQPが利用できていいかも)

    const cnoid::LinkPtr& A_link() const { return A_link_;}
    cnoid::LinkPtr& A_link() { return A_link_;}

    // Aリンク形状のFACE表現. LinkA local frame. cols = 3
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& A_FACE_C() { return A_FACE_C_; }
    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& A_FACE_C() const { return A_FACE_C_; }
    std::vector<Eigen::VectorXd>& A_FACE_dl() { return A_FACE_dl_; }
    const std::vector<Eigen::VectorXd>& A_FACE_dl() const { return A_FACE_dl_; }
    std::vector<Eigen::VectorXd>& A_FACE_du() { return A_FACE_du_; }
    const std::vector<Eigen::VectorXd>& A_FACE_du() const { return A_FACE_du_; }
    const double& shrinkA() const { return shrinkA_;} // この値だけ縮小させる
    double& shrinkA() { return shrinkA_;}

    // 点群B. LinkA local frame
    std::vector<Eigen::Vector3d>& B_POINT() { B_POINT_changed_ = true; return B_POINT_;}
    const std::vector<Eigen::Vector3d>& B_POINT() const {return B_POINT_;}
    const double& maxError() const { return maxError_;}
    double& maxError() { return maxError_;}
    const double& precision() const { return precision_;}
    double& precision() { return precision_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}
    const double& ignorePenetration() const { return ignorePenetration_;}
    double& ignorePenetration() { return ignorePenetration_;}
    const cnoid::Vector3& currentp() const { return currentp_;}

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
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    virtual void copy(std::shared_ptr<PointKeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

  protected:

    cnoid::LinkPtr A_link_ = nullptr;
    double maxError_ = 0.05;
    double precision_ = 1e-3;
    double weight_ = 1.0;
    double ignorePenetration_ = 0.1;

    Eigen::SparseMatrix<double,Eigen::RowMajor> A_currentC_{0,3}; // ? x 3. linkA local frame
    Eigen::VectorXd A_currentdl_;
    Eigen::VectorXd A_currentdu_;

    cnoid::Vector3 currentp_ = cnoid::Vector3::Zero(); //world frame
    double currentDistance_ = 0.0;

    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のupdateJacobian時のjoints
    cnoid::LinkPtr jacobian_A_link_ = nullptr;// 前回のjacobian計算時のA_link
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;
    std::vector<cnoid::LinkPtr> path_A_joints_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_A_full_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_A_local_; // p-Aのヤコビアン(A local)

    cnoid::SgPointSetPtr points_ = nullptr;
    std::vector<cnoid::SgNodePtr> dummyDrawOnObjects_;

    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > A_FACE_C_; // ? x 3. linkA local frame.
    std::vector<Eigen::VectorXd> A_FACE_dl_;
    std::vector<Eigen::VectorXd> A_FACE_du_;
    double shrinkA_ = 0.0;
    std::vector<Eigen::Vector3d> B_POINT_; // linkB local frame.

    bool B_POINT_changed_ = false;
    Eigen::MatrixXd B_POINT_mat_{4,0}; // linkB local frame.

    std::vector<Eigen::MatrixXd> value; // メモリ確保回数を減らすため. B_POINTが巨大な場合に有用
  };
}

#endif
