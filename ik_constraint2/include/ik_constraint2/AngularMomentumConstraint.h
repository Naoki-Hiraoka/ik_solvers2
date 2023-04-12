#ifndef IK_CONSTRAINT2_ANGULARMOMENTUMCONSTRAINT_H
#define IK_CONSTRAINT2_ANGULARMOMENTUMCONSTRAINT_H

#include <ik_constraint2/IKConstraint.h>
#include <cnoid/EigenUtil>
#include <cnoid/SceneMarkers>
#include <iostream>

namespace ik_constraint2{
  class AngularMomentumConstraint : public IKConstraint
  {
  public:
    //robotの重心周りの角運動量を目標の値[kg m^2/s]に一致させる
    //  内部の処理では角運動量を重心周りのイナーシャで割ってdtでかけて、[rad]の次元で扱う
    //  dt: [s]
    //  maxError: エラーの頭打ち[rad]
    //  weight: コスト関数の重み. error * weight^2 * error.
    //  precision: 収束判定の閾値[rad]. error * weightのノルムと比べる
    //  targetAngularMomentum: 重心周り. ワールド座標系. [kg m^2/s]
    //状態が更新される度に, 手動でcalcForwardKinematics()とcalcCenterOfMass()を呼ぶ必要が有る.
    const cnoid::BodyPtr& robot() const { return robot_;}
    cnoid::BodyPtr& robot() { return robot_;}
    const cnoid::Matrix3d& eval_R() const { return eval_R_;}
    cnoid::Matrix3d& eval_R() { return eval_R_;}

    const cnoid::Vector3& targetAngularMomentum() const { return targetAngularMomentum_;}
    cnoid::Vector3& targetAngularMomentum() { return targetAngularMomentum_;}
    const double& dt() const { return dt_;}
    double& dt() { return dt_;}

    const cnoid::Vector3& maxError() const { return maxError_;}
    cnoid::Vector3& maxError() { return maxError_;}
    const double& precision() const { return precision_;}
    double& precision() { return precision_;}
    const cnoid::Vector3& weight() const { return weight_;}
    cnoid::Vector3& weight() { return weight_;}

    //内部状態更新
    virtual void update (const std::vector<cnoid::LinkPtr>& joints) override;
    // 達成判定
    virtual bool isSatisfied () const override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    cnoid::BodyPtr robot_ = nullptr;
    cnoid::Matrix3d eval_R_ = cnoid::Matrix3d::Identity();

    cnoid::Vector3 targetAngularMomentum_ = cnoid::Vector3::Ones();
    double dt_ = 1e-2;

    cnoid::Vector3 maxError_ = 0.05 * cnoid::Vector3::Ones();
    double precision_ = 1e-4;
    cnoid::Vector3 weight_ = cnoid::Vector3::Ones();

    cnoid::BodyPtr jacobian_robot_ = nullptr;// 前回のjacobian計算時のrobot
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_full_;

    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;


    static void calcAngularMomentumJacobianShape(const std::vector<cnoid::LinkPtr>& joints,//input
                                                 const cnoid::BodyPtr& A_robot,//input
                                                 const cnoid::BodyPtr& B_robot,//input
                                                 Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian,//output
                                                 std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap //output
                                                 );
    static void calcAngularMomentumJacobianCoef(const std::vector<cnoid::LinkPtr>& joints,//input
                                                const cnoid::BodyPtr& A_robot,//input
                                                const cnoid::BodyPtr& B_robot,//input
                                                std::unordered_map<cnoid::LinkPtr,int>& jacobianColMap, //input
                                                Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian//output
                                                );
  };

  namespace cnoid18 {
    // choreonoidのrelease1.7の calcAngularMomentumJacobianにはバグがあり、開発版ではhttps://github.com/s-nakaoka/choreonoid/pull/234 で修正されている. 修正された版の関数(https://github.com/choreonoid/choreonoid/blob/master/src/Body/Jacobian.cpp )を使う
    void calcAngularMomentumJacobian(cnoid::Body* body, cnoid::Link* base, Eigen::MatrixXd& H);
  }
}

#endif
