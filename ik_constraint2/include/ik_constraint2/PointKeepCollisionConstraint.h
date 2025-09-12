#ifndef IK_CONSTRAINT2_POINTKEEPCOLLISIONCONSTRAINT_H
#define IK_CONSTRAINT2_POINTKEEPCOLLISIONCONSTRAINT_H

#include <ik_constraint2/KeepCollisionConstraint.h>

namespace ik_constraint2{
  class PointKeepCollisionConstraint : public ik_constraint2::KeepCollisionConstraint {
    /*
      A_linkの形状は、FACE表現のeigenの行列形式で与えられるものとする.
      B_linkの形状は、点の集合であるとする.
     */
  public:
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<PointKeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

    // Aリンク形状のFACE表現. LinkA local frame
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& A_FACE_C() { return A_FACE_C_; }
    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& A_FACE_C() const { return A_FACE_C_; }
    std::vector<Eigen::VectorXd>& A_FACE_dl() { return A_FACE_dl_; }
    const std::vector<Eigen::VectorXd>& A_FACE_dl() const { return A_FACE_dl_; }
    std::vector<Eigen::VectorXd>& A_FACE_du() { return A_FACE_du_; }
    const std::vector<Eigen::VectorXd>& A_FACE_du() const { return A_FACE_du_; }
    const double& shrinkA() const { return shrinkA_;} // この値だけ縮小させる
    double& shrinkA() { return shrinkA_;}

    // Bリンク形状. LinkA local frame
    std::vector<Eigen::Vector3d>& B_POINT() {return B_POINT_;}
    const std::vector<Eigen::Vector3d>& B_POINT() const {return B_POINT_;}

  protected:
    //pはworld
    virtual bool computeCommonPoint(const cnoid::LinkPtr A_link,
                                    const cnoid::LinkPtr B_link,
                                    cnoid::Vector3& p, // common point. world frame
                                    double& distance, // AとBの距離. 負の値はpenetration depth
                                    Eigen::SparseMatrix<double,Eigen::RowMajor>& A_C, // ? * 3. linkA local frame. pとAが干渉するためのpの条件
                                    Eigen::VectorXd& A_dl,
                                    Eigen::VectorXd& A_du,
                                    Eigen::SparseMatrix<double,Eigen::RowMajor>& B_C, // ? * 3. linkB local frame. pとBが干渉するためのpの条件
                                    Eigen::VectorXd& B_dl,
                                    Eigen::VectorXd& B_du
                                    ) override;

    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > A_FACE_C_; // ? x 3. linkA local frame.
    std::vector<Eigen::VectorXd> A_FACE_dl_;
    std::vector<Eigen::VectorXd> A_FACE_du_;
    double shrinkA_ = 0.0;
    std::vector<Eigen::Vector3d> B_POINT_; // linkB local frame.

    Eigen::SparseMatrix<double,Eigen::RowMajor> I3_;
  };
}

#endif
