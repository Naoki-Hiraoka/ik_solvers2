#ifndef IK_CONSTRAINT2_BULLET_BULLETKEEPCOLLISIONCONSTRAINT_H
#define IK_CONSTRAINT2_BULLET_BULLETKEEPCOLLISIONCONSTRAINT_H

#include <ik_constraint2/KeepCollisionConstraint.h>

class btConvexShape;

namespace ik_constraint2_bullet{
  class BulletKeepCollisionConstraint : public ik_constraint2::KeepCollisionConstraint {
  public:
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<BulletKeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

    // meshの点が少なく次元が縮退しているとqhullがエラーになるので、特にuseSingleMesh=falseのときは注意.
    bool& useSingleMeshA() { return useSingleMeshA_; }
    const bool& useSingleMeshA() const { return useSingleMeshA_; }
    bool& useSingleMeshB() { return useSingleMeshB_; }
    const bool& useSingleMeshB() const { return useSingleMeshB_; }

    // 各リンクのbullet model. *_link_bulletModelと*_linkが一致していなければ、自動で作成される.
    std::vector<std::shared_ptr<btConvexShape> >& A_bulletModel() { return A_bulletModel_; }
    const std::vector<std::shared_ptr<btConvexShape> >& A_bulletModel() const { return A_bulletModel_; }
    std::vector<std::shared_ptr<btConvexShape> >& B_bulletModel() { return B_bulletModel_; }
    const std::vector<std::shared_ptr<btConvexShape> >& B_bulletModel() const { return B_bulletModel_; }
    // *_bulletModel_作成時のの*_link
    cnoid::LinkPtr& A_link_bulletModel() { return A_link_bulletModel_; }
    const cnoid::LinkPtr& A_link_bulletModel() const { return A_link_bulletModel_; }
    cnoid::LinkPtr& B_link_bulletModel() { return B_link_bulletModel_; }
    const cnoid::LinkPtr& B_link_bulletModel() const { return B_link_bulletModel_; }

    // 各リンク形状のFACE表現. サイズは*_bulletModelと同じでなければならない.
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& A_FACE_C() { return A_FACE_C_; }
    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& A_FACE_C() const { return A_FACE_C_; }
    std::vector<Eigen::VectorXd>& A_FACE_dl() { return A_FACE_dl_; }
    const std::vector<Eigen::VectorXd>& A_FACE_dl() const { return A_FACE_dl_; }
    std::vector<Eigen::VectorXd>& A_FACE_du() { return A_FACE_du_; }
    const std::vector<Eigen::VectorXd>& A_FACE_du() const { return A_FACE_du_; }
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& B_FACE_C() { return B_FACE_C_; }
    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& B_FACE_C() const { return B_FACE_C_; }
    std::vector<Eigen::VectorXd>& B_FACE_dl() { return B_FACE_dl_; }
    const std::vector<Eigen::VectorXd>& B_FACE_dl() const { return B_FACE_dl_; }
    std::vector<Eigen::VectorXd>& B_FACE_du() { return B_FACE_du_; }
    const std::vector<Eigen::VectorXd>& B_FACE_du() const { return B_FACE_du_; }

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
                                    ) override;

    bool useSingleMeshA_ = true;
    bool useSingleMeshB_ = true;

    std::vector<std::shared_ptr<btConvexShape> > A_bulletModel_;
    cnoid::LinkPtr A_link_bulletModel_; // A_bulletModel_のA_link
    std::vector<std::shared_ptr<btConvexShape> > B_bulletModel_;
    cnoid::LinkPtr B_link_bulletModel_; // B_bulletModel_のB_link

    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > A_FACE_C_; // ? x 3. linkA local frame. 要素数と順序はA_bulletModelと同じ
    std::vector<Eigen::VectorXd> A_FACE_dl_;
    std::vector<Eigen::VectorXd> A_FACE_du_;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > B_FACE_C_; // ? x 3. linkB local frame. 要素数と順序はA_bulletModelと同じ
    std::vector<Eigen::VectorXd> B_FACE_dl_;
    std::vector<Eigen::VectorXd> B_FACE_du_;

  };
}

#endif
