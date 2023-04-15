#ifndef IK_CONSTRAINT2_IKCONSTRAINT_H
#define IK_CONSTRAINT2_IKCONSTRAINT_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <Eigen/Sparse>
#include <unordered_map>

namespace ik_constraint2{
  class IKConstraint
  {
  public:

    // 必ず,状態更新(qとdqとrootLinkのT,v,w) -> ForwardKinematics(true) -> calcCenterOfMass() -> updateBounds() -> (isSatisfied / getEq / getMin/MaxIneq / distance / getDrawOnObjects) -> updateJacobian() -> getJacobian / getJacobianIneq の順で呼ぶので、同じ処理を何度も行うのではなく最初に呼ばれる関数で1回だけ行って以降はキャッシュを使ってよい

    // 内部状態更新. eq, minIneq, maxIneqを生成
    virtual void updateBounds () = 0;
    // 内部状態更新. jacobian, jacobianIneqを生成
    virtual void updateJacobian (const std::vector<cnoid::LinkPtr>& joints) = 0;
    // 達成判定
    virtual bool isSatisfied () const {return this->distance() == 0.0;}
    // 達成までの距離. getEqなどは、エラーの頭打ちを行うが、distanceは行わないので、より純粋なisSatisfiedまでの距離を表す.
    virtual double distance() const { return std::sqrt(this->eq_.squaredNorm() + this->minIneq_.cwiseMax(Eigen::VectorXd::Zero(this->minIneq_.size())).squaredNorm() + this->maxIneq_.cwiseMin(Eigen::VectorXd::Zero(this->maxIneq_.size())).squaredNorm()); }
    // for debug view
    virtual std::vector<cnoid::SgNodePtr>& getDrawOnObjects() {return this->drawOnObjects_;}
    // 等式制約のエラーを返す.  // getEq = getJacobian * dq となるようなdqを探索する
    const Eigen::VectorXd& getEq () const { return this->eq_; }
    // 等式制約のヤコビアンを返す. bodyのroot6dof+全関節が変数
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getJacobian () const { return this->jacobian_; }
    // 不等式制約のヤコビアンを返す. bodyのroot6dof+全関節が変数  // getMinIneq <= getJacobianIneq * dq <= getMaxIneq() となるようなdqを探索する
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getJacobianIneq () const { return this->jacobianIneq_; }
    // 不等式制約のmin値を返す
    const Eigen::VectorXd& getMinIneq () const { return this->minIneq_; }
    // 不等式制約のmax値を返す
    const Eigen::VectorXd& getMaxIneq () const { return this->maxIneq_; }

    const int& debugLevel() const { return debugLevel_;}
    int& debugLevel() { return debugLevel_;}

    static inline size_t getJointDOF(const cnoid::LinkPtr& joint) {
      if(joint->isRevoluteJoint() || joint->isPrismaticJoint()) return 1;
      else if(joint->isFreeJoint()) return 6;
      else return 0;
    }
    static inline bool isJointsSame(const std::vector<cnoid::LinkPtr>& joints1,const std::vector<cnoid::LinkPtr>& joints2) {
      if (joints1.size() != joints2.size() ) return false;
      for(size_t i=0;i<joints1.size();i++){
        if (joints1[i] != joints2[i] ) return false;
      }
      return true;
    }
    static inline double clamp(const double& value, const double& limit_value) {
      return std::min(std::max(value, -limit_value), limit_value);
    }
    template<typename Derived>
    static inline typename Derived::PlainObject clamp(const Eigen::MatrixBase<Derived>& value, const Eigen::MatrixBase<Derived>& limit_value) {
      return value.array().max(-limit_value.array()).min(limit_value.array());
    }
    static inline Eigen::SparseMatrix<double,Eigen::RowMajor> cross(const cnoid::Vector3 v) {
      Eigen::SparseMatrix<double,Eigen::RowMajor> m(3,3);
      m.insert(0,1) = -v[2]; m.insert(0,2) = v[1];
      m.insert(1,0) = -v[2]; m.insert(1,2) = -v[0];
      m.insert(2,0) = -v[1]; m.insert(2,1) = v[0];
      return m;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:

    int debugLevel_ = 0;
    std::vector<cnoid::SgNodePtr> drawOnObjects_;

    Eigen::VectorXd eq_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_;
    Eigen::VectorXd minIneq_;
    Eigen::VectorXd maxIneq_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianIneq_;

  };
}

#endif
