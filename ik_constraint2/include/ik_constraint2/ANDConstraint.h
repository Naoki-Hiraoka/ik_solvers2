#ifndef IK_CONSTRAINT2_ANDCONSTRAINT_H
#define IK_CONSTRAINT2_ANDCONSTRAINT_H

#include <ik_constraint2/IKConstraint.h>
#include <iostream>

namespace ik_constraint2{
  class ANDConstraint : public IKConstraint
  {
  public:
    //Childrenのどれか一つを満たす
    //具体的には、childrenのなかで最もmarginが大きいもののみを有効にする
    //equality constraintを持つ制約がchildrenにあるとうまく動かないので注意
    const std::vector<std::shared_ptr<IKConstraint> >& children() const { return children_;}
    std::vector<std::shared_ptr<IKConstraint> >& children() { return children_;}

    // 内部状態更新. eq, minIneq, maxIneqを生成
    virtual void updateBounds () override;
    // 内部状態更新. jacobian, jacobianIneqを生成
    virtual void updateJacobian (const std::vector<cnoid::LinkPtr>& joints) override;
    // 達成判定
    virtual bool isSatisfied () const override;
    // 達成までの距離. getEqなどは、エラーの頭打ちを行うが、distanceは行わないので、より純粋なisSatisfiedまでの距離を表す.
    virtual double distance() const override;
    // 制約を満たさなくなるまでの最短距離. 現在満たしていない場合は-distanceと同じ. getEqなどは、エラーの頭打ちを行うが、marginは行わないので、より純粋な距離を表す.
    virtual double margin() const override;    // for debug view
    virtual std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<ANDConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    std::vector<std::shared_ptr<IKConstraint> > children_;
  };
}

#endif
