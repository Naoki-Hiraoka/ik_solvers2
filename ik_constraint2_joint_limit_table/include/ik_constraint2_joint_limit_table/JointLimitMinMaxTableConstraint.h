#ifndef IK_CONSTRAINT2_JOINT_LIMIT_TABLE_JOINTLIMITMINMAXTABLECONSTRAINT_H
#define IK_CONSTRAINT2_JOINT_LIMIT_TABLE_JOINTLIMITMINMAXTABLECONSTRAINT_H

#include <ik_constraint2/JointLimitConstraint.h>
#include <joint_limit_table/JointLimitTable.h>

namespace ik_constraint2_joint_limit_table{
  class JointLimitMinMaxTableConstraint : public ik_constraint2::JointLimitConstraint {
  public:
    //jointのqをq_upperとq_lowerの間かつmin-max-tableの間にさせる.

    const std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> >& jointLimitTables() const { return jointLimitTables_;}
    std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> >& jointLimitTables() { return jointLimitTables_;}
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<JointLimitMinMaxTableConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;


  protected:
    virtual void calcMinMaxIneq(Eigen::VectorXd& maxIneq, Eigen::VectorXd& minIneq) override;
    std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTables_;
  };
}

#endif
