#ifndef IK_CONSTRAINT2_JOINTLIMITMINMAXTABLECONSTRAINT_H
#define IK_CONSTRAINT2_JOINTLIMITMINMAXTABLECONSTRAINT_H

#include <ik_constraint2/JointLimitConstraint.h>
#include <joint_limit_table/JointLimitTable.h>

namespace ik_constraint_joint_limit_table2{
  class JointLimitMinMaxTableConstraint : public ik_constraint2::JointLimitConstraint {
  public:
    //jointのqをq_upperとq_lowerの間かつmin-max-tableの間にさせる.

    const std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> >& jointLimitTables() const { return jointLimitTables_;}
    std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> >& jointLimitTables() { return jointLimitTables_;}

  protected:
    virtual void calcMinMaxIneq(Eigen::VectorXd& maxIneq, Eigen::VectorXd& minIneq) override;
    std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTables_;
  };
}

#endif
