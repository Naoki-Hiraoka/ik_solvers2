#ifndef IK_CONSTRAINT2_REGION_CDD_CDDREGIONCONSTRAINT_H
#define IK_CONSTRAINT2_REGION_CDD_CDDREGIONCONSTRAINT_H

#include <ik_constraint2/RegionConstraint.h>

namespace ik_constraint2_region_cdd{
  class CddRegionConstraint : public ik_constraint2::RegionConstraint {
  public:
    // V = [v1, v2, v3, ...]. Vはrowsが3. A-Bのeval_R系
    bool setVertices(const Eigen::MatrixXd V);

    // for debug view
    virtual std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;


    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<CddRegionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;


  protected:
    Eigen::Matrix<double, 3, Eigen::Dynamic> V_;
    cnoid::SgPointSetPtr points_;
  };
}

#endif
