#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace prioritized_inverse_kinematics_solver2_sample{
  void sample1_4limb();
  class sample1_4limbItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_4limbItem>("sample1_4limbItem"); }
  protected:
    virtual void main() override{ sample1_4limb(); return; }
  };
  typedef cnoid::ref_ptr<sample1_4limbItem> sample1_4limbItemPtr;

  class PrioritizedInverseKinematicsSolver2SamplePlugin : public cnoid::Plugin
  {
  public:

    PrioritizedInverseKinematicsSolver2SamplePlugin() : Plugin("PrioritizedInverseKinematicsSolver2Sample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample1_4limbItem::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(prioritized_inverse_kinematics_solver2_sample::PrioritizedInverseKinematicsSolver2SamplePlugin)
