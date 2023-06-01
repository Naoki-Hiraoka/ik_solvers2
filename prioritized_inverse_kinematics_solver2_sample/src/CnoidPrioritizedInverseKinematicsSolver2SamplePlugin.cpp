#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace prioritized_inverse_kinematics_solver2_sample{
  void sample0_1_bullet();
  class sample0_1_bulletItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample0_1_bulletItem>("sample0_1_bulletItem"); }
  protected:
    virtual void main() override{ sample0_1_bullet(); return; }
  };
  typedef cnoid::ref_ptr<sample0_1_bulletItem> sample0_1_bulletItemPtr;

  void sample0_2_bullet();
  class sample0_2_bulletItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample0_2_bulletItem>("sample0_2_bulletItem"); }
  protected:
    virtual void main() override{ sample0_2_bullet(); return; }
  };
  typedef cnoid::ref_ptr<sample0_2_bulletItem> sample0_2_bulletItemPtr;

  void sample1_4limb();
  class sample1_4limbItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_4limbItem>("sample1_4limbItem"); }
  protected:
    virtual void main() override{ sample1_4limb(); return; }
  };
  typedef cnoid::ref_ptr<sample1_4limbItem> sample1_4limbItemPtr;

  void sample2_collision();
  class sample2_collisionItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample2_collisionItem>("sample2_collisionItem"); }
  protected:
    virtual void main() override{ sample2_collision(); return; }
  };
  typedef cnoid::ref_ptr<sample2_collisionItem> sample2_collisionItemPtr;

  void sample3_collision();
  class sample3_collisionItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample3_collisionItem>("sample3_collisionItem"); }
  protected:
    virtual void main() override{ sample3_collision(); return; }
  };
  typedef cnoid::ref_ptr<sample3_collisionItem> sample3_collisionItemPtr;

  void sample4_region();
  class sample4_regionItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample4_regionItem>("sample4_regionItem"); }
  protected:
    virtual void main() override{ sample4_region(); return; }
  };
  typedef cnoid::ref_ptr<sample4_regionItem> sample4_regionItemPtr;

  void sample5_or();
  class sample5_orItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample5_orItem>("sample5_orItem"); }
  protected:
    virtual void main() override{ sample5_or(); return; }
  };
  typedef cnoid::ref_ptr<sample5_orItem> sample5_orItemPtr;

  void sample6_keep();
  class sample6_keepItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample6_keepItem>("sample6_keepItem"); }
  protected:
    virtual void main() override{ sample6_keep(); return; }
  };
  typedef cnoid::ref_ptr<sample6_keepItem> sample6_keepItemPtr;

  class PrioritizedInverseKinematicsSolver2SamplePlugin : public cnoid::Plugin
  {
  public:

    PrioritizedInverseKinematicsSolver2SamplePlugin() : Plugin("PrioritizedInverseKinematicsSolver2Sample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample0_1_bulletItem::initializeClass(this);
      sample0_2_bulletItem::initializeClass(this);
      sample1_4limbItem::initializeClass(this);
      sample2_collisionItem::initializeClass(this);
      sample3_collisionItem::initializeClass(this);
      sample4_regionItem::initializeClass(this);
      sample5_orItem::initializeClass(this);
      sample6_keepItem::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(prioritized_inverse_kinematics_solver2_sample::PrioritizedInverseKinematicsSolver2SamplePlugin)
