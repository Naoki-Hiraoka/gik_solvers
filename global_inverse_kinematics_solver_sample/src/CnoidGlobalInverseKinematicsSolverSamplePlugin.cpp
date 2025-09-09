#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace global_inverse_kinematics_solver_sample{
  void sample1_4limb();
  class sample1_4limbItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_4limbItem>("sample1_4limbItem"); }
  protected:
    virtual void main() override{ sample1_4limb(); return; }
  };
  typedef cnoid::ref_ptr<sample1_4limbItem> sample1_4limbItemPtr;

  void sample2_desk();
  class sample2_deskItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample2_deskItem>("sample2_deskItem"); }
  protected:
    virtual void main() override{ sample2_desk(); return; }
  };
  typedef cnoid::ref_ptr<sample2_deskItem> sample2_deskItemPtr;

  void sample3_desk();
  class sample3_deskItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample3_deskItem>("sample3_deskItem"); }
  protected:
    virtual void main() override{ sample3_desk(); return; }
  };
  typedef cnoid::ref_ptr<sample3_deskItem> sample3_deskItemPtr;

  void sample4_jaxon(bool rejection);
  class sample4_jaxonItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample4_jaxonItem>("sample4_jaxonItem"); }
  protected:
    virtual void main() override{ sample4_jaxon(false); return; }
  };
  typedef cnoid::ref_ptr<sample4_jaxonItem> sample4_jaxonItemPtr;

  class sample4_jaxonrejItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample4_jaxonrejItem>("sample4_jaxonrejItem"); }
  protected:
    virtual void main() override{ sample4_jaxon(true); return; }
  };
  typedef cnoid::ref_ptr<sample4_jaxonrejItem> sample4_jaxonrejItemPtr;

  void sample5_jaxon();
  class sample5_jaxonItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample5_jaxonItem>("sample5_jaxonItem"); }
  protected:
    virtual void main() override{ sample5_jaxon(); return; }
  };
  typedef cnoid::ref_ptr<sample5_jaxonItem> sample5_jaxonItemPtr;

  void sample6_root();
  class sample6_rootItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample6_rootItem>("sample6_rootItem"); }
  protected:
    virtual void main() override{ sample6_root(); return; }
  };
  typedef cnoid::ref_ptr<sample6_rootItem> sample6_rootItemPtr;

  void sample7_rootrej();
  class sample7_rootrejItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample7_rootrejItem>("sample7_rootrejItem"); }
  protected:
    virtual void main() override{ sample7_rootrej(); return; }
  };
  typedef cnoid::ref_ptr<sample7_rootrejItem> sample7_rootrejItemPtr;

  void sample8_opt();
  class sample8_optItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample8_optItem>("sample8_optItem"); }
  protected:
    virtual void main() override{ sample8_opt(); return; }
  };
  typedef cnoid::ref_ptr<sample8_optItem> sample8_optItemPtr;

  void sample9_opt();
  class sample9_optItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample9_optItem>("sample9_optItem"); }
  protected:
    virtual void main() override{ sample9_opt(); return; }
  };
  typedef cnoid::ref_ptr<sample9_optItem> sample9_optItemPtr;

  class GlobalInverseKinematicsSolverSamplePlugin : public cnoid::Plugin
  {
  public:

    GlobalInverseKinematicsSolverSamplePlugin() : Plugin("GlobalInverseKinematicsSolverSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample1_4limbItem::initializeClass(this);
      sample2_deskItem::initializeClass(this);
      sample3_deskItem::initializeClass(this);
      sample4_jaxonItem::initializeClass(this);
      sample4_jaxonrejItem::initializeClass(this);
      sample5_jaxonItem::initializeClass(this);
      sample6_rootItem::initializeClass(this);
      sample7_rootrejItem::initializeClass(this);
      sample8_optItem::initializeClass(this);
      sample9_optItem::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(global_inverse_kinematics_solver_sample::GlobalInverseKinematicsSolverSamplePlugin)
