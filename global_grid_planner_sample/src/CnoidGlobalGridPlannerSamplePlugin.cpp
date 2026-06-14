#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace global_grid_planner_sample{
  void sample1();
  class sample1Item : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1Item>("sample1Item"); }
  protected:
    virtual void main() override{ sample1(); return; }
  };
  typedef cnoid::ref_ptr<sample1Item> sample1ItemPtr;

  void sample2();
  class sample2Item : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample2Item>("sample2Item"); }
  protected:
    virtual void main() override{ sample2(); return; }
  };
  typedef cnoid::ref_ptr<sample2Item> sample2ItemPtr;

  class GlobalGridPlannerSamplePlugin : public cnoid::Plugin
  {
  public:

    GlobalGridPlannerSamplePlugin() : Plugin("GlobalGridPlannerSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample1Item::initializeClass(this);
      sample2Item::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(global_grid_planner_sample::GlobalGridPlannerSamplePlugin)
