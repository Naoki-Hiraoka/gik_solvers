#pragma once

#include <cnoid/Body>
#include <ik_constraint2/IKConstraint.h>
#include <graph_search2/graph_search2.h>

namespace global_grid_planner{

  class GGPParam {
  public:
    int debugLevel = 0; // 0: no debug message. 1: time measure. 2: internal state

    enum class GridType
      {
       XYZ,
       XYZTHETA
      };
    GridType gridType = GridType::XYZ;

    double resolutionQ = 0.1;
    double resolutionP = 0.1;
    int resolutionR = 8;
    double maxTranslation = 3.0;

    double forwardScale = 1.0;

    graph_search2::Param gsParam;

    GGPParam(){
      gsParam.solverType = graph_search2::Param::SolverType::BREADH_FIRST;
    }
  };

  bool solveGGP(const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& goals,
                const GGPParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path = nullptr); // 0: states. 1: angles);



  void frame2Link(const std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& links);
  void link2Frame(const std::vector<cnoid::LinkPtr>& links, std::vector<double>& frame);

};
