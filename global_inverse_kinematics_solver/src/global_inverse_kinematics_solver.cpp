#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>

namespace global_inverse_kinematics_solver{
  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, // 0: constriant priority 1: constraints
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& goals, // 0: goals(AND).
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals, // 0: nominals
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){ // 0: states. 1: angles
    return solveGIK(variables,
                    constraints,
                    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >{goals},
                    nominals,
                    param,
                    path);
  }

  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, // 0: constriant priority 1: constraints
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals, // 0: projection priority 1: goals.
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals, // 0: nominals
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){ // 0: states. 1: angles

    std::vector<std::vector<cnoid::LinkPtr> > variabless{variables};
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > constraintss{constraints};
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > goalss{goals};
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > nominalss{nominals};
    std::shared_ptr<UintQueue> modelQueue = std::make_shared<UintQueue>();
    std::vector<std::map<cnoid::BodyPtr, cnoid::BodyPtr> > modelMaps(1);
    GIKParam param2(param);
    modelQueue->push(0);

    if(param.threads >= 2){
      std::set<cnoid::BodyPtr> bodies = getBodies(variables);
      for(int i=1;i<param.threads;i++){
        modelQueue->push(i);
        std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
        {
          /*
             cnoid::Body::clone()は、各リンクのメッシュは複製せず、ポインタで共有する.
             この仕様は高速な複製のために有益である.
             一方で、同一のメッシュオブジェクトを共有するBody間で、
               - Body::clone()
               - Body::~Body()
             などがスレッドセーフでないことに注意が必要である.
             solveGIK関数をマルチスレッドから呼ぶ場合に問題になる場合がある.
          */
          std::shared_ptr<std::lock_guard<std::mutex> > guard;
          if(param.modelMutex) guard = std::make_shared<std::lock_guard<std::mutex> >(*(param.modelMutex));
          for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
            modelMap[*it] = (*it)->clone(); // cloneしたbodyがデストラクトされないように、保管しておく
          }
        }
        modelMaps.push_back(modelMap);
        variabless.push_back(std::vector<cnoid::LinkPtr>(variables.size()));
        for(int v=0;v<variables.size();v++){
          variabless.back()[v] = modelMap[variables[v]->body()]->link(variables[v]->index());
        }
        constraintss.push_back(std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >(constraints.size()));
        for(int j=0;j<constraints.size();j++){
          constraintss.back()[j].resize(constraints[j].size());
          for(int k=0;k<constraints[j].size();k++){
            constraintss.back()[j][k] = constraints[j][k]->clone(modelMap);
          }
        }
        goalss.push_back(std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >(goals.size()));
        for(int j=0;j<goals.size();j++){
          goalss.back()[j].resize(goals[j].size());
          for(int k=0;k<goals[j].size();k++){
            goalss.back()[j][k] = goals[j][k]->clone(modelMap);
          }
        }
        nominalss.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(nominals.size()));
        for(int j=0;j<nominals.size();j++){
          nominalss.back()[j] = nominals[j]->clone(modelMap);
        }
        if(param.projectLink.size() == 1){
          param2.projectLink.push_back(modelMap[param.projectLink[0]->body()]->link(param.projectLink[0]->index()));
        }
      }
    }

    bool result = solveGIK(variabless,
                           constraintss,
                           goalss,
                           nominalss,
                           modelQueue,
                           param2,
                           path);

    {
      std::shared_ptr<std::lock_guard<std::mutex> > guard;
      if(param.modelMutex) guard = std::make_shared<std::lock_guard<std::mutex> >(*(param.modelMutex));
      variabless.clear();
      constraintss.clear();
      goalss.clear();
      nominalss.clear();
      modelMaps.clear();
      param2.projectLink.clear();
    }

    return result;
  }

  bool solveGIK(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, // 0: modelQueue, 1: constriant priority 2: constraints
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& goals, // 0: modelQueue. 1: projection priority 2: goals.
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& nominals, // 0: modelQueue, 1: nominals
                std::shared_ptr<UintQueue> modelQueue,
                const GIKParam& param,
                const std::shared_ptr<std::vector<std::vector<double> > >& path){

    if((variables.size() == 0) ||
       (variables.size() != constraints.size()) ||
       (constraints.size() != goals.size()) ||
       (goals.size() != nominals.size())
       ){
      std::cerr << "[solveGIK] size mismatch" << std::endl;
      return false;
    }

    bool calculate_path = true;
    if(path == nullptr) {
      calculate_path = false;
    }

    ompl::base::StateSpacePtr ambientSpace = createAmbientSpace(variables[0], param.maxTranslation);
    GIKConstraintPtr gikConstraint = std::make_shared<GIKConstraint>(modelQueue, constraints, variables);
    gikConstraint->viewer() = param.viewer;
    gikConstraint->drawLoop() = param.drawLoop;
    gikConstraint->param() = param.pikParam;
    gikConstraint->param().satisfiedConvergeLevel = -1;
    gikConstraint->nearMaxError() = param.nearMaxError;
    GIKStateSpacePtr stateSpace = std::make_shared<GIKStateSpace>(ambientSpace, gikConstraint);
    stateSpace->setDelta(param.delta); // この距離内のstateは、中間のconstraintチェック無しで遷移可能
    ompl_near_projection::NearConstrainedSpaceInformationPtr spaceInformation = std::make_shared<ompl_near_projection::NearConstrainedSpaceInformation>(stateSpace);
    spaceInformation->setStateValidityChecker(std::make_shared<ompl::base::AllValidStateValidityChecker>(spaceInformation)); // validは全てconstraintでチェックするので、StateValidityCheckerは全てvalidでよい
    spaceInformation->setup(); // ここでsetupを呼ばないと、stateSpaceがsetupされないのでlink2State等ができない

    ompl::base::ProblemDefinitionPtr problemDefinition = std::make_shared<ompl::base::ProblemDefinition>(spaceInformation);

    ompl::base::ScopedState<> start(stateSpace);
    link2State(variables[0], stateSpace, start.get());

    stateSpace->enforceBounds(start.get()); // 僅かにjoint limitを超えていた場合、エラーになってしまうので
    problemDefinition->clearStartStates();
    problemDefinition->addStartState(start);

    GIKGoalSpacePtr goalSpace = std::make_shared<GIKGoalSpace>(spaceInformation, ambientSpace, modelQueue, constraints, variables, goals, nominals);

    problemDefinition->setGoal(goalSpace);

    ompl::base::PlannerPtr planner;

    if(param.planner == 0){
      if(param.projectLink.size() == variables.size()){
        GIKProjectionEvaluatorPtr proj = std::make_shared<GIKProjectionEvaluator>(stateSpace, modelQueue, variables);
        proj->parentLink() = param.projectLink;
        proj->localPos() = param.projectLocalPose;
        proj->setCellSizes(std::vector<double>(proj->getDimension(), param.projectCellSize));

        if(param.threads <= 1){
          std::shared_ptr<ompl_near_projection::geometric::NearKPIECE1> planner_ = std::make_shared<ompl_near_projection::geometric::NearKPIECE1>(spaceInformation);
          planner_->setProjectionEvaluator(proj);
          planner_->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
          planner_->setGoalBias(param.goalBias);

          planner = planner_;

        }else{
          std::shared_ptr<ompl_near_projection::geometric::pNearKPIECE1> planner_ = std::make_shared<ompl_near_projection::geometric::pNearKPIECE1>(spaceInformation);
          planner_->setProjectionEvaluator(proj);
          planner_->setThreadCount(param.threads);
          planner_->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
          planner_->setGoalBias(param.goalBias);
          planner = planner_;

        }
      }else{
        std::cerr << "projectLink is not valid" << std::endl;
        return false;
      }
    }else if(param.planner == 1){
      if(param.threads <= 1){
        std::shared_ptr<ompl_near_projection::geometric::NearEST> planner_ = std::make_shared<ompl_near_projection::geometric::NearEST>(spaceInformation);
        planner_->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
        planner_->setGoalBias(param.goalBias);
        planner = planner_;
      }else{
        std::shared_ptr<ompl_near_projection::geometric::pNearEST> planner_ = std::make_shared<ompl_near_projection::geometric::pNearEST>(spaceInformation);
        planner_->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
        planner_->setGoalBias(param.goalBias);
        planner_->setThreadCount(param.threads);
        planner = planner_;
      }
    }else if(param.planner == 2){
      if(param.threads <= 1){
        std::shared_ptr<ompl_near_projection::geometric::NearRRT> planner_ = std::make_shared<ompl_near_projection::geometric::NearRRT>(spaceInformation);
        planner_->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
        planner_->setGoalBias(param.goalBias);
        planner = planner_;
      }else{
        std::shared_ptr<ompl_near_projection::geometric::pNearRRT> planner_ = std::make_shared<ompl_near_projection::geometric::pNearRRT>(spaceInformation);
        planner_->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
        planner_->setGoalBias(param.goalBias);
        planner_->setThreadCount(param.threads);
        planner = planner_;
      }
    }else{
      std::cerr << "invalid planner" << std::endl;
      return false;
    }

    if(!spaceInformation->isSetup()) spaceInformation->setup();
    planner->setProblemDefinition(problemDefinition);
    if(!planner->isSetup()) planner->setup();

    // たまに、サンプル時のIKと補間時のIKが微妙に違うことが原因で、解のpathの補間に失敗する場合があるので、成功するまでとき直す. path.check()と、interpolateやsimplifyのIKは同じものなので、最初のpath.check()が成功すれば、simplifyやinterpolate後のpathも必ずcheck()が成功すると想定.
    ompl::base::PlannerStatus solved;

    //planner->clear();
    //problemDefition->clearSolutionPaths();

    {
      ompl::time::point start = ompl::time::now();
      solved = planner->solve(ompl::base::plannerOrTerminationCondition(ompl::base::timedPlannerTerminationCondition(param.timeout),
                                                                        param.ptc));
      double planTime = ompl::time::seconds(ompl::time::now() - start);
      if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
        OMPL_INFORM("Solution found in %f seconds", planTime);
      else
        OMPL_INFORM("No solution found after %f seconds", planTime);
    }

    // IKの途中経過を使っているので、EXACT_SOLUTIONなら、必ずinterpolateできている.
    // 一時的にstateが最適化の誤差で微妙に!isSatisfiedになっていたり、deltaを上回った距離になっていることがあって、それは許容したい.
    // solutionPath.check()は行わない

    if(calculate_path){
      if(!problemDefinition->hasSolution()) {
        path->resize(0);
      }else{
        ompl::geometric::PathGeometricPtr solutionPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(problemDefinition->getSolutionPath());

        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        // ompl::geometric::PathSimplifierPtr pathSimplifier = std::make_shared<ompl::geometric::PathSimplifier>(spaceInformation);
        // ompl::time::point start = ompl::time::now();
        // std::size_t numStates = solutionPath->getStateCount();
        // gikConstraint->viewer() = nullptr; // simplifySolution()中は描画しない
        // pathSimplifier->simplify(*solutionPath, param.timeout);
        // gikConstraint->viewer() = param.viewer; // simplifySolution()中は描画しない
        // double simplifyTime = ompl::time::seconds(ompl::time::now() - start);
        // OMPL_INFORM("Path simplification took %f seconds and changed from %d to %d states",
        //             simplifyTime, numStates, solutionPath->getStateCount());
        // if(param.debugLevel > 1){
        //   solutionPath->print(std::cout);
        // }

        solutionPath->interpolate();
        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        // 途中の軌道をpathに入れて返す
        path->resize(solutionPath->getStateCount());
        for(int j=0;j<solutionPath->getStateCount();j++){
          //stateSpace->getDimension()は,SO3StateSpaceが3を返してしまう(実際はquaternionで4)ので、使えない
          state2Frame(stateSpace, solutionPath->getState(j), path->at(j));
        }
      }
    }

    // goal stateをvariablesに反映して返す.
    if(problemDefinition->hasSolution()) {
      const ompl::geometric::PathGeometricPtr solutionPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(problemDefinition->getSolutionPath());
      state2Link(stateSpace, solutionPath->getState(solutionPath->getStateCount()-1), variables[0]);
      std::set<cnoid::BodyPtr> bodies = getBodies(variables[0]);
      for(std::set<cnoid::BodyPtr>::const_iterator it=bodies.begin(); it != bodies.end(); it++){
        (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
        (*it)->calcCenterOfMass();
      }
    }

    return solved == ompl::base::PlannerStatus::EXACT_SOLUTION;
  }

  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, // 0: constriant priority 1: constraints
                const std::vector<double>& goal, // 0: angles.
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){ // 0: states. 1: angles

    std::vector<std::vector<cnoid::LinkPtr> > variabless{variables};
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > constraintss{constraints};
    std::shared_ptr<UintQueue> modelQueue = std::make_shared<UintQueue>();
    std::vector<std::map<cnoid::BodyPtr, cnoid::BodyPtr> > modelMaps(1);
    GIKParam param2(param);
    modelQueue->push(0);

    if(param.threads >= 2){
      std::set<cnoid::BodyPtr> bodies = getBodies(variables);
      for(int i=1;i<param.threads;i++){
        modelQueue->push(i);
        std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
        {
          /*
             cnoid::Body::clone()は、各リンクのメッシュは複製せず、ポインタで共有する.
             この仕様は高速な複製のために有益である.
             一方で、同一のメッシュオブジェクトを共有するBody間で、
               - Body::clone()
               - Body::~Body()
             などがスレッドセーフでないことに注意が必要である.
             solveGIK関数をマルチスレッドから呼ぶ場合に問題になる場合がある.
          */
          std::shared_ptr<std::lock_guard<std::mutex> > guard;
          if(param.modelMutex) guard = std::make_shared<std::lock_guard<std::mutex> >(*(param.modelMutex));
          for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
            modelMap[*it] = (*it)->clone(); // cloneしたbodyがデストラクトされないように、保管しておく
          }
        }
        modelMaps.push_back(modelMap); // cloneしたbodyがデストラクトされないように、保管しておく
        variabless.push_back(std::vector<cnoid::LinkPtr>(variables.size()));
        for(int v=0;v<variables.size();v++){
          variabless.back()[v] = modelMap[variables[v]->body()]->link(variables[v]->index());
        }
        constraintss.push_back(std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >(constraints.size()));
        for(int j=0;j<constraints.size();j++){
          constraintss.back()[j].resize(constraints[j].size());
          for(int k=0;k<constraints[j].size();k++){
            constraintss.back()[j][k] = constraints[j][k]->clone(modelMap);
          }
        }
        if(param.projectLink.size() == 1){
          param2.projectLink.push_back(modelMap[param.projectLink[0]->body()]->link(param.projectLink[0]->index()));
        }
      }
    }

    bool result = solveGIK(variabless,
                           constraintss,
                           goal,
                           modelQueue,
                           param2,
                           path);
    {
      std::shared_ptr<std::lock_guard<std::mutex> > guard;
      if(param.modelMutex) guard = std::make_shared<std::lock_guard<std::mutex> >(*(param.modelMutex));
      variabless.clear();
      constraintss.clear();
      modelMaps.clear();
      param2.projectLink.clear();
    }

    return result;
  }

  bool solveGIK(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, // 0: modelQueue, 1: constriant priority 2: constraints
                const std::vector<double>& goal, // 0: angles.
                std::shared_ptr<UintQueue> modelQueue,
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){ // 0: states. 1: angles
    if((variables.size() == 0) ||
       (variables.size() != constraints.size())){
      std::cerr << "[solveGIK] size mismatch" << std::endl;
      return false;
    }

    bool calculate_path = true;
    if(path == nullptr) {
      calculate_path = false;
    }

    ompl::base::StateSpacePtr ambientSpace = createAmbientSpace(variables[0], param.maxTranslation);
    GIKConstraintPtr gikConstraint = std::make_shared<GIKConstraint>(modelQueue, constraints, variables);
    gikConstraint->viewer() = param.viewer;
    gikConstraint->drawLoop() = param.drawLoop;
    gikConstraint->param() = param.pikParam;
    gikConstraint->param().satisfiedConvergeLevel = -1;
    gikConstraint->nearMaxError() = param.nearMaxError;
    GIKStateSpacePtr stateSpace = std::make_shared<GIKStateSpace>(ambientSpace, gikConstraint);
    stateSpace->setDelta(param.delta); // この距離内のstateは、中間のconstraintチェック無しで遷移可能
    ompl_near_projection::NearConstrainedSpaceInformationPtr spaceInformation = std::make_shared<ompl_near_projection::NearConstrainedSpaceInformation>(stateSpace);
    spaceInformation->setStateValidityChecker(std::make_shared<ompl::base::AllValidStateValidityChecker>(spaceInformation)); // validは全てconstraintでチェックするので、StateValidityCheckerは全てvalidでよい
    spaceInformation->setup(); // ここでsetupを呼ばないと、stateSpaceがsetupされないのでlink2State等ができない

    ompl::base::ProblemDefinitionPtr problemDefinition = std::make_shared<ompl::base::ProblemDefinition>(spaceInformation);

    ompl::base::ScopedState<> start(stateSpace);
    link2State(variables[0], stateSpace, start.get());
    stateSpace->enforceBounds(start.get()); // 僅かにjoint limitを超えていた場合、エラーになってしまうので
    problemDefinition->clearStartStates();
    problemDefinition->addStartState(start);

    ompl::base::ScopedState<> goalState(stateSpace);
    frame2State(goal, stateSpace, goalState.get());
    problemDefinition->setGoalState(goalState);

    ompl::base::PlannerPtr planner;

    if(param.threads <= 1){
      std::shared_ptr<ompl_near_projection::geometric::NearRRTConnect> planner_ = std::make_shared<ompl_near_projection::geometric::NearRRTConnect>(spaceInformation, false);
      planner_->setRange(param.range);
      planner = planner_;
    }else{
      std::shared_ptr<ompl_near_projection::geometric::pNearRRTConnect> planner_ = std::make_shared<ompl_near_projection::geometric::pNearRRTConnect>(spaceInformation, false);
      planner_->setRange(param.range);
      planner_->setThreadCount(param.threads);
      planner = planner_;
    }

    if(!spaceInformation->isSetup()) spaceInformation->setup();
    planner->setProblemDefinition(problemDefinition);
    if(!planner->isSetup()) planner->setup();

    // たまに、サンプル時のIKと補間時のIKが微妙に違うことが原因で、解のpathの補間に失敗する場合があるので、成功するまでとき直す. path.check()と、interpolateやsimplifyのIKは同じものなので、最初のpath.check()が成功すれば、simplifyやinterpolate後のpathも必ずcheck()が成功すると想定.
    ompl::base::PlannerStatus solved;

    //planner->clear();
    //problemDefition->clearSolutionPaths();

    {
      ompl::time::point start = ompl::time::now();
      solved = planner->solve(ompl::base::plannerOrTerminationCondition(ompl::base::timedPlannerTerminationCondition(param.timeout),
                                                                        param.ptc));
      double planTime = ompl::time::seconds(ompl::time::now() - start);
      if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
        OMPL_INFORM("Solution found in %f seconds", planTime);
      else
        OMPL_INFORM("No solution found after %f seconds", planTime);
    }

    // IKの途中経過を使っているので、EXACT_SOLUTIONなら、必ずinterpolateできている.
    // 一時的にstateが最適化の誤差で微妙に!isSatisfiedになっていたり、deltaを上回った距離になっていることがあって、それは許容したい.
    // solutionPath.check()は行わない

    if(calculate_path){
      ompl::geometric::PathGeometricPtr solutionPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(problemDefinition->getSolutionPath());
      if(solutionPath == nullptr) {
        path->resize(0);
      }else{
        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        // ompl::geometric::PathSimplifierPtr pathSimplifier = std::make_shared<ompl::geometric::PathSimplifier>(spaceInformation);
        // {
        //   ompl::time::point start = ompl::time::now();
        //   std::size_t numStates = solutionPath->getStateCount();
        //   gikConstraint->viewer() = nullptr; // simplifySolution()中は描画しない
        //   pathSimplifier->simplify(*solutionPath, param.timeout);
        //   gikConstraint->viewer() = param.viewer; // simplifySolution()中は描画しない
        //   double simplifyTime = ompl::time::seconds(ompl::time::now() - start);
        //   OMPL_INFORM("Path simplification took %f seconds and changed from %d to %d states",
        //               simplifyTime, numStates, solutionPath->getStateCount());
        // }
        // if(param.debugLevel > 1){
        //   solutionPath->print(std::cout);
        // }

        solutionPath->interpolate();
        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        // 途中の軌道をpathに入れて返す
        path->resize(solutionPath->getStateCount());
        for(int j=0;j<solutionPath->getStateCount();j++){
          //stateSpace->getDimension()は,SO3StateSpaceが3を返してしまう(実際はquaternionで4)ので、使えない
          state2Frame(stateSpace, solutionPath->getState(j), path->at(j));
        }
      }
    }

    // goal stateをvariablesに反映して返す.
    if(problemDefinition->hasSolution()) {
      const ompl::geometric::PathGeometricPtr solutionPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(problemDefinition->getSolutionPath());
      state2Link(stateSpace, solutionPath->getState(solutionPath->getStateCount()-1), variables[0]);
      std::set<cnoid::BodyPtr> bodies = getBodies(variables[0]);
      for(std::set<cnoid::BodyPtr>::const_iterator it=bodies.begin(); it != bodies.end(); it++){
        (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
        (*it)->calcCenterOfMass();
      }
    }

    return solved == ompl::base::PlannerStatus::EXACT_SOLUTION;
  }

  bool shortCut(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                std::shared_ptr<std::vector<std::vector<double> > >& path,
                const GIKParam& param) // 0: states. 1: angles
  {
    for(int i=1;i+1<path->size();){
      bool cut = true;
      int idx = 0;
      for(int l=0;l<variables.size();l++){
        if(variables[l]->isRevoluteJoint() || variables[l]->isPrismaticJoint()) {
          if (std::abs((*path)[i-1][idx] - (*path)[i+1][idx]) > param.shortcutThre) {
            cut = false;
            break;
          }
          idx++;
        } else if(variables[l]->isFreeJoint()) {
          cnoid::Quaternion prevQ((*path)[i-1][idx+6], (*path)[i-1][idx+3], (*path)[i-1][idx+4], (*path)[i-1][idx+5]);
          cnoid::Quaternion nextQ((*path)[i+1][idx+6], (*path)[i+1][idx+3], (*path)[i+1][idx+4], (*path)[i+1][idx+5]);
          cnoid::Matrix3 prevR = prevQ.toRotationMatrix();
          cnoid::Matrix3 nextR = nextQ.toRotationMatrix();
          cnoid::AngleAxis diffAngleAxis = cnoid::AngleAxis(nextR * prevR.transpose());
          if ((std::abs((*path)[i-1][idx+0] - (*path)[i+1][idx+0]) > param.shortcutThre) ||
              (std::abs((*path)[i-1][idx+1] - (*path)[i+1][idx+1]) > param.shortcutThre) ||
              (std::abs((*path)[i-1][idx+2] - (*path)[i+1][idx+2]) > param.shortcutThre) ||
              (std::abs(diffAngleAxis.angle()) > param.shortcutThre)) {
            cut = false;
            break;
          }
          idx+=7;
        }
      }
      if (cut) {
        path->erase(path->begin() + i);
      } else {
        i++;
      }
    }
    return true;
  }

  bool postProcess(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                   const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, // 0: constriant priority 1: constraints
                   const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& goals, // 0: constraints
                   const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals, // 0: constraints
                   std::shared_ptr<std::vector<std::vector<double> > >& path,
                   const GIKParam& param) // 0: states. 1: angles
  {
    // goalsが空 -> 先頭要素と末尾要素は固定する.
    // goalsが空でない -> 先頭要素のみ固定する.

    if(param.debugLevel >=2){
      std::cerr << "start post process. path size: " << path->size() << std::endl;
    }
    if(path->size() <= 2) return true;

    // shortcut.
    global_inverse_kinematics_solver::shortCut(variables,
                                               path,
                                               param);
    if(param.debugLevel >=2){
      std::cerr << "after shortcut. path size: " << path->size() << std::endl;
    }
    if(path->size() <= 2) return true;


    // optimize
    std::vector<std::map<cnoid::BodyPtr, cnoid::BodyPtr> > modelMaps;
    std::vector<std::vector<cnoid::LinkPtr> > variabless; // path[0]からpath[-1]まで含まれる.
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > constraintss;
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > interConstraintss; // path[i]とpath[i+1]間の制約がi番目に入る
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goalss;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > nominalss;
    std::set<cnoid::BodyPtr> bodies = getBodies(variables);

    for (int i=0; i< path->size(); i++){
      std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
      {
        /*
          cnoid::Body::clone()は、各リンクのメッシュは複製せず、ポインタで共有する.
          この仕様は高速な複製のために有益である.
          一方で、同一のメッシュオブジェクトを共有するBody間で、
          - Body::clone()
          - Body::~Body()
          などがスレッドセーフでないことに注意が必要である.
          solveGIK関数をマルチスレッドから呼ぶ場合に問題になる場合がある.
        */
        std::shared_ptr<std::lock_guard<std::mutex> > guard;
        if(param.modelMutex) guard = std::make_shared<std::lock_guard<std::mutex> >(*(param.modelMutex));
        for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
          modelMap[*it] = (*it)->clone();
        }
      }
      modelMaps.push_back(modelMap); // cloneしたbodyがデストラクトされないように、保管しておく

      std::vector<cnoid::LinkPtr> variablesNext;
      for(int v=0;v<variables.size();v++){
        variablesNext.push_back(modelMap[variables[v]->body()]->link(variables[v]->index()));
      }
      variabless.push_back(variablesNext);
      if(i==0){  // start stateは固定なので含めない.
        std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraintsNext(constraints.size());
        if(constraintsNext.size() == 0) constraintsNext.resize(1);
        constraintss.push_back(constraintsNext);
        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominalsNext;
        nominalss.push_back(nominalsNext);
      }else{
        std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraintsNext(constraints.size());
        if(constraintsNext.size() == 0) constraintsNext.resize(1);
        for(int j=0;j<constraints.size();j++){
          for(int k=0;k<constraints[j].size();k++){
            constraintsNext[j].push_back(constraints[j][k]->clone(modelMap));
          }
        }
        constraintss.push_back(constraintsNext);
        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominalsNext;
        for(int k=0;k<nominals.size();k++){
          nominalsNext.push_back(nominals[k]->clone(modelMap));
        }
        nominalss.push_back(nominalsNext);
      }
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > interConstraintsNext(constraintss[i].size());
      interConstraintss.push_back(interConstraintsNext);
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goalsNext;
      goalss.push_back(goalsNext);
    }

    // displacement limit
    for (int i=1; i<path->size(); i++) { // start stateは固定なので含めない.
      for(int v=0;v<variables.size();v++) {
        if(variables[v]->isRevoluteJoint() || variables[v]->isPrismaticJoint() || variables[v]->isFreeJoint()){
          std::shared_ptr<ik_constraint2::JointDisplacementConstraint> constraint = std::make_shared<ik_constraint2::JointDisplacementConstraint>();
          constraint->joint() = variabless[i][v];
          constraint->limit() = param.displacementThre;
          constraintss[i].back().push_back(constraint);
        }
      }
    }
    // adjacent configuration
    for (int i=0; i+1<path->size(); i++) {
      for(int v=0;v<variables.size();v++) {
        if(variables[v]->isRevoluteJoint() || variables[v]->isPrismaticJoint()){
          std::shared_ptr<ik_constraint2::JointRegionConstraint> constraint = std::make_shared<ik_constraint2::JointRegionConstraint>();
          constraint->A_joint() = variabless[i][v];
          constraint->B_joint() = variabless[i+1][v];
          constraint->region() = param.shortcutThre;
          interConstraintss[i].back().push_back(constraint);
        }else if(variables[v]->isFreeJoint()) {
          std::shared_ptr<ik_constraint2::RegionConstraint2> constraint = std::make_shared<ik_constraint2::RegionConstraint2>();
          constraint->A_link() = variabless[i][v];
          constraint->B_link() = variabless[i+1][v];
          constraint->C().resize(6,6);
          constraint->dl().resize(6);
          constraint->du().resize(6);
          for(int i=0;i<6;i++){
            constraint->C().insert(i,i) = 1.0;
            constraint->dl()[i] = - param.shortcutThre;
            constraint->du()[i] = param.shortcutThre;
          }
          interConstraintss[i].back().push_back(constraint);
        }else{
          std::cerr << __FUNCTION__ << " something is wrong" << std::endl;
        }
      }
    }

    for (int i=0; i<path->size(); i++) {
      constraintss[i].resize(constraintss[i].size()+1);
      interConstraintss[i].resize(interConstraintss[i].size()+1);
    }

    // adjacent configuration
    for (int i=0; i+1<path->size(); i++) {
      for(int v=0;v<variables.size();v++) {
        if(variables[v]->isRevoluteJoint() || variables[v]->isPrismaticJoint()){
          std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
          constraint->A_joint() = variabless[i][v];
          constraint->B_joint() = variabless[i+1][v];
          constraint->precision() = 1e-3; // never satisfied
          constraint->maxError() = 1e10;
          interConstraintss[i].back().push_back(constraint);
        }else if(variables[v]->isFreeJoint()) {
          std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
          constraint->A_link() = variabless[i][v];
          constraint->B_link() = variabless[i+1][v];
          constraint->precision() = 1e-3; // never satisfied
          constraint->maxError() << 1e10, 1e10, 1e10, 1e10, 1e10, 1e10;
          interConstraintss[i+1].back().push_back(constraint);
        }else{
          std::cerr << __FUNCTION__ << " something is wrong" << std::endl;
        }
      }
    }
    for (int i=0; i<path->size(); i++) {
      constraintss[i].back().insert(constraintss[i].back().end(),nominalss[i].begin(),nominalss[i].end());
    }

    int prevSize = path->size() + 1;
    while(path->size() > 2 && path->size() != prevSize){
      prevSize = path->size();

      for (int i=0; i<path->size(); i++) {
        global_inverse_kinematics_solver::frame2Link((*path)[i],variabless[i]);
      }

      if(goalss[int(path->size())-1].size() != goals.size()){
        goalss[int(path->size())-1].clear();
        for(int k=0;k<goals.size();k++){
          goalss[int(path->size())-1].push_back(goals[k]->clone(modelMaps[int(path->size())-1]));
        }
      }

      std::vector<cnoid::LinkPtr> variablesAll; // path[1]からpath[-1]またはpath[-2]まで含まれる.
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraintsAll(constraintss[0].size());

      if(goals.size() ==0){
        for (int i=1; i+1<path->size(); i++) { // goalは固定
          variablesAll.insert(variablesAll.end(), variabless[i].begin(), variabless[i].end());
          for(int j=0;j<constraintss[i].size();j++){
            constraintsAll[j].insert(constraintsAll[j].end(), constraintss[i][j].begin(), constraintss[i][j].end());
          }
          for(int j=0;j<interConstraintss[i].size();j++){
            constraintsAll[j].insert(constraintsAll[j].end(), interConstraintss[i][j].begin(), interConstraintss[i][j].end());
          }
        }
      }else{
        for (int i=1; i<path->size(); i++) { // goalは可変
          variablesAll.insert(variablesAll.end(), variabless[i].begin(), variabless[i].end());
          for(int j=0;j<constraintss[i].size();j++){
            constraintsAll[j].insert(constraintsAll[j].end(), constraintss[i][j].begin(), constraintss[i][j].end());
          }
          if(i+1<path->size()){
            for(int j=0;j<interConstraintss[i].size();j++){
              constraintsAll[j].insert(constraintsAll[j].end(), interConstraintss[i][j].begin(), interConstraintss[i][j].end());
            }
          }else{
            constraintsAll[int(constraintsAll.size())-2].insert(constraintsAll[int(constraintsAll.size())-2].end(), goalss[i].begin(), goalss[i].end());
            //constraintsAll[0].insert(constraintsAll[0].end(), goalss[i].begin(), goalss[i].end()); // 最高優先度. 末尾より上であればいずれの優先度でも良いはずだが, より精度良く達成するため.
          }
        }
      }

      std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
      prioritized_inverse_kinematics_solver2::IKParam pikParam = param.postpikParam;
      pikParam.wmaxVec.clear();
      pikParam.wmaxVec.resize(constraintsAll.size(), param.postpikParam_wmaxVec1);
      pikParam.wmaxVec.back() = param.postpikParam_wmaxVec2;
      pikParam.convergeThre = param.postpikParam_convergeThre * std::sqrt(path->size());
      pikParam.satisfiedConvergeLevel = -1;
      bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variablesAll,
                                                                        constraintsAll,
                                                                        tasks,
                                                                        pikParam);
      if(goals.size() ==0){
        for(int i=1;i+1<path->size();i++){ // goalは固定
          global_inverse_kinematics_solver::link2Frame(variabless[i], (*path)[i]); // 更新
        }
      }else{
        for(int i=1;i<path->size();i++){ // goalは可変
          global_inverse_kinematics_solver::link2Frame(variabless[i], (*path)[i]); // 更新
        }
      }

      // shortcut.
      global_inverse_kinematics_solver::shortCut(variables,
                                                 path,
                                                 param);
      if(param.debugLevel >=2){
        std::cerr << "after optimization. path size: " << path->size() << std::endl;
      }
    }

    {
      std::shared_ptr<std::lock_guard<std::mutex> > guard;
      if(param.modelMutex) guard = std::make_shared<std::lock_guard<std::mutex> >(*(param.modelMutex));
      modelMaps.clear();
      variabless.clear();
      constraintss.clear();
      interConstraintss.clear();
      goalss.clear();
      nominalss.clear();
    }

    return true;
  }

}
