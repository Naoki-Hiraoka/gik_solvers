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
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
    return solveGIK(variables, constraints, goals, nominals, rejections, param, path);
  }

  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& goals,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections, // 0: rejections
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){
    return solveGIK(variables, constraints, std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >{goals}, nominals, rejections, param, std::vector<std::shared_ptr<std::vector<std::vector<double> > > >{path});
  }

  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
    return solveGIK(variables, constraints, goals, nominals, rejections, param, path);
  }

  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections, // 0: rejections
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){
    std::vector<std::shared_ptr<std::vector<std::vector<double> > > > paths;
    for(int i=0;i<goals.size();i++) paths.push_back(std::make_shared<std::vector<std::vector<double> > >());
    bool ret = solveGIK(variables, constraints, std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >{goals}, nominals, rejections, param, paths, false);
    for(int i=0;i<goals.size();i++){
      if(paths[i]->size()>0){
        *path = *(paths[i]);
        break;
      }
    }
    return ret;
  }

  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                const GIKParam& param,
                const std::vector<std::shared_ptr<std::vector<std::vector<double> > > >& path,
                bool findAllSolution){
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
    return solveGIK(variables, constraints, goals, nominals, rejections, param, path, findAllSolution);
  }

  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections, // 0: rejections
                const GIKParam& param,
                const std::vector<std::shared_ptr<std::vector<std::vector<double> > > >& path,
                bool findAllSolution){
    std::vector<std::vector<cnoid::LinkPtr> > variabless{variables};
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > constraintss{constraints};
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > goalss{goals};
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > nominalss{nominals};
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > rejectionss{rejections};
    std::shared_ptr<UintQueue> modelQueue = std::make_shared<UintQueue>();
    std::vector<std::map<cnoid::BodyPtr, cnoid::BodyPtr> > modelMaps;
    GIKParam param2(param);
    modelQueue->push(0);

    if(param.threads >= 2){
      std::set<cnoid::BodyPtr> bodies = getBodies(variables);
      std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
      for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
        modelMap[*it] = (*it)->clone();
      }
      modelMaps.push_back(modelMap); // cloneしたbodyがデストラクトされないように、保管しておく
      for(int i=1;i<param.threads;i++){
        modelQueue->push(i);
        std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
        for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
          modelMap[*it] = (*it)->clone();
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
        rejectionss.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(rejections.size()));
        for(int j=0;j<rejections.size();j++){
          rejectionss.back()[j] = rejections[j]->clone(modelMap);
        }
        if(param.projectLink.size() == 1){
          param2.projectLink.push_back(modelMap[param.projectLink[0]->body()]->link(param.projectLink[0]->index()));
        }
      }
    }

    return solveGIK(variabless,
                    constraintss,
                    goalss,
                    nominalss,
                    rejectionss,
                    modelQueue,
                    param2,
                    path,
                    findAllSolution);
  }

  bool solveGIK(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, // 0: modelQueue, 1: constriant priority 2: constraints
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& goals, // 0: modelQueue. 1: goalSpace(OR). 2: goals(AND).
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& nominals, // 0: modelQueue, 1: nominals
                std::shared_ptr<UintQueue> modelQueue,
                const GIKParam& param,
                const std::vector<std::shared_ptr<std::vector<std::vector<double> > > >& path,
                bool findAllSolution){
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > rejections(variables.size());
    return solveGIK(variables, constraints, goals, nominals, rejections, modelQueue, param, path, findAllSolution);
  }


  bool solveGIK(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, // 0: modelQueue, 1: constriant priority 2: constraints
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& goals, // 0: modelQueue. 1: goalSpace(OR). 2: goals(AND).
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& nominals, // 0: modelQueue, 1: nominals
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& rejections, // 0: modelQueue, 1: rejections
                std::shared_ptr<UintQueue> modelQueue,
                const GIKParam& param,
                const std::vector<std::shared_ptr<std::vector<std::vector<double> > > >& path,
                bool findAllSolution){
    if((variables.size() == 0) ||
       (variables.size() != constraints.size()) ||
       (constraints.size() != goals.size()) ||
       (goals.size() != nominals.size()) ||
       (nominals.size() != rejections.size())
       ){
      std::cerr << "[solveGIK] size mismatch" << std::endl;
      return false;
    }

    bool calculate_path = true;
    if(goals.size() == 0 ||
       goals[0].size() != path.size()) calculate_path=false;
    for(int i=0;i<path.size();i++) {
      if(path[i] == nullptr) {
        calculate_path = false;
        break;
      }
    }

    ompl::base::StateSpacePtr ambientSpace = createAmbientSpace(variables[0], param.maxTranslation);
    GIKConstraintPtr gikConstraint = std::make_shared<GIKConstraint>(ambientSpace, modelQueue, constraints, variables, rejections);
    if(param.useProjection){
      GIKConstraint2Ptr gikConstraint_ = std::make_shared<GIKConstraint2>(ambientSpace, modelQueue, constraints, variables, rejections);
      gikConstraint_->projectionRange = param.projectionRange;
      gikConstraint_->projectionTrapThre = param.projectionTrapThre;
      gikConstraint = gikConstraint_;
    }
    gikConstraint->viewer() = param.viewer;
    gikConstraint->drawLoop() = param.drawLoop;
    gikConstraint->param() = param.pikParam;
    gikConstraint->nearMaxError() = param.nearMaxError;
    GIKStateSpacePtr stateSpace = std::make_shared<GIKStateSpace>(ambientSpace, gikConstraint);
    stateSpace->setDelta(param.delta); // この距離内のstateは、中間のconstraintチェック無しで遷移可能
    ompl_near_projection::NearConstrainedSpaceInformationPtr spaceInformation = std::make_shared<ompl_near_projection::NearConstrainedSpaceInformation>(stateSpace);
    spaceInformation->setStateValidityChecker(std::make_shared<ompl::base::AllValidStateValidityChecker>(spaceInformation)); // validは全てconstraintでチェックするので、StateValidityCheckerは全てvalidでよい
    spaceInformation->setup(); // ここでsetupを呼ばないと、stateSpaceがsetupされないのでlink2State等ができない

    ompl_near_projection::NearProblemDefinitionPtr problemDefinition = std::make_shared<ompl_near_projection::NearProblemDefinition>(spaceInformation);
    problemDefinition->setFindAllGoals(findAllSolution);

    ompl::base::ScopedState<> start(stateSpace);
    link2State(variables[0], stateSpace, start.get());

    stateSpace->enforceBounds(start.get()); // 僅かにjoint limitを超えていた場合、エラーになってしまうので
    problemDefinition->clearStartStates();
    problemDefinition->addStartState(start);

    std::vector<ompl_near_projection::NearGoalSpacePtr> goalSpaces;
    for(int i=0;i<goals[0].size();i++){
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goal_(goals.size()); // 0: modelQueue, q: goals(AND)
      for(int m=0;m<goals.size();m++) goal_[m] = goals[m][i];
      GIKGoalSpacePtr goal = std::make_shared<GIKGoalSpace>(spaceInformation, ambientSpace, modelQueue, constraints, variables, goal_, nominals, rejections);
      goal->setViewer(param.viewer);
      goal->setDrawLoop(param.drawLoop);
      goal->setParam(param.pikParam);
      goal->setNearMaxError(param.nearMaxError);
      goalSpaces.push_back(goal);
    }

    problemDefinition->setGoals(goalSpaces);

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
      solved = planner->solve(param.timeout);
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
      int path_num = 0;
      for(int i=0;i<path.size();i++){
        ompl::geometric::PathGeometricPtr solutionPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(problemDefinition->getSolutionPathForAGoal(i));
        if(solutionPath == nullptr) {
          path[i]->resize(0);
          continue;
        }
        path_num++;

        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        ompl::geometric::PathSimplifierPtr pathSimplifier = std::make_shared<ompl::geometric::PathSimplifier>(spaceInformation);
        ompl::time::point start = ompl::time::now();
        std::size_t numStates = solutionPath->getStateCount();
        gikConstraint->viewer() = nullptr; // simplifySolution()中は描画しない
        //pathSimplifier->simplify(*solutionPath, param.timeout);
        gikConstraint->viewer() = param.viewer; // simplifySolution()中は描画しない
        double simplifyTime = ompl::time::seconds(ompl::time::now() - start);
        OMPL_INFORM("Path simplification took %f seconds and changed from %d to %d states",
                    simplifyTime, numStates, solutionPath->getStateCount());

        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        solutionPath->interpolate();
        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        // 途中の軌道をpathに入れて返す
        path[i]->resize(solutionPath->getStateCount());
        for(int j=0;j<solutionPath->getStateCount();j++){
          //stateSpace->getDimension()は,SO3StateSpaceが3を返してしまう(実際はquaternionで4)ので、使えない
          state2Frame(stateSpace, solutionPath->getState(j), path[i]->at(j));
        }
      }
      OMPL_INFORM("Path found %d / %d goals",
                  path_num, path.size());
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
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
    return solveGIK(variables, constraints, rejections, goal, param, path);
  }

  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, // 0: constriant priority 1: constraints
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections, // 0: constriant priority 1: rejections
                const std::vector<double>& goal, // 0: angles.
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){ // 0: states. 1: angles
    std::vector<std::vector<cnoid::LinkPtr> > variabless{variables};
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > constraintss{constraints};
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > rejectionss{rejections};
    std::shared_ptr<UintQueue> modelQueue = std::make_shared<UintQueue>();
    std::vector<std::map<cnoid::BodyPtr, cnoid::BodyPtr> > modelMaps;
    GIKParam param2(param);
    modelQueue->push(0);

    if(param.threads >= 2){
      std::set<cnoid::BodyPtr> bodies = getBodies(variables);
      for(int i=1;i<param.threads;i++){
        modelQueue->push(i);
        std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
        for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
          modelMap[*it] = (*it)->clone();
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
        rejectionss.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(rejections.size()));
        for(int j=0;j<rejections.size();j++){
          rejectionss.back()[j] = rejections[j]->clone(modelMap);
        }
        if(param.projectLink.size() == 1){
          param2.projectLink.push_back(modelMap[param.projectLink[0]->body()]->link(param.projectLink[0]->index()));
        }
      }
    }

    return solveGIK(variabless,
                    constraintss,
                    rejectionss,
                    goal,
                    modelQueue,
                    param2,
                    path);
  }

  bool solveGIK(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, // 0: modelQueue, 1: constriant priority 2: constraints
                const std::vector<double>& goal, // 0: angles.
                std::shared_ptr<UintQueue> modelQueue,
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){ // 0: states. 1: angles
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > rejections(variables.size());
    return solveGIK(variables, constraints, rejections, goal, modelQueue, param, path);
  }

  bool solveGIK(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, // 0: modelQueue, 1: constriant priority 2: constraints
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& rejections, // 0: modelQueue, 1: rejections
                const std::vector<double>& goal, // 0: angles.
                std::shared_ptr<UintQueue> modelQueue,
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){ // 0: states. 1: angles
    if((variables.size() == 0) ||
       (variables.size() != constraints.size()) ||
       (variables.size() != rejections.size())){
      std::cerr << "[solveGIK] size mismatch" << std::endl;
      return false;
    }

    bool calculate_path = true;
    if(path == nullptr) {
      calculate_path = false;
    }

    ompl::base::StateSpacePtr ambientSpace = createAmbientSpace(variables[0], param.maxTranslation);
    GIKConstraintPtr gikConstraint = std::make_shared<GIKConstraint>(ambientSpace, modelQueue, constraints, variables, rejections);
    if(param.useProjection){
      GIKConstraint2Ptr gikConstraint_ = std::make_shared<GIKConstraint2>(ambientSpace, modelQueue, constraints, variables, rejections);
      gikConstraint_->projectionRange = param.projectionRange;
      gikConstraint_->projectionTrapThre = param.projectionTrapThre;
      gikConstraint = gikConstraint_;
    }
    gikConstraint->viewer() = param.viewer;
    gikConstraint->drawLoop() = param.drawLoop;
    gikConstraint->param() = param.pikParam;
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
      solved = planner->solve(param.timeout);
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

        ompl::geometric::PathSimplifierPtr pathSimplifier = std::make_shared<ompl::geometric::PathSimplifier>(spaceInformation);
        {
          ompl::time::point start = ompl::time::now();
          std::size_t numStates = solutionPath->getStateCount();
          gikConstraint->viewer() = nullptr; // simplifySolution()中は描画しない
          //pathSimplifier->simplify(*solutionPath, param.timeout);
          gikConstraint->viewer() = param.viewer; // simplifySolution()中は描画しない
          double simplifyTime = ompl::time::seconds(ompl::time::now() - start);
          OMPL_INFORM("Path simplification took %f seconds and changed from %d to %d states",
                      simplifyTime, numStates, solutionPath->getStateCount());
        }

        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

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
                   const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals, // 0: constraints
                   std::shared_ptr<std::vector<std::vector<double> > >& path,
                   const GIKParam& param) // 0: states. 1: angles
  {
    // 先頭要素と末尾要素は固定する.

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
    {
      std::vector<std::map<cnoid::BodyPtr, cnoid::BodyPtr> > modelMaps;
      std::vector<cnoid::LinkPtr> variablesAll;
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraintsAll(constraints.size());
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominalsAll;
      std::set<cnoid::BodyPtr> bodies = getBodies(variables);
      for (int i=0; i< path->size(); i++){
        std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
        for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
          modelMap[*it] = (*it)->clone();
        }
        modelMaps.push_back(modelMap); // cloneしたbodyがデストラクトされないように、保管しておく

        std::vector<cnoid::LinkPtr> variablesNext;
        for(int v=0;v<variables.size();v++){
          variablesNext.push_back(modelMap[variables[v]->body()]->link(variables[v]->index()));
        }
        global_inverse_kinematics_solver::frame2Link((*path)[i],variablesNext);
        variablesAll.insert(variablesAll.end(), variablesNext.begin(), variablesNext.end());
        for(int j=0;j<constraints.size();j++){
          for(int k=0;k<constraints[j].size();k++){
            constraintsAll[j].push_back(constraints[j][k]->clone(modelMap));
          }
        }
        for(int k=0;k<nominals.size();k++){
          nominalsAll.push_back(nominals[k]->clone(modelMap));
        }
      }

      {
        // start configuration
        for(int v=0;v<variables.size();v++) {
          if(variables[v]->isRevoluteJoint() || variables[v]->isPrismaticJoint()){
            std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
            constraint->A_joint() = variablesAll[v];
            constraint->B_q() = variablesAll[v]->q();
            constraint->precision() = 1e-3;
            constraint->maxError() = 1e10;
            constraintsAll.back().push_back(constraint);
          }else if(variables[v]->isFreeJoint()) {
            std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
            constraint->A_link() = variablesAll[v];
            constraint->B_localpos() = variablesAll[v]->T();
            constraint->precision() = 1e-3;
            constraint->maxError() << 1e10, 1e10, 1e10, 1e10, 1e10, 1e10;
            constraintsAll.back().push_back(constraint);
          }else{
            std::cerr << __FUNCTION__ << " something is wrong" << std::endl;
          }
        }
        // goal configuration
        // for(int k=0;k<goals.size();k++){
        //   constraintsAll.back().push_back(goals[k]->clone(modelMaps.back()));
        // }
        for(int v=0;v<variables.size();v++) {
          if(variables[v]->isRevoluteJoint() || variables[v]->isPrismaticJoint()){
            std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
            constraint->A_joint() = variablesAll[v + variables.size() * (path->size()-1)];
            constraint->B_q() = variablesAll[v + variables.size() * (path->size()-1)]->q();
            constraint->precision() = 1e-3;
            constraint->maxError() = 1e10;
            constraintsAll.back().push_back(constraint);
          }else if(variables[v]->isFreeJoint()) {
            std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
            constraint->A_link() = variablesAll[v + variables.size() * (path->size()-1)];
            constraint->B_localpos() = variablesAll[v + variables.size() * (path->size()-1)]->T();
            constraint->precision() = 1e-3;
            constraint->maxError() << 1e10, 1e10, 1e10, 1e10, 1e10, 1e10;
            constraintsAll.back().push_back(constraint);
          }else{
            std::cerr << __FUNCTION__ << " something is wrong" << std::endl;
          }
        }
        // displacement limit
        for (int i=0; i<path->size(); i++) {
          for(int v=0;v<variables.size();v++) {
            if(variables[v]->isRevoluteJoint() || variables[v]->isPrismaticJoint() || variables[v]->isFreeJoint()){
              std::shared_ptr<ik_constraint2::JointDisplacementConstraint> constraint = std::make_shared<ik_constraint2::JointDisplacementConstraint>();
              constraint->joint() = variablesAll[v + i*variables.size()];
              constraint->limit() = param.displacementThre;
              constraintsAll.back().push_back(constraint);
            }
          }
        }
        // adjacent configuration
        constraintsAll.resize(constraintsAll.size()+1);
        for (int i=0; i+1<path->size(); i++) {
          for(int v=0;v<variables.size();v++) {
            if(variables[v]->isRevoluteJoint() || variables[v]->isPrismaticJoint()){
              std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
              constraint->A_joint() = variablesAll[v + i*variables.size()];
              constraint->B_joint() = variablesAll[v + (i+1)*variables.size()];
              constraint->precision() = 1e-3; // never satisfied
              constraint->maxError() = 1e10;
              constraintsAll.back().push_back(constraint);
            }else if(variables[v]->isFreeJoint()) {
              std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
              constraint->A_link() = variablesAll[v + i*variables.size()];
              constraint->B_link() = variablesAll[v + (i+1)*variables.size()];
              constraint->precision() = 1e-3; // never satisfied
              constraint->maxError() << 1e10, 1e10, 1e10, 1e10, 1e10, 1e10;
              constraintsAll.back().push_back(constraint);
            }else{
              std::cerr << __FUNCTION__ << " something is wrong" << std::endl;
            }
          }
        }
        constraintsAll.back().insert(constraintsAll.back().begin(),nominalsAll.begin(),nominalsAll.end());
      }
      std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
      prioritized_inverse_kinematics_solver2::IKParam pikParam = param.postpikParam;
      pikParam.wmaxVec.resize(constraintsAll.size(), param.postpikParam_wmaxVec1);
      pikParam.wmaxVec.back() = param.postpikParam_wmaxVec2;
      pikParam.convergeThre = param.postpikParam_convergeThre * std::sqrt(path->size());
      //pikParam.satisfiedConvergeLevel = int(constraints.size())-2;
      bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variablesAll,
                                                                        constraintsAll,
                                                                        tasks,
                                                                        pikParam);
      for(int i=0;i<path->size();i++){
        global_inverse_kinematics_solver::link2Frame(std::vector<cnoid::LinkPtr>(variablesAll.begin() + i * variables.size(), variablesAll.begin() + (i+1) * variables.size()), (*path)[i]); // 更新
      }
    }


    // shortcut.
    global_inverse_kinematics_solver::shortCut(variables,
                                               path,
                                               param);
    if(param.debugLevel >=2){
      std::cerr << "after optimization. path size: " << path->size() << std::endl;
    }

    return true;
  }

}
