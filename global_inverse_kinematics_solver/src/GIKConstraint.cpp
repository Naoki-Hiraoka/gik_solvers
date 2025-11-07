#include <global_inverse_kinematics_solver/GIKConstraint.h>

namespace global_inverse_kinematics_solver{
  bool GIKConstraint::project(ompl::base::State *state) const{
    std::cerr << "GIKConstraint::project" << std::endl;
    return projectNearValid(state, state);
  }

  bool GIKConstraint::projectNearValid(ompl::base::State *state, const ompl::base::State *near, double* distance) const{
    const unsigned int m = modelQueue_->pop();

    state2Link(stateSpace_, state, variables_[m]); // spaceとstateの空間をそろえる

    {
      // setup nearConstraints
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nearConstraints = ikConstraints_[m].back();
      nearConstraints.resize(variables_[m].size());
      for(int i=0;i<variables_[m].size();i++){
        if(variables_[m][i]->isRevoluteJoint() || variables_[m][i]->isPrismaticJoint()){
          std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::dynamic_pointer_cast<ik_constraint2::JointAngleConstraint>(nearConstraints[i]);
          if(constraint == nullptr) {
            nearConstraints[i] = constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
          }
          constraint->joint() = variables_[m][i];
          constraint->targetq() = variables_[m][i]->q();
          constraint->precision() = 1e10; // always satisfied
          constraint->maxError() = nearMaxError_;

        }else if(variables_[m][i]->isFreeJoint()) {
          std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::dynamic_pointer_cast<ik_constraint2::PositionConstraint>(nearConstraints[i]);
          if(constraint == nullptr) {
            nearConstraints[i] = constraint = std::make_shared<ik_constraint2::PositionConstraint>();
          }
          constraint->A_link() = variables_[m][i];
          constraint->B_localpos() = variables_[m][i]->T();
          constraint->precision() = 1e10; // always satisfied
          constraint->maxError() << nearMaxError_, nearMaxError_, nearMaxError_, nearMaxError_, nearMaxError_, nearMaxError_;
        }else{
          std::cerr << "[GIKConstraint::projectNear] something is wrong" << std::endl;
        }
      }
    }

    state2Link(stateSpace_, near, variables_[m]); // spaceとstateの空間をそろえる

    std::shared_ptr<std::vector<std::vector<double> > > path;
    ompl_near_projection::NearProjectedStateSpace::StateType* tmp_state = dynamic_cast<ompl_near_projection::NearProjectedStateSpace::StateType*>(state);
    if(tmp_state) path = std::make_shared<std::vector<std::vector<double> > >();
    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables_[m],
                                                                      ikConstraints_[m],
                                                                      tasks_[m],
                                                                      param_,
                                                                      path);

    link2State(variables_[m], stateSpace_, state); // spaceとstateの空間をそろえる

    if(tmp_state) {
      for(int i=0;i<tmp_state->intermediateStates.size();i++) for(int j=0;j<tmp_state->intermediateStates[i].size();j++) stateSpace_->freeState(tmp_state->intermediateStates[i][j]);
      tmp_state->intermediateStates.clear();
      tmp_state->intermediateStates.resize(1);
      for(int i=0;i+1<path->size();i++){ // 終点を含まない
        ompl::base::State* st = stateSpace_->allocState();
        frame2State(path->at(i), stateSpace_, st);
        tmp_state->intermediateStates[0].push_back(st);
      }
    }

    if(distance != nullptr){
      *distance = 0.0;
    }

    if(viewer_ != nullptr && m==0){
      loopCount_++;
      if(loopCount_%drawLoop_==0){
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints_[m].size();j++){
          for(int k=0;k<constraints_[m][j].size(); k++){constraints_[m][j][k]->updateBounds();
            const std::vector<cnoid::SgNodePtr>& marker = constraints_[m][j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        viewer_->drawOn(markers);
        viewer_->drawObjects(true);
      }
    }

    modelQueue_->push(m);

    return true;
  }

  bool GIKConstraint::projectGoalWithNominal(ompl::base::State *state, const ompl::base::State *near, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& nominals, double* distance) const{
    const unsigned int m = modelQueue_->pop();

    {
      // setup nearConstraints
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& goalConstraints = goalIkConstraints_[m][goalIkConstraints_[m].size()-2];
      goalConstraints.resize(goals[m].size());
      for(int i=0;i<goals[m].size(); i++){
        goalConstraints[i] = goals[m][i];
      }
    }

    {
      // setup nearConstraints
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominalConstraints = goalIkConstraints_[m][goalIkConstraints_[m].size()-1];
      nominalConstraints.resize(nominals[m].size());
      for(int i=0;i<nominals[m].size(); i++){
        nominalConstraints[i] = nominals[m][i];
      }
    }

    state2Link(stateSpace_, near, variables_[m]); // spaceとstateの空間をそろえる

    std::shared_ptr<std::vector<std::vector<double> > > path;
    ompl_near_projection::NearProjectedStateSpace::StateType* tmp_state = dynamic_cast<ompl_near_projection::NearProjectedStateSpace::StateType*>(state);
    if(tmp_state) path = std::make_shared<std::vector<std::vector<double> > >();
    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables_[m],
                                                                      goalIkConstraints_[m],
                                                                      tasks_[m],
                                                                      param_,
                                                                      path);

    bool satisfied = true;
    for(size_t j=0;j<goals[m].size()&&satisfied;j++){
      if(!goals[m][j]->isSatisfied()) {
        satisfied = false;
      }
    }

    link2State(variables_[m], stateSpace_, state); // spaceとstateの空間をそろえる

    if(tmp_state) {
      for(int i=0;i<tmp_state->intermediateStates.size();i++) for(int j=0;j<tmp_state->intermediateStates[i].size();j++) stateSpace_->freeState(tmp_state->intermediateStates[i][j]);
      tmp_state->intermediateStates.clear();
      tmp_state->intermediateStates.resize(1);
      for(int i=0;i+1<path->size();i++){ // 終点を含まない
        ompl::base::State* st = stateSpace_->allocState();
        frame2State(path->at(i), stateSpace_, st);
        tmp_state->intermediateStates[0].push_back(st);
      }
    }

    if(distance != nullptr){
      double squaredDistance = 0.0;
      for(size_t j=0;j<goals[m].size();j++){
        //constraints_[m][i][j]->updateBounds();
        squaredDistance += std::pow(goals[m][j]->distance(), 2.0);
      }
      *distance = std::sqrt(squaredDistance);
    }

    if(viewer_ != nullptr && m==0){
      loopCount_++;
      if(loopCount_%drawLoop_==0){
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints_[m].size();j++){
          for(int k=0;k<constraints_[m][j].size(); k++){
            const std::vector<cnoid::SgNodePtr>& marker = constraints_[m][j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        viewer_->drawOn(markers);
        viewer_->drawObjects(true);
      }
    }

    modelQueue_->push(m);

    return satisfied;
  }

  double GIKConstraint::distance (const ompl::base::State *state) const {
    return 0.0;
  }
  bool GIKConstraint::isSatisfied (const ompl::base::State *state) const {
    return true;
  }

  // 各要素ごとにfromからの変位がmaxDistance以下になる範囲内でtoに近づくstateを返す.
  void GIKConstraint::elementWiseDistanceLimit(const ompl::base::State *from, const ompl::base::State *to, double maxDistance, ompl::base::State *state) {
    const unsigned int m = modelQueue_->pop();

    state2Link(stateSpace_, from, variables_[m]); // spaceとstateの空間をそろえる

    std::vector<JointAngle> prevAngle(variables_[m].size());
    for(int k=0;k<variables_[m].size();k++){
      if(variables_[m][k]->isRevoluteJoint() || variables_[m][k]->isPrismaticJoint()) {
        prevAngle[k].q = variables_[m][k]->q();
      }else if(variables_[m][k]->isFreeJoint()) {
        prevAngle[k].T = variables_[m][k]->T();
      }
    }

    state2Link(stateSpace_, to, variables_[m]); // spaceとstateの空間をそろえる

    for(int k=0;k<variables_[m].size();k++){
      if(variables_[m][k]->isRevoluteJoint() || variables_[m][k]->isPrismaticJoint()) {
        variables_[m][k]->q() = std::min(std::max(variables_[m][k]->q(), prevAngle[k].q - maxDistance), prevAngle[k].q + maxDistance);
      }else if(variables_[m][k]->isFreeJoint()) {
        for(int j=0;j<3;j++){
          variables_[m][k]->p()[j] = std::min(std::max(variables_[m][k]->p()[j], prevAngle[k].T.translation()[j] - maxDistance), prevAngle[k].T.translation()[j] + maxDistance);
        }
        cnoid::AngleAxisd w(prevAngle[k].T.linear().transpose() * variables_[m][k]->R());
        variables_[m][k]->R() = prevAngle[k].T.linear() * cnoid::AngleAxisd(std::min(std::max(w.angle(), - maxDistance), maxDistance), w.axis());
      }
    }

    link2State(variables_[m], stateSpace_, state); // spaceとstateの空間をそろえる

    modelQueue_->push(m);
  }

};
