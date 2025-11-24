#include <global_inverse_kinematics_solver/GIKGoalSpace.h>
#include <global_inverse_kinematics_solver/GIKConstraint.h>

namespace global_inverse_kinematics_solver{

  bool GIKGoalSpace::isSatisfied(const ompl::base::State *st, double *distance) const {
    const unsigned int m = modelQueue_->pop();
    state2Link(si_->getStateSpace(), st, variables_[m]); // spaceとstateの空間をそろえる
    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_[m].begin(); it != bodies_[m].end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    bool satisfied = true;
    double squaredDistance = 0.0;
    for(size_t i=0;i<goals_[m].size();i++){
      for(size_t j=0;j<goals_[m][i].size();j++){
        goals_[m][i][j]->updateBounds();
        if(!goals_[m][i][j]->isSatisfied()) satisfied = false;
        if(distance) squaredDistance += std::pow(goals_[m][i][j]->distance(), 2.0);
      }
    }

    if(distance) *distance = std::sqrt(squaredDistance);

    modelQueue_->push(m);
    return satisfied;
  }

  double GIKGoalSpace::distanceGoal(const ompl::base::State *st) const {
    const unsigned int m = modelQueue_->pop();
    state2Link(si_->getStateSpace(), st, variables_[m]); // spaceとstateの空間をそろえる
    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_[m].begin(); it != bodies_[m].end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    double squaredDistance = 0.0;
    for(size_t i=0;i<goals_[m].size();i++){
      for(size_t j=0;j<goals_[m][i].size();j++){
        goals_[m][i][j]->updateBounds();
        squaredDistance += std::pow(goals_[m][i][j]->distance(), 2.0);
      }
    }

    modelQueue_->push(m);
    return std::sqrt(squaredDistance);
  }


  bool GIKGoalSpace::sampleTo(ompl::base::State *state, const ompl::base::State *source, double* distance) const {
    bool ret = stateSpace_->getGIKConstraint()->projectGoalWithNominal(state, source, goals_, nominals_, distance); // goal projection時についでにnominal-poseに近づけることで、tree全体としてnonimalposeに近づけて、逆運動学をときやすくする.
    si_->enforceBounds(state);

    return ret;
  }

};
