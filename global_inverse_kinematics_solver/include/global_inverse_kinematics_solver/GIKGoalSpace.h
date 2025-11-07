#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKGOALSPACE_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKGOALSPACE_H

#include <ompl_near_projection/ompl_near_projection.h>
#include <ik_constraint2/ik_constraint2.h>
#include <global_inverse_kinematics_solver/GIKStateSpace.h>

namespace global_inverse_kinematics_solver{
  OMPL_CLASS_FORWARD(GIKGoalSpace); // *Ptrを定義. (shared_ptr)

  class GIKGoalSpace : public ompl_near_projection::NearGoalSpace{
  public:
    GIKGoalSpace(const ompl::base::SpaceInformationPtr &si, const ompl::base::StateSpacePtr ambientSpace, std::shared_ptr<UintQueue>& modelQueue, const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, const std::vector<std::vector<cnoid::LinkPtr> >& variables, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& nominals) :
      NearGoalSpace(si),
      modelQueue_(modelQueue),
      variables_(variables),
      goals_(goals),
      nominals_(nominals)
    {
      for(int i=0;i<variables_.size();i++){
        bodies_.push_back(getBodies(variables_[i]));
      }

      stateSpace_ = std::static_pointer_cast<GIKStateSpace>(si->getStateSpace());
    }

    virtual bool isSatisfied(const ompl::base::State *st, double *distance) const override;
    virtual double distanceGoal(const ompl::base::State *st) const override;

    bool sampleTo(ompl::base::State *state, const ompl::base::State *source, double* distance = nullptr) const override;

 protected:
    GIKStateSpacePtr stateSpace_; // goalSpaceではなく、constrainedSpace
    mutable std::shared_ptr<UintQueue> modelQueue_;
    const std::vector<std::vector<cnoid::LinkPtr> > variables_;
    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goals_;
    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > nominals_;
    std::vector<std::set<cnoid::BodyPtr> > bodies_;

  };
};

#endif
