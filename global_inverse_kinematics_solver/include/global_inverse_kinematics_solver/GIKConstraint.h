#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKCONSTRAINT_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKCONSTRAINT_H

#include <ompl_near_projection/ompl_near_projection.h>
#include <ik_constraint2/ik_constraint2.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <global_inverse_kinematics_solver/CnoidStateSpace.h>
#include <choreonoid_viewer/choreonoid_viewer.h>

namespace global_inverse_kinematics_solver{
  OMPL_CLASS_FORWARD(GIKConstraint); // GIKConstraintPtrを定義. (shared_ptr)

  class GIKConstraint : public ompl_near_projection::NearConstraint{
  public:
    GIKConstraint(const ompl::base::StateSpacePtr ambientSpace, std::shared_ptr<UintQueue>& modelQueue, const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, const std::vector<std::vector<cnoid::LinkPtr> >& variables, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& rejections) :
      NearConstraint(1,0,0), // 3つの引数は使われないので適当に与える
      ambientSpace_(ambientSpace),
      modelQueue_(modelQueue),
      variables_(variables),
      constraints_(constraints),
      rejections_(rejections)
    {


      for(int i=0;i<variables_.size();i++){
        bodies_.push_back(getBodies(variables_[i]));
        ikConstraints_.push_back(constraints_[i]);
        ikConstraints_.back().push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >());
        tasks_.push_back(std::vector<std::shared_ptr<prioritized_qp_base::Task> >());
        nominalConstraints_.emplace_back();
      }
    }

    // Constraintクラスでpure virtual関数として定義されているので適当に実態を定義する
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override {return ;}

    virtual bool project(ompl::base::State *state) const override;
    virtual bool projectNearValid(ompl::base::State *state, const ompl::base::State *near, double* distance = nullptr) const override;
    virtual bool projectNearValidWithNominal(ompl::base::State *state, const ompl::base::State *near, double* distance = nullptr) const;
    virtual double distance (const ompl::base::State *state) const override;
    virtual bool isSatisfied (const ompl::base::State *state) const override;
    virtual bool isSatisfied (const ompl::base::State *state, double *distance) const;

    // 各要素ごとにfromからの変位がmaxDistance以下になる範囲内でtoに近づくstateを返す.
    virtual void elementWiseDistanceLimit(const ompl::base::State *from, const ompl::base::State *to, double maxDistance, ompl::base::State *state);

    //const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints() const {return constraints_; }
    std::shared_ptr<choreonoid_viewer::Viewer>& viewer() {return viewer_;}
    const std::shared_ptr<choreonoid_viewer::Viewer>& viewer() const {return viewer_;}
    unsigned int& drawLoop() { return drawLoop_; }
    const unsigned int& drawLoop() const { return drawLoop_; }
    double& nearMaxError() { return nearMaxError_; }
    const double& nearMaxError() const { return nearMaxError_; }
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& nominalConstraints() { return nominalConstraints_; } // nominalConstraintsのisSatisfiedは常にtrueを返す必要がある
    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& nominalConstraints() const { return nominalConstraints_; }
    prioritized_inverse_kinematics_solver2::IKParam& param() { return param_;}
    const prioritized_inverse_kinematics_solver2::IKParam& param() const { return param_; }
  protected:
    const ompl::base::StateSpacePtr ambientSpace_;

    // model queueで管理.
    mutable std::shared_ptr<UintQueue> modelQueue_;
    const std::vector<std::vector<cnoid::LinkPtr> > variables_;
    const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > constraints_;
    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > rejections_;
    std::vector<std::set<cnoid::BodyPtr> > bodies_;
    mutable std::vector<std::vector<std::shared_ptr<prioritized_qp_base::Task> > > tasks_;
    // constraintsの末尾にJointAngleConstraintを加えたもの
    mutable std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > ikConstraints_;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > nominalConstraints_;

    prioritized_inverse_kinematics_solver2::IKParam param_;
    std::shared_ptr<choreonoid_viewer::Viewer> viewer_ = nullptr;
    mutable int loopCount_ = 0;
    unsigned int drawLoop_ = 100;
    double nearMaxError_ = 0.05;
  };

  OMPL_CLASS_FORWARD(GIKConstraint2); // GIKConstraint2Ptrを定義. (shared_ptr)
  class GIKConstraint2 : public GIKConstraint{
  public:
    GIKConstraint2(const ompl::base::StateSpacePtr ambientSpace, std::shared_ptr<UintQueue>& modelQueue, const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, const std::vector<std::vector<cnoid::LinkPtr> >& variables, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& rejections) :
      GIKConstraint(ambientSpace, modelQueue, constraints,  variables, rejections)
    {
    }

    virtual bool projectNearValid(ompl::base::State *state, const ompl::base::State *near, double* distance = nullptr) const override;

    double projectionRange = 0.2;
    double projectionTrapThre = 0.03;
  };
};

#endif
