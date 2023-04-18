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
    GIKConstraint(const ompl::base::StateSpacePtr ambientSpace, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, const std::vector<cnoid::LinkPtr>& variables) :
      NearConstraint(1,0,0), // 3つの引数は使われないので適当に与える
      ambientSpace_(ambientSpace),
      variables_(variables),
      bodies_(getBodies(variables_)),
      constraints_(constraints),
      ikConstraints_(constraints)
    {
      param_.we = 1e2; // 逆運動学が振動しないこと優先. 1e0だと不安定. 1e3だと大きすぎる
      param_.maxIteration = 100; // 200 iterationに達するか、convergeしたら終了する. isSatisfiedでは終了しない. ゼロ空間でreference angleに可能な限り近づけるタスクがあるので.
      param_.minIteration = 100;
      param_.checkFinalState = true; // ゼロ空間でreference angleに可能な限り近づけるタスクのprecitionは大きくして、常にsatisfiedになることに注意
      param_.calcVelocity = false; // 疎な軌道生成なので、velocityはチェックしない
      param_.convergeThre = 2.5e-2; // 要パラチューン. IKConsraintのmaxErrorより小さくないと、収束誤判定する. maxErrorが5e-2の場合、5e-2だと大きすぎる. 5e-3だと小さすぎて時間がかかる. ikのwe, wn, wmax, maxErrorといったパラメータと連動してパラチューンせよ.
      ikConstraints_.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >());
    }

    // Constraintクラスでpure virtual関数として定義されているので適当に実態を定義する
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override {return ;}

    virtual bool project(ompl::base::State *state) const override;
    virtual bool projectNear(ompl::base::State *state, const ompl::base::State *near) const override;
    virtual double distance (const ompl::base::State *state) const override;
    virtual bool isSatisfied (const ompl::base::State *state) const override;
    virtual bool isSatisfied (const ompl::base::State *state, double *distance) const;

    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints() const {return constraints_; }
    std::shared_ptr<choreonoid_viewer::Viewer>& viewer() {return viewer_;}
    const std::shared_ptr<choreonoid_viewer::Viewer>& viewer() const {return viewer_;}
  protected:
    const ompl::base::StateSpacePtr ambientSpace_;
    const std::vector<cnoid::LinkPtr>& variables_;
    const std::set<cnoid::BodyPtr> bodies_;
    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints_;
    mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks_;
    prioritized_inverse_kinematics_solver2::IKParam param_;

    // constraintsの末尾にJointAngleConstraintを加えたもの
    mutable std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > ikConstraints_;

    std::shared_ptr<choreonoid_viewer::Viewer> viewer_ = nullptr;
    mutable int loopCount_ = 0;
    int drawLoop_ = 100;
  };
};

#endif
