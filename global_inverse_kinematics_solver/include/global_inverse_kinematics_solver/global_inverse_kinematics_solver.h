#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_H

#include <global_inverse_kinematics_solver/CnoidStateSpace.h>
#include <global_inverse_kinematics_solver/GIKConstraint.h>
#include <global_inverse_kinematics_solver/GIKGoalSpace.h>
#include <global_inverse_kinematics_solver/GIKStateSpace.h>
#include <global_inverse_kinematics_solver/GIKProjectionEvaluator.h>
#include <choreonoid_viewer/choreonoid_viewer.h>

namespace global_inverse_kinematics_solver{
  class GIKParam {
  public:
    int debugLevel = 0; // 0: no debug message. 1: time measure. 2: internal state

    double timeout = 10.0;
    std::function<bool()> ptc = [](){return false;}; // timeoutが経過するか、ptc()=trueになると解が見つからなくても終了する.
    double delta = 0.2; // planning自体の速さには影響はなく、その後のsimplify, interpolateの速さに影響する. 大きければ大きいほど速いが、干渉計算の正確さが犠牲になる. デフォルトは0.05だが、関節変位のノルムを使う都合上、関節数が多いヒューマノイドではもっと大きい方がいい
    double range = 0.3; // planning自体の速さに影響する.
    double goalBias = 0.05; // デフォルトは0.05だが、もっと大きい方がはやく解ける. goalSampingはIKの変位が大きいので、この値が大きいとsample1回あたりの時間が長くなるデメリットもある. [今は使われていない]
    int planner = 0; // 0: KPIECE. 1: EST. 2: RRT

    std::vector<cnoid::LinkPtr> projectLink;
    cnoid::Isometry3 projectLocalPose = cnoid::Isometry3::Identity();
    double projectCellSize = 0.15; // 要パラチューン.  // 0.05よりも0.1の方が速い. 0.3よりも0.2の方が速い?

    double maxTranslation = 3.0;

    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    unsigned int drawLoop = 100; // スレッド0が、drawLoopに一回描画する

    unsigned int threads = 1; // 1以上

    prioritized_inverse_kinematics_solver2::IKParam pikParam; // satisfiedConvergeLevelは読まれない.
    double nearMaxError = 0.05; // sampleNear時のjointAngleConstraintのmaxError. // 大きいとタスクが達成できない場合に不安定になりやすいが、小さいとIKのloopが多く必要になって遅くなる. 各constraintのmaxErrorも同じ値にせよ

    // post processing
    double shortcutThre = 0.08; // 1つ手前のnodeと1つ後ろのnodeの全ての状態変数の値の差がそれぞれこの値以下である場合、そのnodeは削除される.
    double displacementThre = 0.08; // 一回の最適化での変位上限
    prioritized_inverse_kinematics_solver2::IKParam postpikParam;
    double postpikParam_wmaxVec1 = 1e-1; // その他要素
    double postpikParam_wmaxVec2 = 1e-6; // 末尾要素. // 1e-1だと安定だが収束が悪い. それ以下だと1回で動きすぎてdistance field constraintのinvalid領域に入ってしまう. distance field constraintのtoleranceを大きくした上で、この値を小さくせよ.
    double postpikParam_convergeThre = 1e-1; // この値掛けるsqrt(path.size()). // 1e-1はヒューマノイド用なので自由度が少ない場合は1e-2などにせよ

    std::shared_ptr<std::mutex> modelMutex; // variablesのclone, デストラクタに関するmutex

    GIKParam(){
      pikParam.we = 1e2; // 逆運動学が振動しないこと優先. 1e0だと不安定. 1e3だと大きすぎる
      pikParam.maxIteration = 100; // max iterationに達するか、convergeしたら終了する. isSatisfiedでは終了しない. ゼロ空間でreference angleに可能な限り近づけるタスクがあるので. 1 iterationで0.5msくらいかかるので、stateを1つ作るための時間の上限が見積もれる. 一見、この値を小さくすると早くなりそうだが、goalSampling時に本当はgoalに到達できるのにその前に返ってしまうことで遅くなることがあるため、少ないiterationでも収束するように他のパラメータを調整したほうがいい
      pikParam.minIteration = 100;
      pikParam.checkFinalState = true; // ゼロ空間でreference angleに可能な限り近づけるタスクのprecitionは大きくして、常にsatisfiedになることに注意
      pikParam.calcVelocity = false; // 疎な軌道生成なので、velocityはチェックしない
      pikParam.convergeThre = 2.5e-2; // 要パラチューン. IKConsraintのmaxErrorより小さくないと、収束誤判定する. maxErrorが5e-2の場合、5e-2だと大きすぎる. 5e-3だと小さすぎて時間がかかる. ikのwe, wn, wmax, maxErrorといったパラメータと連動してパラチューンせよ.


      postpikParam.we = 1e2;
      postpikParam.maxIteration = 50;
      postpikParam.minIteration = 0;
      postpikParam.calcVelocity = false; // 疎な軌道生成なので、velocityはチェックしない
    }
  };

  // goalsはconstraintsを含まない. 実際のgoalは、constraintsの末尾にgoalsが追加されたものになる
  // pathは、freeJointはx y z qx qy qz qwの順
  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, // 0: constriant priority 1: constraints
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& goals, // 0: goals(AND).
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals, // 0: nominals
                const GIKParam& param = GIKParam(),
                std::shared_ptr<std::vector<std::vector<double> > > path = nullptr); // 0: states. 1: angles

  bool solveGIK(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, // 0: modelQueue, 1: constriant priority 2: constraints
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals, // 0: modelQueue. 1: goals(AND).
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& nominals, // 0: modelQueue, 1: nominals
                std::shared_ptr<UintQueue> modelQueue,
                const GIKParam& param,
                const std::shared_ptr<std::vector<std::vector<double> > >& path); // 0: goalSpace(OR). 1: states. 2: angles

  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, // 0: constriant priority 1: constraints
                const std::vector<double>& goal, // 0: angles.
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path = nullptr); // 0: states. 1: angles
  bool solveGIK(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, // 0: modelQueue, 1: constriant priority 2: constraints
                const std::vector<double>& goal, // 0: angles.
                std::shared_ptr<UintQueue> modelQueue,
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path = nullptr); // 0: states. 1: angles


  bool shortCut(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                std::shared_ptr<std::vector<std::vector<double> > >& path,
                const GIKParam& param); // 0: states. 1: angles

  bool postProcess(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                   const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, // 0: constriant priority 1: constraints
                   const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& goals, // 0: constraints
                   const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals, // 0: constraints
                   std::shared_ptr<std::vector<std::vector<double> > >& path,
                   const GIKParam& param = GIKParam()); // 0: states. 1: angles

}

#endif
