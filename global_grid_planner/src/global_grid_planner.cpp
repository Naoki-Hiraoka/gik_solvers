#include <global_grid_planner/global_grid_planner.h>
#include <iostream>
#include <unordered_set>

namespace global_grid_planner{

  struct EigenVectorXiHash {
    std::size_t operator()(const Eigen::VectorXi& u) const {
      std::size_t hash=0;
      for(int i=0;i<u.rows();i++){
        boost::hash_combine(hash, u[i]);
      }
      return hash;
    }
  };
  struct EigenVectorEqual {
    bool operator()(const Eigen::VectorXi& lhs, const Eigen::VectorXi& rhs) const {
      return lhs == rhs;
    }
  };

  void generateDeltas(int dim,Eigen::VectorXi delta,std::vector<Eigen::VectorXi>& deltas) {
    // if (dim == delta.rows()) {
    //   if(!delta.isZero()){
    //     deltas.push_back(delta);
    //   }
    //   return;
    // }

    // for (int d = -1; d <= 1; d++) {
    //   delta[dim] = d;
    //   generateDeltas(dim+1, delta, deltas);
    // }
    for(int i=0;i<delta.rows();i++){
      for(int j=-1;j<=1;j++){
        Eigen::VectorXi d = Eigen::VectorXi::Zero(delta.rows());
        d[i]=j;
        deltas.push_back(d);
      }
    }
  }

  class ProblemInformation {
  public:
    GGPParam param;
    std::vector<double> initialFrame;
    std::unordered_set<cnoid::BodyPtr> bodies;
    std::vector<cnoid::LinkPtr> variables;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goals;

    std::unordered_set<Eigen::VectorXi, EigenVectorXiHash, EigenVectorEqual> visitedIdx;
    std::unordered_set<Eigen::VectorXi, EigenVectorXiHash, EigenVectorEqual> validIdx;
    std::unordered_set<Eigen::VectorXi, EigenVectorXiHash, EigenVectorEqual> atGoalIdx;
    std::vector<Eigen::VectorXi> deltas;
  };

  class GGPNode : public graph_search2::Node {
  public:
    void calcCost() override {
      frame2Link(this->pi_->initialFrame, this->pi_->variables);
      double distance = 0.0;
      {
        double squaredDistance = 0.0;
        int idx = 0;
        for(cnoid::LinkPtr joint: this->pi_->variables){
          if(joint->isRevoluteJoint() || joint->isPrismaticJoint()) {
            double diff = this->idx_[idx] * this->pi_->param.resolutionQ;
            joint->q() += diff;
            if(std::abs(diff) >= 2 * M_PI ||
               joint->q() < joint->q_lower() ||
               joint->q() > joint->q_upper()) {
              this->inValid_ = true;
              return;
            }
            squaredDistance += std::pow(delta_[idx],2);
            idx++;
          }else if(joint->isFreeJoint()) {
            Eigen::Vector3i diff = delta_.segment<3>(idx);
            double diffX = this->idx_[idx] * this->pi_->param.resolutionP;
            if(std::abs(diffX) > this->pi_->param.maxTranslation){
              this->inValid_ = true;
              return;
            }
            joint->p()[0] += diffX;
            idx++;
            double diffY = this->idx_[idx] * this->pi_->param.resolutionP;
            if(std::abs(diffY) > this->pi_->param.maxTranslation){
              this->inValid_ = true;
              return;
            }
            joint->p()[1] += diffY;
            idx++;
            double diffZ = this->idx_[idx] * this->pi_->param.resolutionP;
            if(std::abs(diffZ) > this->pi_->param.maxTranslation){
              this->inValid_ = true;
              return;
            }
            joint->p()[2] += diffZ;
            idx++;

            if(this->pi_->param.gridType == GGPParam::GridType::XYZTHETA){
              while(this->idx_[idx] > this->pi_->param.resolutionR) this->idx_[idx] -= 2*this->pi_->param.resolutionR;
              while(this->idx_[idx] <= this->pi_->param.resolutionR) this->idx_[idx] += 2*this->pi_->param.resolutionR;
              double diffTheta = this->idx_[idx] * 2*M_PI / this->pi_->param.resolutionR;
              joint->R() = cnoid::AngleAxisd(diffTheta, cnoid::Vector3::UnitZ()).toRotationMatrix() * joint->R();
              squaredDistance += std::pow(delta_[idx],2);
              idx++;
            }
            cnoid::Vector3 diffLocal = joint->R().transpose() * diff.cast<double>();
            if(diffLocal[0]>0.0) diffLocal[0] *= this->pi_->param.forwardScale;
            squaredDistance += diffLocal.squaredNorm();
          }
        }
        distance = std::sqrt(squaredDistance);
      }
      link2Frame(this->pi_->variables, this->frame_);

      bool visited = this->pi_->visitedIdx.contains(this->idx_);
      if(!visited) {
        for(const cnoid::BodyPtr& body: this->pi_->bodies){
          body->calcForwardKinematics();
          body->calcCenterOfMass();
        }
        this->pi_->visitedIdx.insert(this->idx_);
      }

      if(visited) {
        this->inValid_ = !this->pi_->validIdx.contains(this->idx_);
        if(this->inValid_) return;
      }else{
        this->inValid_ = false;
        for(const std::shared_ptr<ik_constraint2::IKConstraint>& constraint: this->pi_->constraints){
          constraint->updateBounds();
          if(!constraint->isSatisfied()) {
            this->inValid_ = true;
            return;
          }
        }
        this->pi_->validIdx.insert(this->idx_);
      }

      // gCost
      if(this->parent_ == nullptr){
        this->gCost_ = 0.0;
      }else{
        const std::shared_ptr<GGPNode>& parent = std::static_pointer_cast<GGPNode>(this->parent_);
        this->gCost_ = parent->gCost();

        this->gCost_ += distance;
      }

      // hCost
      this->hCost_ = 0.0;

      // isGoal
      if(visited) {
        this->isGoal_ = this->pi_->atGoalIdx.contains(this->idx_);
      }else{
        this->isGoal_ = true;
        for(const std::shared_ptr<ik_constraint2::IKConstraint>& goal: this->pi_->goals){
          goal->updateBounds();
          if(!goal->isSatisfied()) {
            this->isGoal_ = false;
            break;
          }
        }
        if(this->isGoal_){
          this->pi_->atGoalIdx.insert(this->idx_);
        }
      }

      // hash
      this->hash_ = EigenVectorXiHash()(this->idx_);

    }
    bool isSame(const std::shared_ptr<Node>& other) const override{
      std::shared_ptr<GGPNode> tmpother = std::static_pointer_cast<GGPNode>(other);
      return this->idx_ == tmpother->idx_;
    }
    bool checkValidity() override {
      if(!this->parent_) return true;
      //std::shared_ptr<FSPNode> parent = std::static_pointer_cast<FSPNode>(this->parent_);

      if(this->pi_->param.debugLevel >= 2) {
        std::cerr << this->idx_.transpose() << std::endl;
      }

      return true;
    }
    std::list<std::shared_ptr<Node> > expand() override {
      std::list<std::shared_ptr<Node> > children;
      for(Eigen::VectorXi& delta: this->pi_->deltas){
        std::shared_ptr<GGPNode> child = std::make_shared<GGPNode>();
        child->parent() = this->shared_from_this();
        child->pi_ = this->pi_;
        child->idx_ = this->idx_ + delta;
        child->delta_ = delta;
        child->calcCost();
        if(child->inValid_) continue;
        children.push_back(child);
      }
      return children;
    }
  public:
    std::shared_ptr<ProblemInformation> pi_;
    Eigen::VectorXi idx_;
    Eigen::VectorXi delta_;

    std::vector<double> frame_;
    bool inValid_ = false;
  };


  bool solveGGP(const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& goals,
                const GGPParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){

    std::shared_ptr<ProblemInformation> pi = std::make_shared<ProblemInformation>();
    pi->param = param;
    link2Frame(variables, pi->initialFrame);
    for(cnoid::LinkPtr variable: variables){
      pi->bodies.insert(variable->body());
    }
    pi->variables = variables;
    pi->constraints = constraints;
    pi->goals = goals;
    std::shared_ptr<GGPNode> startNode = std::make_shared<GGPNode>();
    startNode->pi_ = pi;
    {
      int idx = 0;
      for(cnoid::LinkPtr variable: variables){
        if(variable->isRevoluteJoint() || variable->isPrismaticJoint()) {
          idx++;
        }else if(variable->isFreeJoint()) {
          idx+=3;
          if(param.gridType == GGPParam::GridType::XYZTHETA){
            idx+=1;
          }
        }
      }
      startNode->idx_ = Eigen::VectorXi::Zero(idx);
      startNode->delta_ = Eigen::VectorXi::Zero(idx);
      generateDeltas(0,Eigen::VectorXi::Zero(idx),pi->deltas);
    }

    std::vector<std::shared_ptr<GGPNode> > nodePath = graph_search2::path<GGPNode>(graph_search2::solve(std::list<std::shared_ptr<graph_search2::Node> >{startNode}, param.gsParam));

    if(nodePath.size() == 0){
      return false;
    }

    frame2Link(nodePath.back()->frame_, variables);
    for(const cnoid::BodyPtr& body: pi->bodies){
      body->calcForwardKinematics();
      body->calcCenterOfMass();
    }

    if(path){
      path->clear();
      for(int i=0;i<nodePath.size();i++){
        path->push_back(nodePath[i]->frame_);
      }
    }

    return true;
  }

  void frame2Link(const std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& links){
    unsigned int i=0;
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        links[l]->q() = frame[i];
        i+=1;
      }else if(links[l]->isFreeJoint()) {
        links[l]->p()[0] = frame[i+0];
        links[l]->p()[1] = frame[i+1];
        links[l]->p()[2] = frame[i+2];
        links[l]->R() = cnoid::Quaternion(frame[i+6],
                                          frame[i+3],
                                          frame[i+4],
                                          frame[i+5]).toRotationMatrix();
        i+=7;
      }
    }
  }
  void link2Frame(const std::vector<cnoid::LinkPtr>& links, std::vector<double>& frame){
    frame.clear();
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        frame.push_back(links[l]->q());
      }else if(links[l]->isFreeJoint()) {
        frame.push_back(links[l]->p()[0]);
        frame.push_back(links[l]->p()[1]);
        frame.push_back(links[l]->p()[2]);
        cnoid::Quaternion q(links[l]->R());
        frame.push_back(q.x());
        frame.push_back(q.y());
        frame.push_back(q.z());
        frame.push_back(q.w());
      }
    }
  }



};
