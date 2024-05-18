#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <iostream>
#include <ros/package.h>

#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <ik_constraint2/ik_constraint2.h>

namespace global_inverse_kinematics_solver_sample{
  void sample1_4limb(){
    // load robot
    std::string modelfile = ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body";
    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(modelfile);

    // reset manip pose
    robot->rootLink()->p() = cnoid::Vector3(0,0,0.6);
    robot->rootLink()->v().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// rleg
        0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045,// rarm
        0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// lleg
        0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045,// larm
        0.0, 0.0, 0.0};

    for(int j=0; j < robot->numJoints(); ++j){
      robot->joint(j)->q() = reset_manip_pose[j];
    }
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();

    // setup constraints
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    // joint limit
    for(int i=0;i<robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = robot->joint(i);
      constraints0.push_back(constraint);
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
    {
      // task: rleg to current position
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("RLEG_ANKLE_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
      constraint->B_link() = nullptr;
      constraint->B_localpos() = robot->link("RLEG_ANKLE_R")->T() * constraint->A_localpos();
      constraints1.push_back(constraint);
    }
    {
      // task: lleg to current position
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("LLEG_ANKLE_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
      constraint->B_link() = nullptr;
      constraint->B_localpos() = robot->link("LLEG_ANKLE_R")->T() * constraint->A_localpos();
      constraints1.push_back(constraint);
    }
    {
      // task: COM to current position
      std::shared_ptr<ik_constraint2::COMConstraint> constraint = std::make_shared<ik_constraint2::COMConstraint>();
      constraint->A_robot() = robot;
      constraint->B_localp() = robot->centerOfMass();
      constraints1.push_back(constraint);
    }

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1};

    // setup goals
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goals;
    {
      // task: rarm to target.
      std::shared_ptr<ik_constraint2::PositionConstraint> goal = std::make_shared<ik_constraint2::PositionConstraint>();
      goal->A_link() = robot->link("RARM_WRIST_R");
      goal->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
      goal->B_link() = nullptr;
      goal->B_localpos().translation() = cnoid::Vector3(0.3,-0.2,0.8);
      goal->B_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(-1.5,cnoid::Vector3(0,1,0)));
      goals.push_back(goal);
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
    {
      // task: joint angle to target
      for(int i=0;i<robot->numJoints();i++){
        std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
        constraint->joint() = robot->joint(i);
        constraint->targetq() = reset_manip_pose[i];
        constraint->precision() = 1e10; // always satisfied
        nominals.push_back(constraint);
      }
    }

    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(robot->rootLink());
    for(size_t i=0;i<robot->numJoints();i++){
      variables.push_back(robot->joint(i));
    }

    for(size_t i=0;i<constraints.size();i++){
      for(size_t j=0;j<constraints[i].size();j++){
        constraints[i][j]->debugLevel() = 0;//debug
      }
    }
    global_inverse_kinematics_solver::GIKParam param;
    param.debugLevel=1;
    param.projectLink.push_back(robot->link("RARM_WRIST_R"));
    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
    bool solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                             constraints,
                                                             goals,
                                                             nominals,
                                                             param,
                                                             path);
    std::cerr << "solved: " << solved << std::endl;

    for(size_t i=0;i<constraints.size();i++){
      for(size_t j=0;j<constraints[i].size();j++){
        constraints[i][j]->debugLevel() = 0;//not debug
        constraints[i][j]->updateBounds();
        if(constraints[i][j]->isSatisfied()) std::cerr << "constraint " << i << " " << j << ": satisfied"<< std::endl;
        else std::cerr << "constraint " << i << " " << j << ": NOT satisfied"<< std::endl;
      }
    }
    for(size_t i=0;i<goals.size();i++){
      goals[i]->debugLevel() = 0;//not debug
      goals[i]->updateBounds();
      if(goals[i]->isSatisfied()) std::cerr << "goal " << i << ": satisfied"<< std::endl;
      else std::cerr << "goal " << i << ": NOT satisfied"<< std::endl;
    }

    // setup viewer
    choreonoid_viewer::Viewer viewer;
    viewer.objects(robot);

    // main loop
    for(int i=0;i<path->size();i++){
      global_inverse_kinematics_solver::frame2Link(path->at(i),variables);
      robot->calcForwardKinematics();
      robot->calcCenterOfMass();

      std::vector<cnoid::SgNodePtr> markers;
      for(int j=0;j<constraints.size();j++){
        for(int k=0;k<constraints[j].size(); k++){
          constraints[j][k]->updateBounds();
          const std::vector<cnoid::SgNodePtr>& marker = constraints[j][k]->getDrawOnObjects();
          std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
        }
      }
      for(int j=0;j<goals.size();j++){
        goals[j]->debugLevel() = 0;
        goals[j]->updateBounds();
        const std::vector<cnoid::SgNodePtr>& marker = goals[j]->getDrawOnObjects();
        std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
      }
      viewer.drawOn(markers);
      viewer.drawObjects();

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }

  }

}
