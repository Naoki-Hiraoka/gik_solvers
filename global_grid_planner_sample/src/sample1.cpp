#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/MeshGenerator>
#include <cnoid/SceneMarkers>
#include <iostream>

#include <global_grid_planner/global_grid_planner.h>
#include <ik_constraint2/ik_constraint2.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>

namespace global_grid_planner_sample{
  void sample1(){
    cnoid::MeshGenerator meshGenerator;
    cnoid::BodyPtr bigBox = new cnoid::Body();
    {
      cnoid::SgShapePtr shape = new cnoid::SgShape();
      shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,1,0.1)));
      cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
      material->setTransparency(0.8);
      shape->setMaterial(material);
      cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
      posTransform->translation() = cnoid::Vector3(0,0,0);
      posTransform->addChild(shape);
      cnoid::LinkPtr link = new cnoid::Link();
      link->addShapeNode(posTransform);
      link->setJointType(cnoid::Link::JointType::FreeJoint);
      bigBox->setRootLink(link);
    }
    bigBox->calcForwardKinematics();
    cnoid::BodyPtr smallBox = new cnoid::Body();
    {
      cnoid::SgShapePtr shape = new cnoid::SgShape();
      shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.2,0.4,0.2)));
      cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
      material->setTransparency(0.3);
      shape->setMaterial(material);
      cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
      posTransform->translation() = cnoid::Vector3(0,0,0.0);
      posTransform->addChild(shape);
      cnoid::LinkPtr link = new cnoid::Link();
      link->addShapeNode(posTransform);
      link->setJointType(cnoid::Link::JointType::FreeJoint);
      smallBox->setRootLink(link);
    }
    smallBox->rootLink()->p() << 0,0,-0.5;
    smallBox->calcForwardKinematics();

    // setup constraints
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
    {
      std::shared_ptr<ik_constraint2_bullet::BulletCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletCollisionConstraint>();
      constraint->A_link() = bigBox->rootLink();
      constraint->B_link() = smallBox->rootLink();
      constraint->tolerance() = 0.01;
      constraint->ignoreDistance() = 1e10; // 描画のため
      constraint->updateBounds(); // キャッシュを内部に作る.
      constraints.push_back(constraint);
    }

    // setup goals
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goals;
    {
      std::shared_ptr<ik_constraint2::PositionConstraint> goal = std::make_shared<ik_constraint2::PositionConstraint>();
      goal->A_link() = smallBox->rootLink();
      goal->B_link() = nullptr;
      goal->B_localpos().translation() = cnoid::Vector3(0.0,0.0,0.5);
      goals.push_back(goal);
    }

    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(smallBox->rootLink());

    global_grid_planner::GGPParam param;
    param.debugLevel=2;
    param.maxTranslation=1.0;
    param.gsParam.debugLevel=2;
    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();

    bool solved = global_grid_planner::solveGGP(variables,
                                                constraints,
                                                goals,
                                                param,
                                                path);
    std::cerr << "solved: " << solved << std::endl;

    // setup viewer
    choreonoid_viewer::Viewer viewer;
    viewer.objects(bigBox);
    viewer.objects(smallBox);
    viewer.drawObjects();

    // main loop
    for(int i=0;i<path->size();i++){
      global_grid_planner::frame2Link(path->at(i),variables);
      smallBox->calcForwardKinematics();
      smallBox->calcCenterOfMass();

      std::vector<cnoid::SgNodePtr> markers;
      for(int j=0;j<constraints.size();j++){
        constraints[j]->updateBounds();
        const std::vector<cnoid::SgNodePtr>& marker = constraints[j]->getDrawOnObjects();
        std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
      }
      for(int j=0;j<goals.size();j++){
        goals[j]->updateBounds();
        const std::vector<cnoid::SgNodePtr>& marker = goals[j]->getDrawOnObjects();
        std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
      }
      viewer.drawOn(markers);
      viewer.drawObjects();

      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    while(true){
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  }

}
