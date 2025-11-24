#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/MeshGenerator>

#include <iostream>
#include <ros/package.h>

#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <ik_constraint2/ik_constraint2.h>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>

#include "samplerobot_common.h"

namespace global_inverse_kinematics_solver_sample{
  void sample6_root(){
    cnoid::BodyPtr robot;
    cnoid::BodyPtr abstractRobot;
    cnoid::BodyPtr horizontalRobot;
    generateSampleRobot(robot, abstractRobot, horizontalRobot);

    cnoid::MeshGenerator meshGenerator;
    meshGenerator.setDivisionNumber(8); // default 20. 20だとcddlibが遅い
    cnoid::BodyPtr obstacle = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,1,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0,-0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,1,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1,0,0.35);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
      }
      obstacle->setRootLink(rootLink);
    }

    // collision world
    std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
                                                                                                                                                               5,//size_y
                                                                                                                                                               5,//size_z
                                                                                                                                                               0.04,//resolution
                                                                                                                                                               -2.5,//origin_x
                                                                                                                                                               -2.5,//origin_y
                                                                                                                                                               -2.5,//origin_z
                                                                                                                                                               0.5, // max_distance
                                                                                                                                                               false// propagate_negative_distances
                                                                                                                                                               );
    EigenSTL::vector_Vector3d vertices;
    for(int i=0;i<obstacle->numLinks();i++){
      std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.04);
      for(int j=0;j<vertices_.size();j++){
        vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
      }
    }
    field->addPointsToField(vertices);

    // support polygon
    cnoid::BodyPtr support = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          cnoid::MeshGenerator::Extrusion extrusion;
          extrusion.crossSection.push_back(cnoid::Vector2(+0.4,+0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(-0.4,+0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(-0.4,-0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(+0.4,-0.4));
          extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
          extrusion.spine.push_back(cnoid::Vector3(0,0,0));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          shape->setMesh(meshGenerator.generateExtrusion(extrusion));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0,0.0);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          cnoid::MeshGenerator::Extrusion extrusion;
          extrusion.crossSection.push_back(cnoid::Vector2(+0.4,+0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(-0.4,+0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(-0.4,-0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(+0.4,-0.4));
          extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
          extrusion.spine.push_back(cnoid::Vector3(0,0,0));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          shape->setMesh(meshGenerator.generateExtrusion(extrusion));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.0,0,0.4);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
      }
      support->setRootLink(rootLink);
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    {
      // horizontal
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = abstractRobot->rootLink();
      constraint->B_link() = horizontalRobot->rootLink();
      constraint->eval_link() = horizontalRobot->rootLink();
      constraint->weight() << 1.0, 1.0, 1.0, 0.0, 0.0, 1.0;
      //constraint->debugLevel() = 2;
      constraints0.push_back(constraint);
    }
    {
      // horizontal
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = horizontalRobot->rootLink();
      constraint->B_link() = nullptr;
      constraint->eval_link() = nullptr;
      constraint->weight() << 0.0, 0.0, 0.0, 1.0, 1.0, 0.0;
      //constraint->debugLevel() = 2;
      constraints0.push_back(constraint);
    }
    {
      // root collision
      std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
      constraint->A_link() = abstractRobot->rootLink();
      constraint->field() = field;
      constraint->tolerance() = 0.04;
      constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
      //constraint->debugLevel() = 2;
      constraints0.push_back(constraint);
    }
    {
      // pitch > 0
      std::shared_ptr<ik_constraint2::RegionConstraint> constraint = std::make_shared<ik_constraint2::RegionConstraint>();
      constraint->A_link() = abstractRobot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.1,0.0,0.0);
      constraint->B_link() = abstractRobot->rootLink();
      constraint->eval_link() = nullptr;
      constraint->weightR().setZero();
      constraint->C().resize(1,3);
      constraint->C().insert(0,2) = 1.0;
      constraint->dl().resize(1);
      constraint->dl()[0] = -1e10;
      constraint->du().resize(1);
      constraint->du()[0] = 0.0;
      //constraint->debugLevel() = 2;
      constraints0.push_back(constraint);
    }
    {
      // pitch < 90
      std::shared_ptr<ik_constraint2::RegionConstraint> constraint = std::make_shared<ik_constraint2::RegionConstraint>();
      constraint->A_link() = abstractRobot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
      constraint->B_link() = abstractRobot->rootLink();
      constraint->eval_link() = nullptr;
      constraint->weightR().setZero();
      constraint->C().resize(1,3);
      constraint->C().insert(0,2) = 1.0;
      constraint->dl().resize(1);
      constraint->dl()[0] = -1e10;
      constraint->du().resize(1);
      constraint->du()[0] = 0.0;
      //constraint->debugLevel() = 2;
      constraints0.push_back(constraint);
    }
    {
      // roll = 0
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = abstractRobot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.1,0.0);
      constraint->B_link() = abstractRobot->rootLink();
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,-0.1,0.0);
      constraint->eval_link() = nullptr;
      constraint->weight() << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
      //constraint->debugLevel() = 2;
      constraints0.push_back(constraint);
    }

    {
      std::shared_ptr<ik_constraint2::ORConstraint> orconstraint = std::make_shared<ik_constraint2::ORConstraint>();
      //orconstraint->debugLevel() = 2;
      constraints0.push_back(orconstraint);
      {
        // 両足起立
        std::shared_ptr<ik_constraint2::ANDConstraint> andconstraint = std::make_shared<ik_constraint2::ANDConstraint>();
        //andconstraint->debugLevel() = 2;
        orconstraint->children().push_back(andconstraint);
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEG");
          constraint->B_link() = support->rootLink();
          constraint->useSingleMeshA() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->A_link()->collisionShape(),
                                                      constraint->A_FACE_C(),
                                                      constraint->A_FACE_dl(),
                                                      constraint->A_FACE_du());
          constraint->useSingleMeshB() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                      constraint->B_FACE_C(),
                                                      constraint->B_FACE_dl(),
                                                      constraint->B_FACE_du());
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          //constraint->debugLevel() = 2;
          andconstraint->children().push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEG");
          constraint->B_link() = support->rootLink();
          constraint->useSingleMeshA() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->A_link()->collisionShape(),
                                                      constraint->A_FACE_C(),
                                                      constraint->A_FACE_dl(),
                                                      constraint->A_FACE_du());
          constraint->useSingleMeshB() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                      constraint->B_FACE_C(),
                                                      constraint->B_FACE_dl(),
                                                      constraint->B_FACE_du());
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          //constraint->debugLevel() = 2;
          andconstraint->children().push_back(constraint);
        }
      }
      {
        // 四足
        std::shared_ptr<ik_constraint2::ANDConstraint> andconstraint = std::make_shared<ik_constraint2::ANDConstraint>();
        //andconstraint->debugLevel() = 2;
        orconstraint->children().push_back(andconstraint);
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("RLEG");
          constraint->B_link() = support->rootLink();
          constraint->useSingleMeshA() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->A_link()->collisionShape(),
                                                      constraint->A_FACE_C(),
                                                      constraint->A_FACE_dl(),
                                                      constraint->A_FACE_du());
          constraint->useSingleMeshB() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                      constraint->B_FACE_C(),
                                                      constraint->B_FACE_dl(),
                                                      constraint->B_FACE_du());
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          //constraint->debugLevel() = 2;
          andconstraint->children().push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("LLEG");
          constraint->B_link() = support->rootLink();
          constraint->useSingleMeshA() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->A_link()->collisionShape(),
                                                      constraint->A_FACE_C(),
                                                      constraint->A_FACE_dl(),
                                                      constraint->A_FACE_du());
          constraint->useSingleMeshB() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                      constraint->B_FACE_C(),
                                                      constraint->B_FACE_dl(),
                                                      constraint->B_FACE_du());
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          //constraint->debugLevel() = 2;
          andconstraint->children().push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("RARM");
          constraint->B_link() = support->rootLink();
          constraint->useSingleMeshA() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->A_link()->collisionShape(),
                                                      constraint->A_FACE_C(),
                                                      constraint->A_FACE_dl(),
                                                      constraint->A_FACE_du());
          constraint->useSingleMeshB() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                      constraint->B_FACE_C(),
                                                      constraint->B_FACE_dl(),
                                                      constraint->B_FACE_du());
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          //constraint->debugLevel() = 2;
          andconstraint->children().push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("LARM");
          constraint->B_link() = support->rootLink();
          constraint->useSingleMeshA() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->A_link()->collisionShape(),
                                                      constraint->A_FACE_C(),
                                                      constraint->A_FACE_dl(),
                                                      constraint->A_FACE_du());
          constraint->useSingleMeshB() = false; // support polygonを個別にチェック
          choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                      constraint->B_FACE_C(),
                                                      constraint->B_FACE_dl(),
                                                      constraint->B_FACE_du());
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          //constraint->debugLevel() = 2;
          andconstraint->children().push_back(constraint);
        }
      }

    }


    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(robot);
    viewer->objects(abstractRobot);
    viewer->objects(horizontalRobot);
    viewer->objects(obstacle);
    viewer->objects(support);
    viewer->drawObjects();

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0};

    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(abstractRobot->rootLink());
    variables.push_back(horizontalRobot->rootLink());

    std::vector<double> goal;
    {
      cnoid::Isometry3 org = abstractRobot->rootLink()->T();
      abstractRobot->rootLink()->translation() += cnoid::Vector3(1.0,0.0,0.4);
      horizontalRobot->rootLink()->T() = abstractRobot->rootLink()->T();
      global_inverse_kinematics_solver::link2Frame(variables, goal);
      abstractRobot->rootLink()->T() = org;
      horizontalRobot->rootLink()->T() = abstractRobot->rootLink()->T();
    }

    global_inverse_kinematics_solver::GIKParam param;
    param.debugLevel=0;
    param.range = 2.0; // rootlinkのtranslationのuniform samplingの幅が大きすぎて、rangeで縮小すると、rootLinkのrotationの変位が小さくなってしまうことから、rangeは大きい方がいい
    param.delta = 0.1; // 大きければ大きいほど速いが、干渉計算の正確さが犠牲になる
    param.timeout = 30.0;
    param.viewer = viewer;
    param.drawLoop = 1; // 1drawに10msくらいかかることに注意
    param.maxTranslation = 3.0;
    param.pikParam.debugLevel = 0;
    param.pikParam.maxIteration = 15; // collision invertは振動しやすい
    param.threads = 10;
    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
    bool solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                             constraints,
                                                             goal,
                                                             param,
                                                             path);
    std::cerr << "solved: " << solved << std::endl;

    // main loop
    for(int i=0;i<path->size();i++){
      global_inverse_kinematics_solver::frame2Link(path->at(i),variables);
      abstractRobot->calcForwardKinematics(false);
      abstractRobot->calcCenterOfMass();
      horizontalRobot->calcForwardKinematics(false);
      horizontalRobot->calcCenterOfMass();

      robot->rootLink()->T() = abstractRobot->rootLink()->T();
      robot->calcForwardKinematics(false);
      robot->calcCenterOfMass();

      viewer->drawObjects();

      getchar();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    while(true){
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

}
