#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <iostream>
#include <ros/package.h>

#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <ik_constraint2/ik_constraint2.h>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>
#include <choreonoid_bullet/choreonoid_bullet.h>

namespace global_inverse_kinematics_solver_sample{
  void sample9_opt(){
    cnoid::BodyLoader bodyLoader;

    // load robot
    std::string modelfile = ros::package::getPath("msl_hand_models") + "/JAXON_RED_WITH_MSLHAND/JAXON_REDmain.wrl";
    cnoid::BodyPtr robot = bodyLoader.load(modelfile);
    if(!robot) std::cerr << "!robot" << std::endl;

    // load desk
    std::string deskModelfile = ros::package::getPath("global_inverse_kinematics_solver_sample") + "/models/desk.wrl";
    cnoid::BodyPtr desk = bodyLoader.load(deskModelfile);

    desk->rootLink()->p() = cnoid::Vector3(0.9,0.0,0.7); // これ以上Zが高いとreset-manip-poseの手と干渉する
    desk->calcForwardKinematics();
    desk->calcCenterOfMass();

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(std::vector<cnoid::BodyPtr>{robot, desk});
    viewer->drawObjects(true);

    // setup constraints
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1; // 自己干渉のvclip計算などに時間がかかるので、使いまわしたほうがいい. collisionはsoft constraintにした方が安定する

    // task: joint limit
    for(int i=0;i<robot->numJoints()-12;i++){ // handの12関節を除く
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = robot->joint(i);
      constraints0.push_back(constraint);
    }

    {
      // task: self collision
      std::vector<std::vector<std::string> > pairs {
        std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT6"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT3"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT4"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT5"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT6"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT2","WAIST"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT3","WAIST"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT4","WAIST"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT5","WAIST"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT6","WAIST"}, std::vector<std::string>{"LARM_JOINT2","WAIST"}, std::vector<std::string>{"LARM_JOINT3","WAIST"}, std::vector<std::string>{"LARM_JOINT4","WAIST"}, std::vector<std::string>{"LARM_JOINT5","WAIST"}, std::vector<std::string>{"LARM_JOINT6","WAIST"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT7"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT7"}, std::vector<std::string>{"LARM_JOINT7","WAIST"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT7"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT7","WAIST"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT2"}, std::vector<std::string>{"RLEG_JOINT2","HANDBASE_R"}, std::vector<std::string>{"RLEG_JOINT3","HANDBASE_R"}, std::vector<std::string>{"RLEG_JOINT5","HANDBASE_R"}, std::vector<std::string>{"LLEG_JOINT2","HANDBASE_R"}, std::vector<std::string>{"LLEG_JOINT3","HANDBASE_R"}, std::vector<std::string>{"LLEG_JOINT5","HANDBASE_R"}, std::vector<std::string>{"WAIST","HANDBASE_R"}, std::vector<std::string>{"CHEST_JOINT1","HANDBASE_R"}, std::vector<std::string>{"CHEST_JOINT2","HANDBASE_R"}, std::vector<std::string>{"HEAD_JOINT1","HANDBASE_R"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT0"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT2"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT3"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT4"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","R_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT3","R_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT5","R_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT2","R_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT3","R_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT5","R_THUMB_JOINT1"}, std::vector<std::string>{"WAIST","R_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT1","R_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT2","R_THUMB_JOINT1"}, std::vector<std::string>{"HEAD_JOINT1","R_THUMB_JOINT1"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT0"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","HANDBASE_L"}, std::vector<std::string>{"RLEG_JOINT3","HANDBASE_L"}, std::vector<std::string>{"RLEG_JOINT5","HANDBASE_L"}, std::vector<std::string>{"LLEG_JOINT2","HANDBASE_L"}, std::vector<std::string>{"LLEG_JOINT3","HANDBASE_L"}, std::vector<std::string>{"LLEG_JOINT5","HANDBASE_L"}, std::vector<std::string>{"WAIST","HANDBASE_L"}, std::vector<std::string>{"CHEST_JOINT1","HANDBASE_L"}, std::vector<std::string>{"CHEST_JOINT2","HANDBASE_L"}, std::vector<std::string>{"HEAD_JOINT1","HANDBASE_L"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT0"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT2"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT3"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT4"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","L_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT3","L_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT5","L_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT2","L_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT3","L_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT5","L_THUMB_JOINT1"}, std::vector<std::string>{"WAIST","L_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT1","L_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT2","L_THUMB_JOINT1"}, std::vector<std::string>{"HEAD_JOINT1","L_THUMB_JOINT1"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT0"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"HANDBASE_R","HANDBASE_L"}, std::vector<std::string>{"R_THUMB_JOINT1","L_THUMB_JOINT1"}
      };
      std::unordered_map<cnoid::LinkPtr, std::shared_ptr<btConvexShape> > collisionModels;
      for(int i=0;i<robot->numLinks();i++){
        collisionModels[robot->link(i)] = choreonoid_bullet::convertToBulletModel(robot->link(i)->collisionShape());
      }

      for(int i=0;i<pairs.size();i++){
        std::shared_ptr<ik_constraint2_bullet::BulletCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletCollisionConstraint>();
        constraint->A_link() = robot->link(pairs[i][0]);
        constraint->B_link() = robot->link(pairs[i][1]);
        constraint->A_link_bulletModel() = constraint->A_link();
        constraint->A_bulletModel().push_back(collisionModels[constraint->A_link()]);
        constraint->B_link_bulletModel() = constraint->B_link();
        constraint->B_bulletModel().push_back(collisionModels[constraint->B_link()]);
        constraint->tolerance() = 0.01;
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        constraints1.push_back(constraint);
      }
    }

    {
      // task: env collision
      std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(3,//size_x
                                                                                                                                                                 3,//size_y
                                                                                                                                                                 3,//size_z
                                                                                                                                                                 0.02,//resolution
                                                                                                                                                                 -1.5,//origin_x
                                                                                                                                                                 -1.5,//origin_y
                                                                                                                                                                 -1.5,//origin_z
                                                                                                                                                                 0.5, // max_distance
                                                                                                                                                                 true// propagate_negative_distances
                                                                                                                                                                 );
      EigenSTL::vector_Vector3d vertices;
      for(int i=0;i<desk->numLinks();i++){
        std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(desk->link(i), 0.02);
        for(int j=0;j<vertices_.size();j++){
          vertices.push_back(desk->link(i)->T() * vertices_[j].cast<double>());
        }
      }
      field->addPointsToField(vertices);
      for(int i=0;i<robot->numLinks();i++){
        std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
        constraint->A_link() = robot->link(i);
        constraint->field() = field;
        constraint->tolerance() = 0.06;
        constraint->minDistance() = 0.02;
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        constraints1.push_back(constraint);
      }

    }



    while(true){

      // reset manip pose
      robot->rootLink()->p() = cnoid::Vector3(0,0,1.0);
      robot->rootLink()->v().setZero();
      robot->rootLink()->R() = cnoid::Matrix3::Identity();
      robot->rootLink()->w().setZero();
      std::vector<double> reset_manip_pose{
        0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// rleg
          0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// lleg
          0.0, 0.0, 0.0, // torso
          0.0, 0.0, // head
          //0.0, 0.959931, -0.349066, -0.261799, -1.74533, -0.436332, 0.0, -0.785398,// rarm
          0.0, -1.50000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,// rarm
          0.0, 0.959931, 0.349066, 0.261799, -1.74533, 0.436332, 0.0, -0.785398,// larm
          0.0, 1.5708, 0.0, 0.0, 0.0, 0.0, // right_hand
          0.0, 1.5708, 0.0, 0.0, 0.0, 0.0 // left_hand
          };

      for(int j=0; j < robot->numJoints(); ++j){
        robot->joint(j)->q() = reset_manip_pose[j];
      }
      robot->calcForwardKinematics();
      robot->calcCenterOfMass();

      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2;
      {
        // task: rleg to current position
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = robot->link("RLEG_JOINT5");
        constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
        constraint->B_link() = nullptr;
        constraint->B_localpos() = constraint->A_link()->T() * constraint->A_localpos();
        constraints2.push_back(constraint);
      }
      {
        // task: lleg to current position
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = robot->link("LLEG_JOINT5");
        constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
        constraint->B_link() = nullptr;
        constraint->B_localpos() = constraint->A_link()->T() * constraint->A_localpos();
        constraints2.push_back(constraint);
      }
      {
        // task: COM to current position
        std::shared_ptr<ik_constraint2::COMConstraint> constraint = std::make_shared<ik_constraint2::COMConstraint>();
        constraint->A_robot() = robot;
        constraint->B_localp() = robot->centerOfMass();
        constraint->weight() << 1.0, 1.0, 0.0; // Z free
        constraints2.push_back(constraint);
      }

      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1, constraints2};

      // setup goals
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goals;
      std::shared_ptr<ik_constraint2::PositionConstraint> goalRaw;
      {
        // task: rarm to target.
        std::shared_ptr<ik_constraint2::PositionConstraint> goal = std::make_shared<ik_constraint2::PositionConstraint>();
        goal->A_link() = robot->link("RARM_JOINT7");
        goal->A_localpos().translation() = cnoid::Vector3(0.0,0.055,-0.217);
        goal->A_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(1.5708, cnoid::Vector3(0,1,0)));
        goal->B_link() = nullptr;
        goal->B_localpos().translation() = cnoid::Vector3(0.55,-0.25,0.45); // below desk
        //goal->maxError() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        goal->precision() = 3e-2;
        goals.push_back(goal);

        goalRaw = goal;
      }

      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
      {
        // task: joint angle to target
        for(int i=0;i<robot->numJoints()-12;i++){ // handの12関節は除く
          std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
          constraint->joint() = robot->joint(i);
          constraint->targetq() = reset_manip_pose[i];
          constraint->precision() = 1e10; // always satisfied
          //constraint->maxError() = 0.1;
          nominals.push_back(constraint);
        }
      }

      std::vector<cnoid::LinkPtr> variables;
      variables.push_back(robot->rootLink());
      for(size_t i=0;i<robot->numJoints()-12;i++){ // handの12関節は除く
        variables.push_back(robot->joint(i));
      }

      // for(size_t i=0;i<constraints.size();i++){
      //   for(size_t j=0;j<constraints[i].size();j++){
      //     constraints[i][j]->debugLevel() = 1;//debug
      //   }
      // }
      global_inverse_kinematics_solver::GIKParam param;
      param.debugLevel=0;
      param.range = 0.5; // 0.2よりも0.3の方が速い. sample一回につきprojectGoalを行うので、rangeはなるべく大きい方がいい.
      param.delta = 0.2; // 大きければ大きいほど速いはずだが、干渉計算や補間の正確さが犠牲になる. 0.2だと正確. 0.4だと速いが薄い障害物をsimplify時に貫通する.
      param.goalBias = 0.2; // 0.05よりも0.2や0.3の方が速い. goalSampingはIKの変位が大きいので、この値が大きいとsample1回あたりの時間が長くなるデメリットもある.
      //param.goalBias = 1.0; // RRTは0.2の方がいい? ESTは0.2の方がいい? KPIECEは
      param.timeout = 30.0;
      param.planner = 0;
      param.useProjection = false;
      param.projectionRange = 0.04; // 0.05だと、collision avoidanceがうまくいかず板を貫通する.
      param.projectionTrapThre = 0.1; // samplerobotは0.01, jaxonは0.03
      param.nearMaxError = 0.05; // 0.05だと安心. 0.2だと薄い障害物を貫通する. weも同時に小さくせよ(1e2だと安心. 1e1でも大丈夫で少し速い). 各constraintのmaxErrorにも注意せよ
      param.projectLink.push_back(goalRaw->A_link());
      param.projectLocalPose = goalRaw->A_localpos();
      param.projectCellSize = 0.2; // 0.05よりも0.1の方が速い. 0.3よりも0.2の方が速い? 2m * 2m * 2mの空間を動くとして、samplingを200個くらいまでにしたければ、cellの大きさもそれなりに大きくないとスカスカになってしまう.
      param.viewer = viewer;
      param.drawLoop = 1;
      param.threads = 20;
      param.pikParam.we = 1e2;
      //param.pikParam.wmax = 1e0;
      param.pikParam.convergeThre = 5e-2; // 2.5e-2は小さすぎる. param.pikParam.debugLevel = 1にして観察せよ. goalのprecision()の値をこれにあわせて大きくせよ
      param.pikParam.debugLevel = 0;
      param.pikParam.pathOutputLoop = 5;
      param.pikParam.satisfiedConvergeLevel = int(constraints.size())-1;
      std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
      bool solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                               constraints,
                                                               goals,
                                                               nominals,
                                                               param,
                                                               path);
      std::cerr << "solved: " << solved << std::endl;

      // optimization
      param.debugLevel = 2;
      param.postpikParam.debugLevel = 1;
      param.postpikParam.maxIteration = 5;
      param.shortcutThre = 0.2; // 0.08:安全(細いものを貫通しない). 0.3:高速.
      global_inverse_kinematics_solver::postProcess(variables,
                                                    constraints,
                                                    path,
                                                    param);

      // main loop
      for(int i=0;i<path->size();i++){
        global_inverse_kinematics_solver::frame2Link(path->at(i),variables);
        robot->calcForwardKinematics(false);
        robot->calcCenterOfMass();

        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints.size();j++){
          for(int k=0;k<constraints[j].size(); k++){
            constraints[j][k]->debugLevel() = 0;
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
        viewer->drawOn(markers);
        viewer->drawObjects();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

}
