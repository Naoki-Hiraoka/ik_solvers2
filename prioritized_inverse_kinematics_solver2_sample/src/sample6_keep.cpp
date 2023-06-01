#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/MeshGenerator>
#include <iostream>
#include <ros/package.h>

#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>
#include <ik_constraint2/ik_constraint2.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>

namespace prioritized_inverse_kinematics_solver2_sample{
  void sample6_keep(){
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

    cnoid::MeshGenerator meshGenerator;
    cnoid::BodyPtr ground = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.4,0.4,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0.5);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0,-0.05);
          posTransform->addChild(shape);
          group->addChild(posTransform);
        }
        rootLink->setShape(group);
      }
      ground->setRootLink(rootLink);
    }


    // setup tasks
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    // joint limit
    for(int i=0;i<robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = robot->joint(i);
      constraints0.push_back(constraint);
    }
    {
      // task: self collision
      std::vector<std::vector<std::string> > pairs{
        std::vector<std::string>{"LARM_SHOULDER_R","WAIST"},
        std::vector<std::string>{"LARM_ELBOW","WAIST"},
        std::vector<std::string>{"LARM_WRIST_R","WAIST"},
        std::vector<std::string>{"RARM_SHOULDER_R","WAIST"},
        std::vector<std::string>{"RARM_ELBOW","WAIST"},
        std::vector<std::string>{"RARM_WRIST_R","WAIST"},
        std::vector<std::string>{"LARM_SHOULDER_R","WAIST_R"},
        std::vector<std::string>{"LARM_ELBOW","WAIST_R"},
        std::vector<std::string>{"LARM_WRIST_R","WAIST_R"},
        std::vector<std::string>{"RARM_SHOULDER_R","WAIST_R"},
        std::vector<std::string>{"RARM_ELBOW","WAIST_R"},
        std::vector<std::string>{"RARM_WRIST_R","WAIST_R"},
        std::vector<std::string>{"LARM_SHOULDER_R","CHEST"},
        std::vector<std::string>{"LARM_ELBOW","CHEST"},
        std::vector<std::string>{"LARM_WRIST_R","CHEST"},
        std::vector<std::string>{"RARM_SHOULDER_R","CHEST"},
        std::vector<std::string>{"RARM_ELBOW","CHEST"},
        std::vector<std::string>{"RARM_WRIST_R","CHEST"}
      };
      for(int i=0;i<pairs.size();i++){
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = robot->link(pairs[i][0]);
        constraint->B_link() = robot->link(pairs[i][1]);
        constraint->tolerance() = 0.03;
        constraints0.push_back(constraint);
      }
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
    {
      // task: rleg to ground
      std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
      constraint->A_link() = robot->link("RLEG_ANKLE_R");
      constraint->B_link() = ground->rootLink();
      constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
      choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                 constraint->A_FACE_C()[0],
                                                 constraint->A_FACE_dl()[0],
                                                 constraint->A_FACE_du()[0]);
      constraint->useSingleMeshB() = false;
      choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                  constraint->B_FACE_C(),
                                                  constraint->B_FACE_dl(),
                                                  constraint->B_FACE_du());
      //constraint->debugLevel() = 2;
      constraints1.push_back(constraint);
    }
    {
      // task: lleg to ground
      std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
      constraint->A_link() = robot->link("LLEG_ANKLE_R");
      constraint->B_link() = ground->rootLink();
      constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
      choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                 constraint->A_FACE_C()[0],
                                                 constraint->A_FACE_dl()[0],
                                                 constraint->A_FACE_du()[0]);
      constraint->useSingleMeshB() = false;
      choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                  constraint->B_FACE_C(),
                                                  constraint->B_FACE_dl(),
                                                  constraint->B_FACE_du());
      //constraint->debugLevel() = 2;
      constraints1.push_back(constraint);
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2;
    {
      // task: rarm to target. never reach
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("RARM_WRIST_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(1.0,-0.2,1.0);
      for(size_t i=0;i<3;i++)constraint->weight()[3+i] = 0.0;
      constraints2.push_back(constraint);
    }
    {
      // task: larm to target. rotation-axis nil
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("LARM_WRIST_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.3,0.2,1.0);
      for(size_t i=0;i<3;i++)constraint->weight()[3+i] = 0.0;
      constraints2.push_back(constraint);
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints3;
    {
      // task: joint angle to target
      std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
      constraint->joint() = robot->link("CHEST");
      constraint->targetq() = 0.1;
      constraints3.push_back(constraint);
    }


    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(robot->rootLink());
    for(size_t i=0;i<robot->numJoints();i++){
      variables.push_back(robot->joint(i));
    }
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1,constraints2,constraints3};
    for(size_t i=0;i<constraints.size();i++){
      for(size_t j=0;j<constraints[i].size();j++){
        //constraints[i][j]->debugLevel() = 0;//debug
      }
    }

    // setup viewer
    choreonoid_viewer::Viewer viewer;
    viewer.objects(robot);
    viewer.objects(ground);

    // main loop
    for(int i=0;i<200;i++){
      prioritized_inverse_kinematics_solver2::IKParam param;
      //param.debugLevel = 2; // debug
      param.maxIteration = 1;
      param.we = 1e2;
      bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                        constraints,
                                                                        tasks,
                                                                        param);

      if(solved || i % 10 == 0){
        std::cerr << "loop: " << i << std::endl;
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints.size();j++){
          for(int k=0;k<constraints[j].size(); k++){
            const std::vector<cnoid::SgNodePtr>& marker = constraints[j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        viewer.drawOn(markers);
        viewer.drawObjects();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      if(solved) break;
    }

    for(size_t i=0;i<constraints.size();i++){
      for(size_t j=0;j<constraints[i].size();j++){
        constraints[i][j]->debugLevel() = 0;//not debug
        constraints[i][j]->updateBounds();
        if(constraints[i][j]->isSatisfied()) std::cerr << "constraint " << i << " " << j << ": satisfied"<< std::endl;
        else std::cerr << "constraint " << i << " " << j << ": NOT satidfied"<< std::endl;
      }
    }

    return;
  }
}
