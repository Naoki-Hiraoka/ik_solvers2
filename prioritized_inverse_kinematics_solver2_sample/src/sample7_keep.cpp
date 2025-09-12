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
  void sample7_keep(){
    cnoid::MeshGenerator meshGenerator;
    meshGenerator.setDivisionNumber(8); // default 20. 20だとcddlibが遅い

    cnoid::BodyPtr candidate = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        for(int i=-10;i<=10;i++){
          for(int j=-10;j<=10;j++){
            cnoid::SgShapePtr shape = new cnoid::SgShape();
            shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.01,0.01,0.01)));
            cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
            material->setTransparency(0);
            material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
            shape->setMaterial(material);
            cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
            posTransform->translation() = cnoid::Vector3(0.1*i,0.1*j,0);
            posTransform->addChild(shape);
            rootLink->addShapeNode(posTransform);
          }
        }
      }
      candidate->setRootLink(rootLink);
      candidate->calcForwardKinematics();
    }


    cnoid::BodyPtr region = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateSphere(0.55/*radius*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.5);
        material->setDiffuseColor(cnoid::Vector3f(0.3, 0.3, 0.3));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,0,0.0);
        posTransform->addChild(shape);
        rootLink->addShapeNode(posTransform);
      }
      rootLink->setJointType(cnoid::Link::FreeJoint);
      region->setRootLink(rootLink);
      region->calcForwardKinematics();
    }

    std::shared_ptr<ik_constraint2::PointKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2::PointKeepCollisionConstraint>();
    constraint->A_link() = region->rootLink();
    constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
    choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                               constraint->A_FACE_C()[0],
                                               constraint->A_FACE_dl()[0],
                                               constraint->A_FACE_du()[0]);
    constraint->B_link() = candidate->rootLink();
    for(int i=-10;i<=10;i++){
      for(int j=-10;j<=10;j++){
        constraint->B_POINT().push_back(cnoid::Vector3(0.1*i,0.1*j,0));
      }
    }
    constraint->shrinkA() = 0.1;
    constraint->ignorePenetration() = 1e10;
    //constraint->debugLevel() = 2;



    // task: move target
    std::shared_ptr<ik_constraint2::PositionConstraint> moveconstraint = std::make_shared<ik_constraint2::PositionConstraint>();
    moveconstraint->A_link() = region->rootLink();
    moveconstraint->B_link() = nullptr;
    //moveconstraint->debugLevel() = 2;

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    //viewer->objects(robot);
    viewer->objects(candidate);
    viewer->objects(region);

    viewer->drawObjects();

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
    std::vector<cnoid::LinkPtr> variables{region->rootLink()};
    prioritized_inverse_kinematics_solver2::IKParam param;
    // param.debugLevel = 2;
    param.we = 1e2;

    int i = 0;
    while(true){
      moveconstraint->B_localpos().translation()[0] = std::sin(i / 200.0);
      moveconstraint->B_localpos().translation()[1] = std::sin(i / 300.0);
      moveconstraint->B_localpos().translation()[2] = std::sin(i / 100.0);

      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{
        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >{constraint},
        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >{moveconstraint}};
      bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                        constraints,
                                                                        tasks,
                                                                        param);
      std::vector<cnoid::SgNodePtr> markers = moveconstraint->getDrawOnObjects();
      viewer->drawOn(markers);
      viewer->drawObjects();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      i++;
    }

  }
}
