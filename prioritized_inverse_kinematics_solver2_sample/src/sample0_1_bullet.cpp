#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/MeshGenerator>
#include <iostream>
#include <ros/package.h>

#include <ik_constraint2_bullet/ik_constraint2_bullet.h>

namespace prioritized_inverse_kinematics_solver2_sample{
  void sample0_1_bullet(){
    cnoid::MeshGenerator meshGenerator;
    cnoid::BodyPtr bigBox = new cnoid::Body();
    {
      cnoid::SgShapePtr shape = new cnoid::SgShape();
      shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,1,1)));
      cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
      material->setTransparency(0.8);
      shape->setMaterial(material);
      cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
      posTransform->translation() = cnoid::Vector3(0,0,0);
      posTransform->addChild(shape);
      cnoid::SgGroupPtr group = new cnoid::SgGroup();
      group->addChild(posTransform);
      cnoid::LinkPtr link = new cnoid::Link();
      link->setShape(group);
      bigBox->setRootLink(link);
    }
    cnoid::BodyPtr smallBox = new cnoid::Body();
    {
      cnoid::SgShapePtr shape = new cnoid::SgShape();
      shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.4,0.4,0.4)));
      cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
      material->setTransparency(0.3);
      shape->setMaterial(material);
      cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
      posTransform->translation() = cnoid::Vector3(0,0,0);
      posTransform->addChild(shape);
      cnoid::SgGroupPtr group = new cnoid::SgGroup();
      group->addChild(posTransform);
      cnoid::LinkPtr link = new cnoid::Link();
      link->setShape(group);
      smallBox->setRootLink(link);
    }

    std::shared_ptr<ik_constraint2_bullet::BulletCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletCollisionConstraint>();
    {
      constraint->A_link() = bigBox->rootLink();
      constraint->B_link() = smallBox->rootLink();
      constraint->tolerance() = 0.01;
      constraint->ignoreDistance() = 1e10; // 描画のため
      constraint->debugLevel() = 2;
      constraint->updateBounds(); // キャッシュを内部に作る.
    }

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(bigBox);
    viewer->objects(smallBox);

    for(int i=0;i<1000;i++) {
      smallBox->rootLink()->p()[0] = 1 * std::sin(i * 0.01 * M_PI);
      smallBox->rootLink()->p()[1] = 1 * std::sin(i * 0.01 * M_PI);
      smallBox->rootLink()->p()[2] = 1 * std::sin(i * 0.01 * M_PI);

      std::vector<cnoid::SgNodePtr> markers;
      {
        constraint->updateBounds();
        const std::vector<cnoid::SgNodePtr>& marker = constraint->getDrawOnObjects();
        std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
      }

      viewer->drawOn(markers);
      viewer->drawObjects();

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

}
