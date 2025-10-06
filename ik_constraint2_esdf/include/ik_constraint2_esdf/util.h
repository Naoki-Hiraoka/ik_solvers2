#ifndef IK_CONSTRAINT2_ESDF_UTIL_H
#define IK_CONSTRAINT2_ESDF_UTIL_H

#include <vector>
#include <cnoid/Body>

namespace ik_constraint2_esdf{

  // link local frame
  std::vector<cnoid::Vector3> getSurfaceVertices(cnoid::LinkPtr link, float resolution = 0.02);

  std::vector<std::pair<cnoid::Vector3, cnoid::Vector3> > getSurfaceVerticesAndNormals(cnoid::LinkPtr link, float resolution = 0.02, float minangle = M_PI/3);

};

#endif
