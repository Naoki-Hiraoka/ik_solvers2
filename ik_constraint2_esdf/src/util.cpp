#include <ik_constraint2_esdf/util.h>
#include <iostream>
#include <cnoid/MeshExtractor>
#include <cnoid/MeshFilter>
#include <cnoid/SceneDrawables>

namespace ik_constraint2_esdf{

  std::vector<cnoid::Vector3> getSurfaceVertices(cnoid::LinkPtr link, float resolution){
    // 1つのvertexを取得したら、resolutionのサイズの同じ立方体の中にある他のvertexは取得しない
    // faceが巨大な場合、faceの内部の点をresolutionの間隔でサンプリングして取得する

    cnoid::MeshExtractor meshExtractor;

    std::vector<cnoid::Vector3> vertices;
    cnoid::SgMeshPtr mesh = meshExtractor.integrate(link->collisionShape());
    if(mesh && (mesh->numTriangles() != 0)) {
      mesh->updateBoundingBox();
      cnoid::BoundingBoxf bbx = mesh->boundingBox();
      cnoid::Vector3f bbxSize = bbx.max() - bbx.min();
      std::vector<std::vector<std::vector<bool> > > bin;
      bin.resize(int(bbxSize[0]/resolution)+1);
      for(int x=0;x<bin.size();x++){
        bin[x].resize(int(bbxSize[1]/resolution)+1);
        for(int y=0;y<bin[x].size();y++){
          bin[x][y].resize(int(bbxSize[2]/resolution)+1,false);
        }
      }

      for(int j=0;j<mesh->numTriangles();j++){
        cnoid::Vector3f v0 = mesh->vertices()->at(mesh->triangle(j)[0]);
        cnoid::Vector3f v1 = mesh->vertices()->at(mesh->triangle(j)[1]);
        cnoid::Vector3f v2 = mesh->vertices()->at(mesh->triangle(j)[2]);
        float l1 = (v1 - v0).norm();
        float l2 = (v2 - v0).norm();
        cnoid::Vector3f d1 = (v1 - v0).normalized();
        cnoid::Vector3f d2 = (v2 - v0).normalized();

        for(float m=0;;){
          float n_max = (l1==0)? l2 : l2*(1-m/l1);
          for(float n=0;;){
            cnoid::Vector3f v = v0 + d1 * m + d2 * n;
            int x = int((v[0] - bbx.min()[0])/resolution);
            int y = int((v[1] - bbx.min()[1])/resolution);
            int z = int((v[2] - bbx.min()[2])/resolution);
            if(!bin[x][y][z]){
              bin[x][y][z] = true;
              vertices.push_back(v.cast<double>());
            }

            if(n>= n_max) break;
            else n = std::min(n+resolution, n_max);
          }

          if(m>=l1) break;
          else m = std::min(m+resolution, l1);
        }
      }
    }
    return vertices;
  }

  std::vector<std::pair<cnoid::Vector3, cnoid::Vector3> > getSurfaceVerticesAndNormals(cnoid::LinkPtr link, float resolution, float minangle) {
    // 1つのvertexを取得したら、resolutionのサイズの同じ立方体の中にありかつ法線がminangle以下の他のvertexは取得しない
    // faceが巨大な場合、faceの内部の点をresolutionの間隔でサンプリングして取得する

    cnoid::MeshExtractor meshExtractor;
    cnoid::MeshFilter meshFilter;

    std::vector<std::pair<cnoid::Vector3, cnoid::Vector3> > vertices;
    cnoid::SgMeshPtr mesh = meshExtractor.integrate(link->collisionShape());
    if(mesh && (mesh->numTriangles() != 0)) {
      meshFilter.generateNormals(mesh,M_PI,true);
      mesh->updateBoundingBox();
      cnoid::BoundingBoxf bbx = mesh->boundingBox();
      cnoid::Vector3f bbxSize = bbx.max() - bbx.min();
      std::vector<std::vector<std::vector<std::vector<cnoid::Vector3f> > > > bin; // normalを入れる
      bin.resize(int(bbxSize[0]/resolution)+1);
      for(int x=0;x<bin.size();x++){
        bin[x].resize(int(bbxSize[1]/resolution)+1);
        for(int y=0;y<bin[x].size();y++){
          bin[x][y].resize(int(bbxSize[2]/resolution)+1);
        }
      }

      for(int j=0;j<mesh->numTriangles();j++){
        cnoid::Vector3f v0 = mesh->vertices()->at(mesh->triangle(j)[0]);
        cnoid::Vector3f v1 = mesh->vertices()->at(mesh->triangle(j)[1]);
        cnoid::Vector3f v2 = mesh->vertices()->at(mesh->triangle(j)[2]);
        cnoid::Vector3f normal = mesh->normals()->at(mesh->normalIndices()[j*3]); // linkの外側に向かう方向
        float l1 = (v1 - v0).norm();
        float l2 = (v2 - v0).norm();
        cnoid::Vector3f d1 = (v1 - v0).normalized();
        cnoid::Vector3f d2 = (v2 - v0).normalized();

        for(float m=0;;){
          float n_max = (l1==0)? l2 : l2*(1-m/l1);
          for(float n=0;;){
            cnoid::Vector3f v = v0 + d1 * m + d2 * n;
            int x = int((v[0] - bbx.min()[0])/resolution);
            int y = int((v[1] - bbx.min()[1])/resolution);
            int z = int((v[2] - bbx.min()[2])/resolution);

            bool exists = false;
            for(int s=0;s<bin[x][y][z].size();s++){
              if(minangle >= std::acos(std::min(1.0f,(std::max(-1.0f,normal.dot(bin[x][y][z][s])))))){
                exists = true;
                break;
              }
            }
            if(!exists){
              bin[x][y][z].push_back(normal);
              vertices.emplace_back(v.cast<double>(), normal.cast<double>());
            }

            if(n>= n_max) break;
            else n = std::min(n+resolution, n_max);
          }

          if(m>=l1) break;
          else m = std::min(m+resolution, l1);
        }
      }
    }
    return vertices;
  }
}
