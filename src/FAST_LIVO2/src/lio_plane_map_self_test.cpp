#include "vio.h"
#include <vikit/pinhole_camera.h>
#include <cassert>
#include <iostream>

int main()
{
  using V = Eigen::Vector3d;
  std::vector<pointWithVar> points(6);
  for (int i=0;i<6;++i) {points[i].point_w=V(i,0,0);points[i].var=Eigen::Matrix3d::Identity()*.001;}
  VoxelOctoTree node(0,0,5,50,.005);
  node.init_plane(points,node.plane_ptr_); assert(!node.plane_ptr_->is_plane_);
  for(auto &p:points)p.point_w.setZero();
  node.init_plane(points,node.plane_ptr_); assert(!node.plane_ptr_->is_plane_);
  for(int i=0;i<6;++i)points[i].point_w=V(i%3,i/3,.7);
  node.init_plane(points,node.plane_ptr_);
  assert(node.plane_ptr_->is_plane_ && node.plane_ptr_->plane_var_.allFinite());
  for(int i=0;i<10000;++i)assert(planeTangentialDistanceSquared(V(0,0,.000001*i),V(0,0,1))==0);
  assert(!std::isfinite(planeTangentialDistanceSquared(V::Zero(),V::Zero())));
  VOXEL_LOCATION key;
  assert(!storedLidarVoxelLocation(V::Zero(), 0, key));
  assert(!storedLidarVoxelLocation(V::Constant(INFINITY), .8, key));
  for (double size : {.25, .5, .8, 1., 2.})
    for (int i = -32; i <= 32; ++i) {
      V point(i * static_cast<float>(size), .13, -.17);
      assert(storedLidarVoxelLocation(point, size, key));
      float x = point.x() / static_cast<float>(size); if (x < 0) x -= 1;
      assert(key.x == int64_t(x));
    }
 vk::PinholeCamera camera(1280,720,1,529.938228576,532.081773898,634.696805930,357.311987738,-.091446989773,.071009135497,0,0);
 VIOManager manager;manager.patch_size_half=4;
 const V2D center(634.696805930,357.311987738);const V3D p(0,0,5),n(0,0,1);const SE3 identity;Matrix2d a=Matrix2d::Identity();
 auto valid=[&](V2D px,V3D p,V3D n,SE3 t,Matrix2d a,int level=0){return VIOManager::planeWarpFootprintVisible(camera,px,p,n,t,a,level,3,4);};
 assert(valid(center,p,n,identity,a));assert(valid(center,p,-n,identity,a));
 assert(!valid(center,-p,n,identity,a));assert(!valid(center,p,V3D::Zero(),identity,a));
 assert(!valid(center,p,n,SE3(Eigen::Matrix3d::Identity(),V3D(0,0,-10)),a));
 assert(!valid(center,p,n,identity,Matrix2d::Zero()));
 assert(!valid(center,p,n,identity,Matrix2d::Constant(NAN)));
 V3D grazing(0,1,.001);manager.getWarpMatrixAffineHomography(camera,center,p,grazing,identity,0,a);
 assert(a.allFinite() && std::abs(a.determinant())>.9);assert(!valid(center,p,grazing,identity,a));
 // Real 2.611882-second reference geometry. Even identity pose has behind-camera support.
 V2D px(778.58,360.79);V3D rp(2.721793980536901,.06546584103090475,9.960119520086891),rn(-.013868681867255733,.9999013468508251,.0022262590080051594);
 manager.getWarpMatrixAffineHomography(camera,px,rp,rn,identity,0,a);assert(a.allFinite());assert(!valid(px,rp,rn,identity,a));
 for(int i=0;i<1000;++i){double v=.0001*i;SE3 t(Eigen::AngleAxisd(v,V3D::UnitZ()).toRotationMatrix(),V3D(v,0,.1));manager.getWarpMatrixAffineHomography(camera,center,p,n,t,0,a);assert(valid(center,p,n,t,a));assert(valid(center,p,-n,t,a));}
  std::cout << "PASS: plane, distance, stored-map index and positive-depth footprint checks\n";
}
