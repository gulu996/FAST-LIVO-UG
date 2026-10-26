#include "voxel_map.h"
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
  std::cout << "PASS: plane and tangential-distance checks\n";
}
