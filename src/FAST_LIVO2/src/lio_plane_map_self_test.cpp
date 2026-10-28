#include "vio.h"
#include <vikit/pinhole_camera.h>
#include <cassert>
#include <iostream>


// Observation maintenance is independent of normal convergence and reference selection.
static void checkObservationMaintenance(bool converged)
{
  vk::PinholeCamera camera(320, 240, 1, 100, 100, 160, 120, 0, 0, 0, 0);
  VIOManager manager;
  StatesGroup state;
  const StatesGroup before = state;
  manager.state = &state; manager.cam = &camera; manager.total_points = 1;
  manager.patch_size = 8; manager.patch_size_half = 4; manager.patch_size_total = 64;
  manager.lidar_map_voxel_size = .8; manager.visual_patch_quality_gate_en = false;
  manager.console_timing_print_en = false; manager.visual_submap = new SubSparseMap;
  manager.visual_submap->search_levels = {0}; manager.update_flag = {0};
  cv::Mat image(240, 320, CV_8UC1);
  for (int y = 0; y < image.rows; ++y)
    for (int x = 0; x < image.cols; ++x) image.at<unsigned char>(y, x) = 40 + (x * 7 + y * 3) % 160;
  manager.new_frame_.reset(new Frame(&camera, image));
  VisualPoint point(V3D(0, 0, 5));
  point.normal_ = point.previous_normal_ = V3D(0, 0, 1);
  point.is_normal_initialized_ = true; point.is_converged_ = converged;
  point.covariance_ = M3D::Identity() * .001;
  manager.visual_submap->voxel_points.push_back(&point);
  float *patch = new float[64]; manager.getImagePatch(image, V2D(160, 120), patch, 0);
  Feature *initial = new Feature(&point, patch, V2D(160, 120), V3D(0, 0, 1), SE3(), 0);
  initial->img_ = image; initial->id_ = 0; point.addFrameRef(initial);
  point.ref_patch = initial; point.has_ref_patch_ = true;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> empty;
  for (int i = 1; i <= 60; ++i)
  {
    manager.new_frame_->id_ = 2 * i;
    manager.new_frame_->T_f_w_ = SE3(M3D::Identity(), V3D(i % 2 ? 1 : 0, 0, 0));
    manager.update_flag[0] = 0; manager.updateVisualMapPoints(image);
    assert(manager.update_flag[0] == 1 && point.obs_.front()->id_ == 2 * i);
    assert(point.obs_.size() > 1 && point.obs_.size() <= 30);
    assert(point.ref_patch != nullptr); // pruning alone cannot clear the active reference
    if (i == 1) { point.ref_patch = nullptr; point.has_ref_patch_ = false; }
    manager.updateReferencePatch(empty);
    assert(point.ref_patch && point.has_ref_patch_);
    const auto history = point.obs_;
    Feature *reference = point.ref_patch;
    manager.new_frame_->id_++; manager.update_flag[0] = 0;
    manager.updateVisualMapPoints(image); // same pose: do not compare against the oldest view
    assert(manager.update_flag[0] == 0 && point.obs_ == history && point.ref_patch == reference);
    manager.updateReferencePatch(empty);
    assert(point.ref_patch == reference);
    for (const Feature *f : point.obs_)
      assert((camera.world2cam(f->T_f_w_ * point.pos_) - f->px_).norm() < 1e-6);
  }
  assert(point.obs_.size() == 30);
  const auto history = point.obs_; Feature *reference = point.ref_patch;
  manager.visual_patch_quality_gate_en = true;
  manager.image_quality_saturated_pixel_value = 250;
  manager.visual_patch_max_saturated_fraction = .2;
  manager.visual_patch_min_intensity_std = 1;
  manager.new_frame_->T_f_w_ = SE3(M3D::Identity(), V3D(1, 0, 0));
  manager.update_flag[0] = 0;
  manager.updateVisualMapPoints(cv::Mat(240, 320, CV_8UC1, cv::Scalar(255)));
  assert(manager.update_flag[0] == 0 && point.obs_ == history && point.ref_patch == reference);
  point.ref_patch->score_ = -.5; Feature *prune = nullptr;
  point.findMinScoreFeature(V3D::Zero(), prune);
  assert(prune && prune != point.ref_patch);
  assert((state - before).norm() == 0 && (state.cov - before.cov).norm() == 0);
  assert(point.is_converged_ == converged && (point.normal_ - V3D(0, 0, 1)).norm() == 0);
}

int main()
{
  checkObservationMaintenance(false);
  checkObservationMaintenance(true);
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
      V point(double(i) * size, .13, -.17);
      assert(storedLidarVoxelLocation(point, size, key));
      assert(key.x == int64_t(std::floor(point.x() / size)));
    }
  // Exact negative cell boundaries must stay in their own cell (floor), not
  // move one cell down. Whole-cell translations must shift the key equally.
  for (double size : {.25, .5, 1., 2.}) {
    for (int i = -4; i <= 4; ++i) {
      const V point(double(i) * size, 0.0, 0.0);
      assert(storedLidarVoxelLocation(point, size, key));
      assert(key.x == i);
      VOXEL_LOCATION shifted_key;
      assert(storedLidarVoxelLocation(point + V(3.0 * size, 0, 0), size,
                                      shifted_key));
      assert(shifted_key.x == key.x + 3);
      assert(storedLidarVoxelLocation(
          point - V(1e-6 * size, 0, 0), size, shifted_key));
      assert(shifted_key.x == key.x - 1);
    }
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
