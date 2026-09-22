#include "p4_frontend_diagnostics.h"
#include "p5_seed_basin.h"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  using fast_livo::p4::Association;
  Association a0, a1, b0, b1;
  a0.point_index = b0.point_index = 0;
  a1.point_index = b1.point_index = 1;
  a0.plane_id = b0.plane_id = 10;
  a1.plane_id = 11;
  b1.plane_id = 12;
  a0.voxel_x = b0.voxel_x = 1;
  a1.voxel_x = 2;
  b1.voxel_x = 3;
  a0.normal = b0.normal = Eigen::Vector3d::UnitX();
  a1.normal = b1.normal = Eigen::Vector3d::UnitY();
  a0.residual = 1.0;
  b0.residual = -1.0;
  const auto churn = fast_livo::p4::correspondenceChurn({a0, a1}, {b0, b1});
  assert(churn.retained_count == 2 && churn.changed_plane_count == 1);
  assert(churn.residual_sign_flip_count == 1);
  assert(std::abs(churn.voxel_jaccard - 1.0 / 3.0) < 1e-12);

  const auto signed_metrics = fast_livo::p4::signedResidualMetrics(
      {-2.0, 1.0, 3.0}, {1.0, 2.0, 1.0});
  assert(signed_metrics.count == 3);
  assert(std::abs(signed_metrics.median - 1.0) < 1e-12);
  assert(std::abs(signed_metrics.weighted_mean - 0.75) < 1e-12);

  const auto geometry = fast_livo::p4::normalGeometry(
      {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
       Eigen::Vector3d::UnitZ()});
  assert(geometry.valid);
  assert((geometry.eigenvalues - Eigen::Vector3d::Constant(1.0 / 3.0)).norm() <
         1e-12);
  assert(std::abs(geometry.directional_entropy - 1.0) < 1e-12);

  assert(fast_livo::p4::ageBin(0.49) == 0);
  assert(fast_livo::p4::ageBin(0.5) == 1);
  assert(fast_livo::p4::ageBin(20.0) == 4);
  assert(fast_livo::p4::mapInsertionAllowed(false, true, 10.0, 5.0));
  assert(!fast_livo::p4::mapInsertionAllowed(true, true, 10.0, 5.0));
  assert(fast_livo::p4::mapInsertionAllowed(true, false, 10.0, 5.0));

  const auto reuse = fast_livo::p4::recentReuseRatios({9, 7, 5, -1}, 10);
  assert(std::abs(reuse[0] - 0.25) < 1e-12);
  assert(std::abs(reuse[1] - 0.50) < 1e-12);
  assert(std::abs(reuse[2] - 0.75) < 1e-12);

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 6);
  jacobian.diagonal().setOnes();
  Eigen::VectorXd residual(6);
  residual << 1, 2, 3, 4, 5, 6;
  const auto fixed = fast_livo::p4::solveFixedCorrespondences(
      jacobian, residual, Eigen::VectorXd::Ones(6));
  assert(fixed.valid && fixed.rank == 6);
  assert((fixed.delta + residual).norm() < 1e-12);

  const double weak_projection =
      fast_livo::p4::signedResidualWeakProjection(
          {1.0, -1.0}, {Eigen::Vector3d::UnitX(),
                        -Eigen::Vector3d::UnitX()},
          Eigen::Vector3d::UnitX());
  assert(std::abs(weak_projection - 1.0) < 1e-12);

  Eigen::Matrix<double, 6, 1> pose_delta =
      Eigen::Matrix<double, 6, 1>::Zero();
  pose_delta[3] = 2.0;
  assert(std::abs(fast_livo::p5::posePriorMahalanobis(
                      pose_delta,
                      2.0 * Eigen::Matrix<double, 6, 6>::Identity()) -
                  2.0) < 1e-12);
  assert(fast_livo::p5::kDiagnosticSeedAlphas.size() == 5);
  assert(fast_livo::p5::kProfileAlphas.size() == 9);
  assert(std::abs(fast_livo::p5::associationJaccard({a0, a1}, {b0, b1}) -
                  1.0 / 3.0) < 1e-12);
  assert(fast_livo::p5::selectJointMapCandidate(
             {{10.0, 8.0, 6.0}}, {{0.0, 1.0, 5.0}},
             {{true, true, true}}) == 1);
  assert(fast_livo::p5::selectJointMapCandidate(
             {{10.0, 1.0, 6.0}}, {{0.0, 0.0, 0.0}},
             {{true, false, true}}) == 2);

  std::cout << "p4_frontend_diagnostics_self_test: PASS\n";
  return 0;
}
