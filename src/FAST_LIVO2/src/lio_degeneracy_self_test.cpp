#include "lio_degeneracy.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{

using Vector6d = Eigen::Matrix<double, 6, 1>;

void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

fast_livo::Matrix6d blockDiagonalInformation(
    const Eigen::Vector3d &rotation_eigenvalues,
    const Eigen::Vector3d &translation_eigenvalues,
    const Eigen::Matrix3d &translation_directions = Eigen::Matrix3d::Identity())
{
  fast_livo::Matrix6d information = fast_livo::Matrix6d::Zero();
  information.block<3, 3>(0, 0) = rotation_eigenvalues.asDiagonal();
  information.block<3, 3>(3, 3) =
      translation_directions * translation_eigenvalues.asDiagonal() *
      translation_directions.transpose();
  return information;
}

void checkNormalPositiveDefiniteAndConditionalAnalysis()
{
  const Eigen::AngleAxisd rotation(0.63, Eigen::Vector3d(1.0, 2.0, -0.5).normalized());
  const Eigen::Matrix3d directions = rotation.toRotationMatrix();
  const Eigen::Vector3d expected_eigenvalues(1.0, 5.0, 20.0);
  const Eigen::Matrix3d expected =
      directions * expected_eigenvalues.asDiagonal() * directions.transpose();
  const Eigen::Matrix3d h_rr = Eigen::Vector3d(80.0, 100.0, 120.0).asDiagonal();
  Eigen::Matrix3d h_rt;
  h_rt << 2.0, -1.0, 0.5,
          -0.3, 1.2, 0.7,
           0.8, 0.2, -1.4;

  fast_livo::Matrix6d information = fast_livo::Matrix6d::Zero();
  information.block<3, 3>(0, 0) = h_rr;
  information.block<3, 3>(0, 3) = h_rt;
  information.block<3, 3>(3, 0) = h_rt.transpose();
  information.block<3, 3>(3, 3) =
      expected + h_rt.transpose() * h_rr.ldlt().solve(h_rt);

  const auto metrics = fast_livo::analyzeLioPoseInformation(information, 1e-12, true);
  require(metrics.valid, "normal positive-definite information must be valid");
  require((metrics.translation_eigenvalues - expected_eigenvalues).norm() < 1e-8,
          "conditional translation eigenvalues are wrong");
  require(std::fabs(metrics.weak_translation_direction_world.dot(directions.col(0))) >
              1.0 - 1e-8,
          "weak direction must follow geometry, not a fixed world axis");
  require(metrics.translation_eigenvalues[0] <= metrics.translation_eigenvalues[1] &&
              metrics.translation_eigenvalues[1] <= metrics.translation_eigenvalues[2],
          "translation eigenvalues must be sorted ascending");
  require(std::fabs(metrics.weak_translation_direction_world.norm() - 1.0) < 1e-12,
          "weak translation direction must have unit length");
  require(std::fabs(metrics.weak_rotation_direction_body.norm() - 1.0) < 1e-12,
          "weak rotation direction must have unit length");
  require(std::fabs(metrics.rotation_eigenvalue_ratio - 80.0 / 120.0) < 1e-12,
          "rotation eigenvalue ratio is wrong");
}

void checkWeakTranslationAxes()
{
  const auto one_weak = fast_livo::analyzeLioPoseInformation(
      blockDiagonalInformation(Eigen::Vector3d(10.0, 20.0, 30.0),
                               Eigen::Vector3d(1e-6, 2.0, 8.0)),
      1e-6, true);
  require(one_weak.valid && one_weak.translation_eigenvalues[0] < 1e-5 &&
              one_weak.translation_eigenvalues[1] > 1.0,
          "single weak translation axis was not diagnosed");

  const auto two_weak = fast_livo::analyzeLioPoseInformation(
      blockDiagonalInformation(Eigen::Vector3d(10.0, 20.0, 30.0),
                               Eigen::Vector3d(1e-9, 1e-7, 5.0)),
      1e-6, true);
  require(two_weak.valid && two_weak.translation_eigenvalues[1] < 1e-6 &&
              two_weak.translation_eigenvalues[2] > 1.0,
          "two weak translation axes were not diagnosed");
}

void checkSingularRotationBlocks()
{
  const auto near_singular = fast_livo::analyzeLioPoseInformation(
      blockDiagonalInformation(Eigen::Vector3d(1e-14, 1.0, 10.0),
                               Eigen::Vector3d(1.0, 2.0, 3.0)),
      1e-6, true);
  require(near_singular.valid, "regularized near-singular rotation block must be safe");

  const auto singular = fast_livo::analyzeLioPoseInformation(
      blockDiagonalInformation(Eigen::Vector3d::Zero(),
                               Eigen::Vector3d(1.0, 2.0, 3.0)),
      1e-6, true);
  require(singular.valid && singular.rotation_eigenvalues.isZero(1e-15),
          "regularized fully singular rotation block must be safe");
}

void checkInvalidInputsReturnSafely()
{
  const fast_livo::Matrix6d normal = blockDiagonalInformation(
      Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(4.0, 5.0, 6.0));

  fast_livo::Matrix6d nan_information = normal;
  nan_information(0, 0) = std::numeric_limits<double>::quiet_NaN();
  require(!fast_livo::analyzeLioPoseInformation(nan_information, 1e-6, true).valid,
          "NaN information must return invalid");

  fast_livo::Matrix6d inf_information = normal;
  inf_information(4, 4) = std::numeric_limits<double>::infinity();
  require(!fast_livo::analyzeLioPoseInformation(inf_information, 1e-6, true).valid,
          "Inf information must return invalid");

  fast_livo::Matrix6d indefinite_information = normal;
  indefinite_information(3, 3) = -1.0;
  require(!fast_livo::analyzeLioPoseInformation(indefinite_information, 1e-6, true).valid,
          "non-positive-semidefinite information must return invalid");
}

void checkExtremeConditionNumber()
{
  const auto metrics = fast_livo::analyzeLioPoseInformation(
      blockDiagonalInformation(Eigen::Vector3d(1.0, 2.0, 3.0),
                               Eigen::Vector3d(1e-15, 1.0, 1e12)),
      1e-6, false);
  require(metrics.valid, "finite extremely conditioned information must remain diagnosable");
  require(std::isinf(metrics.translation_condition_number),
          "near-zero weak eigenvalue must report infinite condition number");
}

void checkDiagnosticModeDoesNotMutateInputs()
{
  fast_livo::Matrix6d information = blockDiagonalInformation(
      Eigen::Vector3d(3.0, 4.0, 5.0), Eigen::Vector3d(1e-6, 2.0, 9.0));
  Vector6d rhs;
  rhs << 0.5, -0.2, 0.7, 1.0, -2.0, 0.3;
  Vector6d state;
  state << 1.0, 2.0, 3.0, -1.0, -2.0, -3.0;
  fast_livo::Matrix6d covariance = fast_livo::Matrix6d::Identity() * 0.25;
  const fast_livo::Matrix6d information_before = information;
  const Vector6d rhs_before = rhs;
  const Vector6d state_before = state;
  const fast_livo::Matrix6d covariance_before = covariance;

  const auto metrics = fast_livo::analyzeLioPoseInformation(information, 1e-6, true);
  require(metrics.valid, "diagnostic-only fixture must be valid");
  require(information == information_before, "diagnostics modified the input information");
  require(rhs == rhs_before, "diagnostics modified the RHS");
  require(state == state_before, "diagnostics modified the state");
  require(covariance == covariance_before, "diagnostics modified the covariance");
}

void checkRelativeDirectionWeights()
{
  const Eigen::Vector3d eigenvalues(0.01, 0.05, 1.0);
  const Eigen::Vector3d weights =
      fast_livo::lioRelativeEigenDirectionWeights(eigenvalues, 0.10);
  require(weights[0] == 0.0, "ratio below one-quarter threshold must be fully suppressed");
  require(std::fabs(weights[1] - std::sqrt(0.5)) < 1e-12,
          "partial direction weight does not match the established observability formula");
  require(weights[2] == 1.0, "strong direction must remain unchanged");
}

void checkDirectionalProjectorNormalEquation()
{
  const fast_livo::Matrix6d information = blockDiagonalInformation(
      Eigen::Vector3d(1.0, 4.0, 9.0), Eigen::Vector3d(0.01, 2.0, 8.0));
  Vector6d rhs;
  rhs << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  const auto metrics = fast_livo::analyzeLioPoseInformation(information, 1e-6, true);
  require(metrics.valid, "directional projector fixture must be observable");

  const auto unchanged = fast_livo::applyLioDirectionalProjectors(
      information, rhs, metrics, Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
  require(unchanged.information == information,
          "all-one directional projector must preserve information exactly");
  require(unchanged.rhs == rhs, "all-one directional projector must preserve RHS exactly");

  Eigen::Vector3d translation_weights = Eigen::Vector3d::Ones();
  translation_weights[0] = 0.0;
  const auto suppressed = fast_livo::applyLioDirectionalProjectors(
      information, rhs, metrics, Eigen::Vector3d::Ones(), translation_weights);
  require(std::fabs(suppressed.information(3, 3)) < 1e-12,
          "fully suppressed translation direction retained information");
  require(std::fabs(suppressed.rhs[3]) < 1e-12,
          "fully suppressed translation direction retained RHS");
  require(std::fabs(suppressed.information(4, 4) - information(4, 4)) < 1e-12 &&
              std::fabs(suppressed.information(5, 5) - information(5, 5)) < 1e-12,
          "strong translation directions changed unexpectedly");
}

} // namespace

int main()
{
  try
  {
    checkNormalPositiveDefiniteAndConditionalAnalysis();
    checkWeakTranslationAxes();
    checkSingularRotationBlocks();
    checkInvalidInputsReturnSafely();
    checkExtremeConditionNumber();
    checkDiagnosticModeDoesNotMutateInputs();
    checkRelativeDirectionWeights();
    checkDirectionalProjectorNormalEquation();
    std::cout << "lio_degeneracy_self_test: PASS\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "lio_degeneracy_self_test: FAIL: " << error.what() << "\n";
    return 1;
  }
}
