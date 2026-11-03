#pragma once
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace fast_livo {
struct PositionCovarianceCheck {
  bool valid = false;
  std::string reason;
  Eigen::Vector3d eigenvalues = Eigen::Vector3d::Constant(NAN);
  double tolerance_m2 = 0.;
};
// Input is a double-precision ENU position covariance in m^2. No repair or
// rescaling: numerical tolerance is 1e-12 m^2 + 1e-10 * max|C_ij| m^2.
inline PositionCovarianceCheck checkPositionCovariance(const Eigen::Matrix3d& C) {
  PositionCovarianceCheck r;
  if (!C.allFinite()) {r.reason="COV_NONFINITE";return r;}
  r.tolerance_m2=1e-12+1e-10*C.cwiseAbs().maxCoeff();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e((.5*(C+C.transpose())).eval());
  if(e.info()==Eigen::Success)r.eigenvalues=e.eigenvalues();
  if((C.diagonal().array()<=0.).any())r.reason="COV_UNKNOWN_OR_NONPOSITIVE_DIAGONAL";
  else if((C-C.transpose()).cwiseAbs().maxCoeff()>r.tolerance_m2)r.reason="COV_ASYMMETRIC";
  else if(!r.eigenvalues.allFinite())r.reason="COV_EIGENSOLVE_FAILED";
  else if(r.eigenvalues.minCoeff() < -r.tolerance_m2)r.reason="COV_NOT_PSD";
  else {r.valid=true;r.reason="VALID_FULL_COVARIANCE_DIAGONAL_BACKEND_MODEL";}
  return r;
}
template<class Array> inline Eigen::Matrix3d positionCovariance(const Array& a) {
  Eigen::Matrix3d C;for(int i=0;i<3;++i)for(int j=0;j<3;++j)C(i,j)=a[i*6+j];return C;
}
inline std::string covarianceEvidence(const Eigen::Matrix3d& C,const PositionCovarianceCheck& r) {
  std::ostringstream s;s<<std::setprecision(17)<<"reason="<<r.reason<<" C_m2=[";
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)s<<C(i,j)<<",";
  s<<"] eigenvalues_m2=["<<r.eigenvalues.transpose()<<"] tolerance_m2="<<r.tolerance_m2;
  return s.str();
}
}
