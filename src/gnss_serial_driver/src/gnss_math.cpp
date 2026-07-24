#include <gnss_serial_driver/gnss_math.hpp>

#include <cmath>
#include <limits>

namespace
{
constexpr double kGpsEpochUnixSeconds = 315964800.0;
constexpr double kGpsWeekSeconds = 604800.0;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
constexpr double kWgs84SemiMajorAxis = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84EccentricitySquared =
    kWgs84Flattening * (2.0 - kWgs84Flattening);
constexpr double kWgs84SemiMinorAxis =
    kWgs84SemiMajorAxis * (1.0 - kWgs84Flattening);
constexpr double kWgs84SecondEccentricitySquared =
    (kWgs84SemiMajorAxis * kWgs84SemiMajorAxis -
     kWgs84SemiMinorAxis * kWgs84SemiMinorAxis) /
    (kWgs84SemiMinorAxis * kWgs84SemiMinorAxis);

struct LeapSecond
{
  double utc_effective_unix_seconds;
  int gps_minus_utc_seconds;
};

// UTC effective instants from IERS leap-second history, newest first.
constexpr LeapSecond kLeapSeconds[] = {
    {1483228800.0, 18}, {1435708800.0, 17}, {1341100800.0, 16},
    {1230768000.0, 15}, {1136073600.0, 14}, {915148800.0, 13},
    {867715200.0, 12},  {820454400.0, 11},  {773020800.0, 10},
    {741484800.0, 9},   {709948800.0, 8},   {662688000.0, 7},
    {631152000.0, 6},   {567993600.0, 5},   {489024000.0, 4},
    {425865600.0, 3},   {394329600.0, 2},   {362793600.0, 1},
};

Eigen::Vector3d invalidVector()
{
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  return Eigen::Vector3d(invalid, invalid, invalid);
}

bool finiteVector(const Eigen::Vector3d &value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) &&
         std::isfinite(value.z());
}
} // namespace

namespace gnss_serial_driver
{

double gpsWeekTowToUnixUtc(uint32_t week, double tow)
{
  if (!std::isfinite(tow) || tow < 0.0 || tow >= kGpsWeekSeconds)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double gps_as_unix =
      kGpsEpochUnixSeconds + static_cast<double>(week) * kGpsWeekSeconds + tow;
  for (const LeapSecond &leap : kLeapSeconds)
  {
    const double utc = gps_as_unix - leap.gps_minus_utc_seconds;
    if (utc >= leap.utc_effective_unix_seconds) return utc;
  }
  return gps_as_unix;
}

Eigen::Vector3d geodeticToEcef(
    const Eigen::Vector3d &latitude_longitude_height)
{
  if (!finiteVector(latitude_longitude_height) ||
      std::fabs(latitude_longitude_height.x()) > 90.0 ||
      std::fabs(latitude_longitude_height.y()) > 180.0)
  {
    return invalidVector();
  }

  const double latitude = latitude_longitude_height.x() * kDegreesToRadians;
  const double longitude = latitude_longitude_height.y() * kDegreesToRadians;
  const double height = latitude_longitude_height.z();
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double normal_radius =
      kWgs84SemiMajorAxis /
      std::sqrt(1.0 - kWgs84EccentricitySquared *
                          sin_latitude * sin_latitude);

  return Eigen::Vector3d(
      (normal_radius + height) * cos_latitude * std::cos(longitude),
      (normal_radius + height) * cos_latitude * std::sin(longitude),
      (normal_radius * (1.0 - kWgs84EccentricitySquared) + height) *
          sin_latitude);
}

Eigen::Vector3d ecefToGeodetic(const Eigen::Vector3d &ecef)
{
  if (!finiteVector(ecef)) return invalidVector();

  const double horizontal = std::hypot(ecef.x(), ecef.y());
  if (horizontal < 1e-9)
  {
    if (std::fabs(ecef.z()) < 1e-9) return invalidVector();
    return Eigen::Vector3d(
        ecef.z() < 0.0 ? -90.0 : 90.0, 0.0,
        std::fabs(ecef.z()) - kWgs84SemiMinorAxis);
  }

  const double longitude = std::atan2(ecef.y(), ecef.x());
  const double bowring_angle = std::atan2(
      ecef.z() * kWgs84SemiMajorAxis,
      horizontal * kWgs84SemiMinorAxis);
  const double sin_angle = std::sin(bowring_angle);
  const double cos_angle = std::cos(bowring_angle);
  const double latitude = std::atan2(
      ecef.z() + kWgs84SecondEccentricitySquared * kWgs84SemiMinorAxis *
                     sin_angle * sin_angle * sin_angle,
      horizontal - kWgs84EccentricitySquared * kWgs84SemiMajorAxis *
                       cos_angle * cos_angle * cos_angle);
  const double sin_latitude = std::sin(latitude);
  const double normal_radius =
      kWgs84SemiMajorAxis /
      std::sqrt(1.0 - kWgs84EccentricitySquared *
                          sin_latitude * sin_latitude);
  const double height =
      horizontal / std::cos(latitude) - normal_radius;

  return Eigen::Vector3d(latitude * kRadiansToDegrees,
                         longitude * kRadiansToDegrees, height);
}

Eigen::Vector3d ecefDeltaToEnu(
    const Eigen::Vector3d &reference_latitude_longitude_height,
    const Eigen::Vector3d &ecef_delta)
{
  if (!finiteVector(reference_latitude_longitude_height) ||
      !finiteVector(ecef_delta) ||
      std::fabs(reference_latitude_longitude_height.x()) > 90.0 ||
      std::fabs(reference_latitude_longitude_height.y()) > 180.0)
  {
    return invalidVector();
  }

  const double latitude =
      reference_latitude_longitude_height.x() * kDegreesToRadians;
  const double longitude =
      reference_latitude_longitude_height.y() * kDegreesToRadians;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double sin_longitude = std::sin(longitude);
  const double cos_longitude = std::cos(longitude);

  return Eigen::Vector3d(
      -sin_longitude * ecef_delta.x() + cos_longitude * ecef_delta.y(),
      -sin_latitude * cos_longitude * ecef_delta.x() -
          sin_latitude * sin_longitude * ecef_delta.y() +
          cos_latitude * ecef_delta.z(),
      cos_latitude * cos_longitude * ecef_delta.x() +
          cos_latitude * sin_longitude * ecef_delta.y() +
          sin_latitude * ecef_delta.z());
}

} // namespace gnss_serial_driver
