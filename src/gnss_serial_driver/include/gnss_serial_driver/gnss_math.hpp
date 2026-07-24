#ifndef GNSS_SERIAL_DRIVER_GNSS_MATH_HPP_
#define GNSS_SERIAL_DRIVER_GNSS_MATH_HPP_

#include <cstdint>

#include <Eigen/Core>

namespace gnss_serial_driver
{

double gpsWeekTowToUnixUtc(uint32_t week, double tow);

Eigen::Vector3d geodeticToEcef(const Eigen::Vector3d &latitude_longitude_height);

Eigen::Vector3d ecefToGeodetic(const Eigen::Vector3d &ecef);

Eigen::Vector3d ecefDeltaToEnu(
    const Eigen::Vector3d &reference_latitude_longitude_height,
    const Eigen::Vector3d &ecef_delta);

} // namespace gnss_serial_driver

#endif
