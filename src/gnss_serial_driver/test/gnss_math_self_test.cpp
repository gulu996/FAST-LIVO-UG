#include <gnss_serial_driver/gnss_math.hpp>

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

bool near(double actual, double expected, double tolerance)
{
  return std::fabs(actual - expected) <= tolerance;
}

TEST(GnssMath, ConvertsGpsTimeToUtc)
{
  EXPECT_TRUE(near(
      gnss_serial_driver::gpsWeekTowToUnixUtc(2393, 186157.8),
      1763437339.8, 1e-6));
  EXPECT_TRUE(near(
      gnss_serial_driver::gpsWeekTowToUnixUtc(0, 0.0),
      315964800.0, 1e-9));
  EXPECT_TRUE(std::isnan(gnss_serial_driver::gpsWeekTowToUnixUtc(
      2393, std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(std::isnan(
      gnss_serial_driver::gpsWeekTowToUnixUtc(2393, 604800.0)));
}

TEST(GnssMath, ConvertsWgs84Coordinates)
{
  const Eigen::Vector3d lla(22.5299731, 113.9331154, 39.324);
  const Eigen::Vector3d ecef = gnss_serial_driver::geodeticToEcef(lla);
  const Eigen::Vector3d recovered =
      gnss_serial_driver::ecefToGeodetic(ecef);
  EXPECT_TRUE(near(recovered.x(), lla.x(), 1e-9));
  EXPECT_TRUE(near(recovered.y(), lla.y(), 1e-9));
  EXPECT_TRUE(near(recovered.z(), lla.z(), 1e-4));

  const Eigen::Vector3d reference(0.0, 0.0, 0.0);
  const Eigen::Vector3d east =
      gnss_serial_driver::ecefDeltaToEnu(reference,
                                         Eigen::Vector3d(0.0, 1.0, 0.0));
  const Eigen::Vector3d north =
      gnss_serial_driver::ecefDeltaToEnu(reference,
                                         Eigen::Vector3d(0.0, 0.0, 1.0));
  const Eigen::Vector3d up =
      gnss_serial_driver::ecefDeltaToEnu(reference,
                                         Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_LT((east - Eigen::Vector3d::UnitX()).norm(), 1e-12);
  EXPECT_LT((north - Eigen::Vector3d::UnitY()).norm(), 1e-12);
  EXPECT_LT((up - Eigen::Vector3d::UnitZ()).norm(), 1e-12);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
