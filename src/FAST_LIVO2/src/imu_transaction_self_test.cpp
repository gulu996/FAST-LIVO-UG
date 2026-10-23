#include "imu_transaction_interface.h"
#include <iostream>

namespace {
void check(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}
sensor_msgs::ImuConstPtr sample(double time)
{
  auto imu = boost::make_shared<sensor_msgs::Imu>();
  imu->header.stamp.fromSec(time);
  imu->linear_acceleration.z = 9.81;
  return imu;
}
}

int main()
{
  try
  {
    ImuProcess imu;
    StatesGroup state;
    LidarMeasureGroup group;
    group.lio_vio_flg = LIO;
    MeasureGroup measurement;
    measurement.lio_time = 10.102;
    for (int i = 0; i <= 20; ++i) measurement.imu.push_back(sample(10.0 + i * .005));
    group.measures.push_back(measurement);
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    imu.Process2(group, state, cloud);
    const double anchor = sample(10.1)->header.stamp.toSec();
    check(!imu.imu_need_init && imu.coverageStateTime() == anchor, "initialization anchor");
    const auto prior = state;
    const auto before = imu.causalCaptureSnapshot();
    std::deque<sensor_msgs::ImuConstPtr> received;
    check(!imu_transaction::view(received, imu, 10.102).ready, "missing support must wait");
    received.push_back(sample(10.105));
    const auto view = imu_transaction::view(received, imu, 10.102);
    check(view.ready && view.right_support && view.eligible == 0, "real right support");
    group.lio_vio_flg = VIO;
    group.measures.back().vio_time = 10.102;
    group.measures.back().imu = view.samples;
    imu_transaction::WorkingCheckpoint query;
    query.begin(state, imu);
    imu.Process2(group, state, cloud);
    check(imu.coveragePredictionComplete(10.102), "query coverage");
    query.finish(state, imu, false, false);
    check((state.cov.array() == prior.cov.array()).all(), "rollback complete covariance");
    check((state.pos_end.array() == prior.pos_end.array()).all(), "rollback position");
    const auto after = imu.causalCaptureSnapshot();
    check(after.last_propagation_end_time == before.last_propagation_end_time &&
          after.last_imu.header.stamp == before.last_imu.header.stamp &&
          after.imu_pose.size() == before.imu_pose.size(), "rollback time/carry/history");
    check(received.size() == 1, "query must not consume input");
    imu_transaction::release(received, anchor);
    check(received.size() == 1, "release must preserve future support");
    bool rejected = false;
    try { query.finish(state, imu, false, false); }
    catch (const std::logic_error &) { rejected = true; }
    check(rejected, "double finish must fail");
    std::cout << "IMU_TRANSACTION_SELF_TEST_PASS\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
