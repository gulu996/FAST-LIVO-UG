#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;

namespace {
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "VPPC requires IEEE-754 float32");
constexpr size_t kHeaderSize = 36;
constexpr size_t kStride = 16;
constexpr size_t kMaxPayload = 64 * 1024 * 1024;

void put(std::vector<uint8_t> &out, uint64_t value, unsigned bytes) {
  for (unsigned i = 0; i < bytes; ++i) out.push_back(static_cast<uint8_t>(value >> (8 * i)));
}
uint32_t readWord(const uint8_t *p, bool big_endian) {
  uint32_t bits = 0;
  for (unsigned i = 0; i < 4; ++i)
    bits |= uint32_t(p[big_endian ? 3 - i : i]) << (8 * i);
  return bits;
}
float readFloat(const uint8_t *p, bool big_endian) {
  const uint32_t bits = readWord(p, big_endian);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
void putFloat(std::vector<uint8_t> &out, float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  put(out, bits, 4);
}
struct Cell {
  std::array<int64_t, 3> v;
  bool operator==(const Cell &other) const { return v == other.v; }
};
struct CellHash {
  size_t operator()(const Cell &cell) const {
    size_t h = 0;
    for (auto v : cell.v) h ^= std::hash<int64_t>{}(v) + size_t(0x9e3779b9) + (h << 6) + (h >> 2);
    return h;
  }
};

class Gateway;
class Session : public std::enable_shared_from_this<Session> {
 public:
  Session(tcp::socket socket, Gateway &gateway) : ws_(std::move(socket)), gateway_(gateway) {}
  void accept();
  void send(std::shared_ptr<std::vector<uint8_t>> frame);
  void close() { boost::system::error_code ec; ws_.next_layer().close(ec); }
  bool writing() const { return bool(current_); }
 private:
  void read();
  void fail(const boost::system::error_code &ec);
  websocket::stream<tcp::socket> ws_;
  beast::flat_buffer input_;
  std::shared_ptr<std::vector<uint8_t>> current_;
  Gateway &gateway_;
};

class Gateway {
 public:
  Gateway(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
      : io_(), acceptor_(io_), work_(asio::make_work_guard(io_)) {
    private_nh.param<std::string>("input_topic", topic_, "/cloud_registered");
    private_nh.param<std::string>("listen_address", address_, "0.0.0.0");
    private_nh.param<int>("port", port_, 8765);
    private_nh.param<double>("voxel_size_m", voxel_, 0.0);
    private_nh.param<double>("max_range_m", range_, 0.0);
    private_nh.param<bool>("print_statistics", print_statistics_, true);
    private_nh.param<double>("statistics_interval_s", interval_, 5.0);
    int max_pending = 1;
    private_nh.param<int>("max_pending_frames", max_pending, 1);
    if (port_ < 1 || port_ > 65535 || !std::isfinite(voxel_) || voxel_ < 0 ||
        !std::isfinite(range_) || range_ < 0 || !std::isfinite(interval_) ||
        interval_ <= 0 || max_pending != 1)
      throw std::invalid_argument("invalid gateway parameter (max_pending_frames must be 1)");
    const auto ip = asio::ip::make_address(address_);
    tcp::endpoint endpoint(ip, static_cast<uint16_t>(port_));
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(asio::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();
    accept();
    thread_ = std::thread([this] { io_.run(); });
    subscriber_ = nh.subscribe(topic_, 1, &Gateway::cloud, this);
    if (print_statistics_) timer_ = nh.createTimer(ros::Duration(interval_), &Gateway::statistics, this);
    ROS_INFO("VisionPro gateway listening on %s:%d, topic %s", address_.c_str(), port_, topic_.c_str());
  }
  ~Gateway() {
    subscriber_.shutdown();
    timer_.stop();
    asio::post(io_, [this] {
      boost::system::error_code ec;
      acceptor_.close(ec);
      if (session_) session_->close();
      work_.reset();
    });
    if (thread_.joinable()) thread_.join();
  }
  void connected(const std::shared_ptr<Session> &session) {
    connected_.store(1);
    session_ = session;
    { std::lock_guard<std::mutex> lock(mutex_); pending_.reset(); }
    ROS_INFO("VisionPro client connected");
  }
  void disconnected(Session *session) {
    if (session_.get() != session) return;
    connected_.store(0);
    session_.reset();
    { std::lock_guard<std::mutex> lock(mutex_); if (pending_) { ++dropped_; pending_.reset(); } }
    ROS_INFO("VisionPro client disconnected");
  }
  void pump() {
    if (!session_ || session_->writing()) return;
    std::shared_ptr<std::vector<uint8_t>> frame;
    { std::lock_guard<std::mutex> lock(mutex_); frame.swap(pending_); }
    if (frame) session_->send(std::move(frame));
  }
  void sent(size_t bytes) { ++sent_; bytes_ += bytes; pump(); }
 private:
  void accept() {
    acceptor_.async_accept([this](boost::system::error_code ec, tcp::socket socket) {
      if (!ec) {
        if (session_) { boost::system::error_code ignored; socket.close(ignored); }
        else { session_ = std::make_shared<Session>(std::move(socket), *this); session_->accept(); }
      }
      if (acceptor_.is_open()) accept();
    });
  }
  void cloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    ++received_;
    if (!connected_.load()) return;
    const uint64_t n = uint64_t(msg->width) * msg->height;
    if (msg->header.frame_id.size() > UINT16_MAX || n > kMaxPayload / kStride ||
        msg->point_step < 16 || uint64_t(msg->width) * msg->point_step > msg->row_step ||
        uint64_t(msg->height) * msg->row_step != msg->data.size()) {
      ROS_WARN_THROTTLE(5, "VisionPro invalid PointCloud2 dimensions");
      ++dropped_; return;
    }
    std::array<unsigned, 3> offset{};
    const char *names[] = {"x", "y", "z"};
    for (size_t i = 0; i < 3; ++i) {
      bool found = false;
      for (const auto &field : msg->fields)
        if (field.name == names[i] && field.datatype == sensor_msgs::PointField::FLOAT32 &&
            field.count == 1 && field.offset <= msg->point_step - 4) {
          offset[i] = field.offset; found = true; break;
        }
      if (!found) { ROS_WARN_THROTTLE(5, "VisionPro requires FLOAT32 x/y/z"); ++dropped_; return; }
    }
    bool has_rgb = false, has_intensity = false;
    unsigned color_offset = 0, intensity_offset = 0;
    for (const auto &field : msg->fields) {
      if ((field.name == "rgb" || field.name == "rgba") &&
          (field.datatype == sensor_msgs::PointField::FLOAT32 || field.datatype == sensor_msgs::PointField::UINT32) &&
          field.count == 1 && field.offset <= msg->point_step - 4) {
        has_rgb = true; color_offset = field.offset;
      }
      if (field.name == "intensity" && field.datatype == sensor_msgs::PointField::FLOAT32 &&
          field.count == 1 && field.offset <= msg->point_step - 4) {
        has_intensity = true; intensity_offset = field.offset;
      }
    }
    if (!has_rgb && !has_intensity) {
      ROS_WARN_THROTTLE(5, "VisionPro requires rgb/rgba or FLOAT32 intensity");
      ++dropped_; return;
    }
    input_points_ += n;
    auto out = std::make_shared<std::vector<uint8_t>>();
    out->reserve(kHeaderSize + msg->header.frame_id.size() + size_t(n) * kStride);
    out->insert(out->end(), {'V','P','P','C'});
    put(*out, 1, 2); put(*out, has_rgb ? 2 : 1, 2); put(*out, ++sequence_, 8);
    put(*out, msg->header.stamp.toNSec(), 8);
    const size_t count_at = out->size();
    put(*out, 0, 4); put(*out, kStride, 2);
    put(*out, msg->header.frame_id.size(), 2);
    const size_t payload_at = out->size(); put(*out, 0, 4);
    out->insert(out->end(), msg->header.frame_id.begin(), msg->header.frame_id.end());
    std::unordered_set<Cell, CellHash> cells;
    uint32_t count = 0;
    for (uint32_t row = 0; row < msg->height; ++row) {
      for (uint32_t col = 0; col < msg->width; ++col) {
        const uint8_t *p = msg->data.data() + size_t(row) * msg->row_step + size_t(col) * msg->point_step;
        float v[3];
        for (size_t i = 0; i < 3; ++i) v[i] = readFloat(p + offset[i], msg->is_bigendian);
        if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]) ||
            (!has_rgb && !std::isfinite(readFloat(p + intensity_offset, msg->is_bigendian)))) continue;
        const double distance2 = double(v[0])*v[0] + double(v[1])*v[1] + double(v[2])*v[2];
        if (range_ > 0 && distance2 > range_*range_) continue;
        if (voxel_ > 0) {
          Cell cell;
          bool valid = true;
          for (size_t i = 0; i < 3; ++i) {
            const double index = std::floor(double(v[i]) / voxel_);
            if (index < double(INT64_MIN) || index >= 9223372036854775808.0) { valid = false; break; }
            cell.v[i] = static_cast<int64_t>(index);
          }
          if (!valid || !cells.insert(cell).second) continue;
        }
        for (float value : v) putFloat(*out, value);
        if (has_rgb) {
          const uint32_t rgb = readWord(p + color_offset, msg->is_bigendian);
          out->push_back(uint8_t(rgb >> 16));
          out->push_back(uint8_t(rgb >> 8));
          out->push_back(uint8_t(rgb));
          out->push_back(255);
        } else putFloat(*out, readFloat(p + intensity_offset, msg->is_bigendian));
        ++count;
      }
    }
    const uint32_t payload = count * kStride;
    for (unsigned i = 0; i < 4; ++i) {
      (*out)[count_at+i] = uint8_t(count >> (8*i));
      (*out)[payload_at+i] = uint8_t(payload >> (8*i));
    }
    { std::lock_guard<std::mutex> lock(mutex_); if (pending_) ++dropped_; pending_ = std::move(out); }
    // ponytail: one pending frame plus one in-flight write is the fixed ceiling; a newer scan replaces the pending one.
    if (!notified_.exchange(true)) asio::post(io_, [this] { notified_.store(false); pump(); });
  }
  void statistics(const ros::TimerEvent &event) {
    const uint64_t received = received_.load(), points = input_points_.load();
    const uint64_t frame_delta = received - last_received_, point_delta = points - last_input_points_;
    const double elapsed = (event.current_real - event.last_real).toSec();
    last_received_ = received; last_input_points_ = points;
    ROS_INFO("VisionPro received_frames=%llu sent_frames=%llu dropped_frames=%llu connected_clients=%u bytes_sent=%llu input_hz=%.2f points_per_frame=%.1f",
      static_cast<unsigned long long>(received_.load()), static_cast<unsigned long long>(sent_.load()),
      static_cast<unsigned long long>(dropped_.load()), connected_.load(),
      static_cast<unsigned long long>(bytes_.load()),
      elapsed > 0 ? frame_delta / elapsed : 0.0,
      frame_delta ? double(point_delta) / frame_delta : 0.0);
  }
  friend class Session;
  asio::io_context io_;
  tcp::acceptor acceptor_;
  asio::executor_work_guard<asio::io_context::executor_type> work_;
  std::thread thread_;
  ros::Subscriber subscriber_;
  ros::Timer timer_;
  std::shared_ptr<Session> session_;
  std::mutex mutex_;
  std::shared_ptr<std::vector<uint8_t>> pending_;
  std::atomic<bool> notified_{false};
  std::atomic<uint64_t> received_{0}, sent_{0}, dropped_{0}, bytes_{0}, input_points_{0};
  uint64_t last_received_ = 0, last_input_points_ = 0;
  std::atomic<unsigned> connected_{0};
  uint64_t sequence_ = 0;
  std::string topic_, address_;
  int port_;
  double voxel_, range_, interval_;
  bool print_statistics_;
};

void Session::accept() {
  ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
  ws_.async_accept([self=shared_from_this()](boost::system::error_code ec) {
    if (ec) { self->fail(ec); return; }
    self->ws_.binary(true);
    self->ws_.read_message_max(1024);
    self->gateway_.connected(self);
    self->read();
  });
}
void Session::read() {
  ws_.async_read(input_, [self=shared_from_this()](boost::system::error_code ec, size_t) {
    if (ec) { self->fail(ec); return; }
    self->input_.consume(self->input_.size());
    self->read();
  });
}
void Session::send(std::shared_ptr<std::vector<uint8_t>> frame) {
  current_ = std::move(frame);
  ws_.async_write(asio::buffer(*current_), [self=shared_from_this()](boost::system::error_code ec, size_t) {
    if (ec) { self->current_.reset(); self->fail(ec); return; }
    const size_t bytes = self->current_->size();
    self->current_.reset();
    self->gateway_.sent(bytes);
  });
}
void Session::fail(const boost::system::error_code &ec) {
  if (ec != websocket::error::closed && ec != asio::error::operation_aborted)
    ROS_WARN("VisionPro WebSocket: %s", ec.message().c_str());
  close();
  gateway_.disconnected(this);
}
} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "visionpro_pointcloud_gateway");
  ros::NodeHandle nh, private_nh("~");
  try {
    Gateway gateway(nh, private_nh);
    ros::spin();
  } catch (const std::exception &e) {
    ROS_FATAL("VisionPro gateway: %s", e.what());
    return 1;
  }
  return 0;
}
