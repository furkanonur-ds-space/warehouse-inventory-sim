// HITL bridge between Gazebo Harmonic and PX4 running on a VOXL2 board.
//
// Stage 2: the simulated sensors are packed into HIL_SENSOR and HIL_GPS and
// sent over UDP. Nothing is received yet, so the vehicle does not move; that
// is stage 3.
//
// Every conversion here has a source behind it, recorded in
// docs/units_and_frames.md. The two that are not obvious: the magnetometer
// already arrives in gauss despite its field name, and it arrives in a frame
// of its own that is not the frame the other sensors use.
//
// The plugin is attached to the world rather than to the model, because it
// holds one socket for all the sensors.

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <common/mavlink.h>

namespace hitl_bridge
{

// Bits of HIL_SENSOR's fields_updated, which tells PX4 which of the fields
// in this particular message carry something new. Sending a stale
// magnetometer reading 250 times a second would be a lie about its rate.
//
// The values are PX4's, from SensorSource in SimulatorMavlink.hpp, and the
// barometer one is not the single bit its name suggests. PX4 tests
// (fields_updated & BARO) == BARO, so all three of absolute pressure,
// pressure altitude and temperature have to be marked, and a message that
// marks only the pressure is dropped in silence.
constexpr uint32_t kFieldAccel = 0x07;    // 0b111
constexpr uint32_t kFieldGyro = 0x38;     // 0b111000
constexpr uint32_t kFieldMag = 0x1C0;     // 0b111000000
constexpr uint32_t kFieldBaro = 0x1A00;   // 0b1101000000000

// Standard atmosphere, for the pressure altitude PX4 wants alongside the
// pressure itself.
constexpr double kSeaLevelHpa = 1013.25;

struct Vec3
{
  double x = 0.0, y = 0.0, z = 0.0;
};

// One counter per stream, so the report can say what arrived and what left.
struct Counter
{
  uint64_t total = 0;
  uint64_t at_last_report = 0;
};

class HitlBridge : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPostUpdate
{
public:
  ~HitlBridge() override
  {
    running_ = false;
    // Closing the socket is what wakes the receive thread out of recvfrom.
    if (socket_ >= 0)
      close(socket_);
    if (receiver_.joinable())
      receiver_.join();
  }

  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    // Topic names come from the world file rather than being built here.
    // Gazebo derives them from the scoped name of the sensor, so guessing
    // them in code means the plugin silently reads nothing when the model
    // is renamed.
    imu_topic_ = sdf->Get<std::string>("imu_topic", "").first;
    mag_topic_ = sdf->Get<std::string>("mag_topic", "").first;
    baro_topic_ = sdf->Get<std::string>("baro_topic", "").first;
    gps_topic_ = sdf->Get<std::string>("gps_topic", "").first;
    report_every_s_ = sdf->Get<double>("report_every_s", 1.0).first;

    // Names and defaults follow the ModalAI document, so that a world file
    // written for their plugin needs no translation.
    const std::string addr =
        sdf->Get<std::string>("mavlink_addr", "127.0.0.1").first;
    const int remote_port =
        sdf->Get<int>("mavlink_udp_remote_port", 14560).first;
    const int local_port = sdf->Get<int>("mavlink_udp_local_port", 0).first;
    send_ = sdf->Get<bool>("send", true).first;

    if (send_ && !OpenSocket(addr, remote_port, local_port))
      send_ = false;

    // Motor commands arrive on the same socket the sensors leave by, which
    // is why PX4 does not need to be told a port to answer on.
    // The motor model builds this from the model name and its
    // commandSubTopic, with no /model prefix, so publishing to
    // /model/<name>/command/motor_speed reaches nobody. Nothing reports the
    // mistake: the topic simply has no subscriber and the vehicle sits
    // still while the commands arrive.
    motor_topic_ = sdf->Get<std::string>(
        "motor_topic", "/x500_voxl/command/motor_speed").first;
    max_rot_velocity_ = sdf->Get<double>("max_rot_velocity", 1000.0).first;
    motor_count_ = sdf->Get<int>("motor_count", 4).first;
    if (send_)
    {
      motor_pub_ = node_.Advertise<gz::msgs::Actuators>(motor_topic_);
      if (!motor_pub_)
        std::cerr << "[hitl_bridge] ERROR could not advertise "
                  << motor_topic_ << "\n";
      running_ = true;
      receiver_ = std::thread(&HitlBridge::ReceiveLoop, this);
    }

    if (!imu_topic_.empty() &&
        !node_.Subscribe(imu_topic_, &HitlBridge::OnImu, this))
      Failed(imu_topic_);
    if (!mag_topic_.empty() &&
        !node_.Subscribe(mag_topic_, &HitlBridge::OnMag, this))
      Failed(mag_topic_);
    if (!baro_topic_.empty() &&
        !node_.Subscribe(baro_topic_, &HitlBridge::OnBaro, this))
      Failed(baro_topic_);
    if (!gps_topic_.empty() &&
        !node_.Subscribe(gps_topic_, &HitlBridge::OnGps, this))
      Failed(gps_topic_);

    if (send_)
      std::cout << "[hitl_bridge] stage 2: sending HIL_SENSOR and HIL_GPS to "
                << addr << ":" << remote_port << std::endl;
    else
      std::cout << "[hitl_bridge] stage 2: counting only, sending nothing" << std::endl;
  }

  // Reporting is driven by simulated time, not the wall clock, so the rates
  // printed are the rates PX4 would see. A slow machine stretches the wall
  // clock without changing them.
  //
  // Simulated time is also what stamps the outgoing messages. PX4 compares
  // those stamps against each other, so they have to come from the same
  // clock that drives the physics rather than from this machine's.
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &) override
  {
    if (info.paused)
      return;

    const double now_s = std::chrono::duration<double>(info.simTime).count();
    sim_time_us_.store(
        static_cast<uint64_t>(std::llround(now_s * 1.0e6)));

    if (last_report_s_ < 0.0)
    {
      last_report_s_ = now_s;
      return;
    }
    const double elapsed = now_s - last_report_s_;
    if (elapsed < report_every_s_)
      return;
    last_report_s_ = now_s;

    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << "[hitl_bridge] in: " << Rate(imu_in_, elapsed, "imu")
              << Rate(mag_in_, elapsed, "mag") << Rate(baro_in_, elapsed, "baro")
              << Rate(gps_in_, elapsed, "gps") << " | out: "
              << Rate(hil_sensor_out_, elapsed, "HIL_SENSOR")
              << Rate(hil_gps_out_, elapsed, "HIL_GPS")
              << "| back: " << Rate(actuators_in_, elapsed, "actuators");
    if (!last_controls_.empty())
    {
      std::cout << " [";
      for (size_t i = 0; i < last_controls_.size(); ++i)
        std::cout << (i ? " " : "") << last_controls_[i];
      std::cout << "] flags 0x" << std::hex << last_actuator_flags_
                << std::dec;
    }
    if (send_failures_ > 0)
      std::cout << " | send failures " << send_failures_;
    std::cout << std::endl;
  }

private:
  static void Failed(const std::string &topic)
  {
    std::cerr << "[hitl_bridge] ERROR could not subscribe to " << topic
              << "\n";
  }

  bool OpenSocket(const std::string &addr, int remote_port, int local_port)
  {
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0)
    {
      std::cerr << "[hitl_bridge] ERROR could not open UDP socket\n";
      return false;
    }

    // Binding a local port is optional, but the reply stream in stage 3
    // arrives on whatever port this socket holds, so a fixed one can be
    // asked for.
    if (local_port > 0)
    {
      sockaddr_in local{};
      local.sin_family = AF_INET;
      local.sin_addr.s_addr = htonl(INADDR_ANY);
      local.sin_port = htons(static_cast<uint16_t>(local_port));
      if (bind(socket_, reinterpret_cast<sockaddr *>(&local),
               sizeof(local)) < 0)
      {
        std::cerr << "[hitl_bridge] ERROR could not bind local port "
                  << local_port << "\n";
        close(socket_);
        socket_ = -1;
        return false;
      }
    }

    // Without a receive timeout the plugin cannot be shut down. The
    // receive thread blocks in recvfrom, and closing the socket from
    // another thread does not reliably wake it on Linux, so the destructor
    // waits for a thread that never returns and the simulator hangs after
    // finishing its run. This was not theoretical: three simulators were
    // left behind before it was found.
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;  // 100 ms
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                   sizeof(timeout)) < 0)
      std::cerr << "[hitl_bridge] WARNING could not set a receive timeout; "
                   "shutdown may hang\n";

    std::memset(&remote_, 0, sizeof(remote_));
    remote_.sin_family = AF_INET;
    remote_.sin_port = htons(static_cast<uint16_t>(remote_port));
    if (inet_pton(AF_INET, addr.c_str(), &remote_.sin_addr) != 1)
    {
      std::cerr << "[hitl_bridge] ERROR bad address " << addr << "\n";
      close(socket_);
      socket_ = -1;
      return false;
    }
    return true;
  }

  void Send(const mavlink_message_t &msg)
  {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);
    const ssize_t sent =
        sendto(socket_, buffer, length, 0,
               reinterpret_cast<const sockaddr *>(&remote_), sizeof(remote_));
    if (sent != static_cast<ssize_t>(length))
      ++send_failures_;
  }

  // PX4 answers on the socket the sensors arrive from, so this thread does
  // nothing but block on that socket. It is a thread rather than a poll in
  // PostUpdate because the motor stream runs at 200 Hz and a command that
  // waits for the next physics step is a command that arrives late.
  void ReceiveLoop()
  {
    uint8_t buffer[2048];
    mavlink_message_t msg;
    mavlink_status_t status;

    while (running_)
    {
      sockaddr_in from{};
      socklen_t from_len = sizeof(from);
      const ssize_t bytes =
          recvfrom(socket_, buffer, sizeof(buffer), 0,
                   reinterpret_cast<sockaddr *>(&from), &from_len);
      if (bytes <= 0)
      {
        if (!running_)
          break;
        continue;
      }

      for (ssize_t i = 0; i < bytes; ++i)
      {
        if (!mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
          continue;
        if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS)
          OnActuatorControls(msg);
      }
    }
  }

  // PX4 sends each motor as a number from 0 to 1. The motor model wants a
  // rotor velocity, and maxRotVelocity in the model says what 1 means.
  void OnActuatorControls(const mavlink_message_t &msg)
  {
    mavlink_hil_actuator_controls_t controls;
    mavlink_msg_hil_actuator_controls_decode(&msg, &controls);

    gz::msgs::Actuators command;
    for (int i = 0; i < motor_count_; ++i)
    {
      double normalised = controls.controls[i];
      if (!std::isfinite(normalised) || normalised < 0.0)
        normalised = 0.0;
      if (normalised > 1.0)
        normalised = 1.0;
      command.add_velocity(normalised * max_rot_velocity_);
    }

    motor_pub_.Publish(command);

    std::lock_guard<std::mutex> lock(mutex_);
    ++actuators_in_.total;
    last_controls_.assign(controls.controls,
                          controls.controls + motor_count_);
    last_actuator_flags_ = controls.flags;
  }

  std::string Rate(const Counter &counter, double elapsed,
                   const std::string &name) const
  {
    const double hz =
        elapsed > 0.0
            ? static_cast<double>(counter.total - counter.at_last_report) /
                  elapsed
            : 0.0;
    const_cast<Counter &>(counter).at_last_report = counter.total;
    return name + " " + std::to_string(static_cast<int>(hz + 0.5)) + " Hz  ";
  }

  // The IMU is the fastest sensor and the one PX4 paces itself by, so it
  // carries the message. Magnetometer and barometer ride along on the
  // messages that follow their own arrivals, marked in fields_updated, and
  // are not repeated in between.
  void OnImu(const gz::msgs::IMU &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++imu_in_.total;
    if (!send_)
      return;

    mavlink_hil_sensor_t hil{};
    hil.time_usec = sim_time_us_.load();
    hil.id = 0;

    // Gazebo's body frame is FLU, PX4's is FRD.
    hil.xacc = static_cast<float>(msg.linear_acceleration().x());
    hil.yacc = static_cast<float>(-msg.linear_acceleration().y());
    hil.zacc = static_cast<float>(-msg.linear_acceleration().z());
    hil.xgyro = static_cast<float>(msg.angular_velocity().x());
    hil.ygyro = static_cast<float>(-msg.angular_velocity().y());
    hil.zgyro = static_cast<float>(-msg.angular_velocity().z());
    uint32_t fields = kFieldAccel | kFieldGyro;

    if (mag_fresh_)
    {
      hil.xmag = static_cast<float>(mag_.x);
      hil.ymag = static_cast<float>(mag_.y);
      hil.zmag = static_cast<float>(mag_.z);
      fields |= kFieldMag;
      mag_fresh_ = false;
    }
    if (baro_fresh_)
    {
      hil.abs_pressure = static_cast<float>(baro_hpa_);
      hil.pressure_alt = static_cast<float>(
          44330.0 * (1.0 - std::pow(baro_hpa_ / kSeaLevelHpa, 1.0 / 5.255)));
      fields |= kFieldBaro;
      baro_fresh_ = false;
    }
    hil.fields_updated = fields;
    hil.temperature = kTemperatureC;

    mavlink_message_t out;
    mavlink_msg_hil_sensor_encode(kSystemId, kComponentId, &out, &hil);
    Send(out);
    ++hil_sensor_out_.total;
  }

  // Already gauss, and in a frame of its own. See docs/units_and_frames.md;
  // this mapping is PX4's own, from GZBridge::magnetometerCallback.
  void OnMag(const gz::msgs::Magnetometer &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mag_.x = -msg.field_tesla().y();
    mag_.y = -msg.field_tesla().x();
    mag_.z = msg.field_tesla().z();
    mag_fresh_ = true;
    ++mag_in_.total;
  }

  void OnBaro(const gz::msgs::FluidPressure &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    baro_hpa_ = msg.pressure() / 100.0;  // Pa to hPa
    baro_fresh_ = true;
    ++baro_in_.total;
  }

  // HIL_GPS is integers rather than SI, and it is its own message rather
  // than fields inside HIL_SENSOR.
  void OnGps(const gz::msgs::NavSat &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++gps_in_.total;
    if (!send_)
      return;

    mavlink_hil_gps_t gps{};
    gps.time_usec = sim_time_us_.load();
    gps.fix_type = 3;  // 3D fix
    gps.lat = static_cast<int32_t>(std::llround(msg.latitude_deg() * 1e7));
    gps.lon = static_cast<int32_t>(std::llround(msg.longitude_deg() * 1e7));
    gps.alt = static_cast<int32_t>(std::llround(msg.altitude() * 1000.0));
    gps.eph = 100;  // 1.00 m, centimetres
    gps.epv = 100;

    // NavSat velocities are ENU; HIL_GPS wants NED, in centimetres.
    const double vn = msg.velocity_north();
    const double ve = msg.velocity_east();
    const double vd = -msg.velocity_up();
    gps.vn = static_cast<int16_t>(std::llround(vn * 100.0));
    gps.ve = static_cast<int16_t>(std::llround(ve * 100.0));
    gps.vd = static_cast<int16_t>(std::llround(vd * 100.0));
    gps.vel = static_cast<uint16_t>(
        std::llround(std::sqrt(vn * vn + ve * ve) * 100.0));

    // Course over ground in centidegrees, measured from north. Undefined
    // when the vehicle is not moving, and PX4 reads 65535 as "unknown".
    if (std::abs(vn) > 0.01 || std::abs(ve) > 0.01)
    {
      double cog_deg = std::atan2(ve, vn) * 180.0 / M_PI;
      if (cog_deg < 0.0)
        cog_deg += 360.0;
      gps.cog = static_cast<uint16_t>(std::llround(cog_deg * 100.0));
    }
    else
    {
      gps.cog = 65535;
    }
    gps.satellites_visible = 12;
    gps.id = 0;

    mavlink_message_t out;
    mavlink_msg_hil_gps_encode(kSystemId, kComponentId, &out, &gps);
    Send(out);
    ++hil_gps_out_.total;
  }

  static constexpr uint8_t kSystemId = 1;
  static constexpr uint8_t kComponentId = MAV_COMP_ID_AUTOPILOT1;

  // Gazebo's air pressure sensor does not simulate temperature, and PX4
  // uses this value for the barometer's own temperature reading.
  static constexpr float kTemperatureC = 25.0f;

  gz::transport::Node node_;
  std::mutex mutex_;

  std::string imu_topic_, mag_topic_, baro_topic_, gps_topic_;
  Counter imu_in_, mag_in_, baro_in_, gps_in_;
  Counter hil_sensor_out_, hil_gps_out_;
  Counter actuators_in_;

  gz::transport::Node::Publisher motor_pub_;
  std::string motor_topic_;
  double max_rot_velocity_ = 1000.0;
  int motor_count_ = 4;
  std::vector<float> last_controls_;
  uint32_t last_actuator_flags_ = 0;
  std::thread receiver_;
  std::atomic<bool> running_{false};

  Vec3 mag_;
  double baro_hpa_ = 0.0;
  bool mag_fresh_ = false;
  bool baro_fresh_ = false;

  std::atomic<uint64_t> sim_time_us_{0};
  double report_every_s_ = 1.0;
  double last_report_s_ = -1.0;

  bool send_ = true;
  int socket_ = -1;
  sockaddr_in remote_{};
  uint64_t send_failures_ = 0;
};

}  // namespace hitl_bridge

GZ_ADD_PLUGIN(hitl_bridge::HitlBridge, gz::sim::System,
              hitl_bridge::HitlBridge::ISystemConfigure,
              hitl_bridge::HitlBridge::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(hitl_bridge::HitlBridge, "hitl_bridge::HitlBridge")
