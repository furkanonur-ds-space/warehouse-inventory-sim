// HITL bridge between Gazebo Harmonic and PX4 running on a VOXL2 board.
//
// Stage 1, which is what this file is now: subscribe to the simulated
// sensors and report what arrives and how often. Nothing is sent anywhere
// yet. The point of stopping here is that every later stage depends on the
// rates being right, and the ModalAI document is specific about them: IMU
// 250 Hz, magnetometer 50 Hz, barometer 10 Hz, GPS 30 Hz.
//
// The plugin is attached to the world rather than to the model, because it
// has to see every sensor and, later, hold one UDP socket for all of them.

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>

namespace hitl_bridge
{

// One counter per sensor. The last message is kept so that the report can
// show a value, which is what catches a sensor that publishes at the right
// rate but in the wrong units or the wrong frame.
struct Channel
{
  std::string topic;
  uint64_t count = 0;
  uint64_t count_at_last_report = 0;
  double last_value[3] = {0.0, 0.0, 0.0};
  bool seen = false;
};

class HitlBridge : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    // Topic names come from the world file rather than being built here.
    // Gazebo derives them from the scoped name of the sensor, so guessing
    // them in code means the plugin silently reads nothing when the model
    // is renamed.
    imu_.topic = Param(sdf, "imu_topic", "");
    mag_.topic = Param(sdf, "mag_topic", "");
    baro_.topic = Param(sdf, "baro_topic", "");
    gps_.topic = Param(sdf, "gps_topic", "");
    report_every_s_ = ParamDouble(sdf, "report_every_s", 1.0);

    // Subscribed one by one rather than through a helper: the message type
    // has to be deducible from the callback, and a wrapper hides it.
    if (!imu_.topic.empty() &&
        !node_.Subscribe(imu_.topic, &HitlBridge::OnImu, this))
      Failed(imu_.topic);
    if (!mag_.topic.empty() &&
        !node_.Subscribe(mag_.topic, &HitlBridge::OnMag, this))
      Failed(mag_.topic);
    if (!baro_.topic.empty() &&
        !node_.Subscribe(baro_.topic, &HitlBridge::OnBaro, this))
      Failed(baro_.topic);
    if (!gps_.topic.empty() &&
        !node_.Subscribe(gps_.topic, &HitlBridge::OnGps, this))
      Failed(gps_.topic);

    std::cout << "[hitl_bridge] stage 1: counting sensor messages, sending "
                 "nothing\n";
  }

  // Reporting is driven by simulated time, not the wall clock, so the rates
  // printed are the rates PX4 would see. A slow machine stretches the wall
  // clock without changing them.
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &) override
  {
    if (info.paused)
      return;

    const double now_s =
        std::chrono::duration<double>(info.simTime).count();
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
    std::cout << "[hitl_bridge] " << Rate(imu_, elapsed, "imu")
              << Rate(mag_, elapsed, "mag") << Rate(baro_, elapsed, "baro")
              << Rate(gps_, elapsed, "gps") << "\n";
    Report(imu_, "imu accel m/s^2");
    Report(mag_, "mag");
    Report(baro_, "baro Pa");
    Report(gps_, "gps lat/lon/alt");
  }

private:
  static std::string Param(const std::shared_ptr<const sdf::Element> &sdf,
                           const std::string &name,
                           const std::string &fallback)
  {
    return sdf->Get<std::string>(name, fallback).first;
  }

  static double ParamDouble(const std::shared_ptr<const sdf::Element> &sdf,
                            const std::string &name, double fallback)
  {
    return sdf->Get<double>(name, fallback).first;
  }

  static void Failed(const std::string &topic)
  {
    std::cerr << "[hitl_bridge] ERROR could not subscribe to " << topic
              << "\n";
  }

  std::string Rate(const Channel &channel, double elapsed,
                   const std::string &name) const
  {
    if (channel.topic.empty())
      return "";
    const double hz =
        elapsed > 0.0
            ? static_cast<double>(channel.count - channel.count_at_last_report) /
                  elapsed
            : 0.0;
    return name + " " + std::to_string(static_cast<int>(hz + 0.5)) + " Hz  ";
  }

  void Report(Channel &channel, const std::string &label)
  {
    if (channel.topic.empty())
      return;
    if (!channel.seen)
    {
      std::cerr << "[hitl_bridge] WARNING no message yet on "
                << channel.topic << "\n";
    }
    else
    {
      std::cout << "[hitl_bridge]   " << label << ": " << channel.last_value[0]
                << ", " << channel.last_value[1] << ", "
                << channel.last_value[2] << "\n";
    }
    channel.count_at_last_report = channel.count;
  }

  void OnImu(const gz::msgs::IMU &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_.last_value[0] = msg.linear_acceleration().x();
    imu_.last_value[1] = msg.linear_acceleration().y();
    imu_.last_value[2] = msg.linear_acceleration().z();
    imu_.seen = true;
    ++imu_.count;
  }

  void OnMag(const gz::msgs::Magnetometer &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mag_.last_value[0] = msg.field_tesla().x();
    mag_.last_value[1] = msg.field_tesla().y();
    mag_.last_value[2] = msg.field_tesla().z();
    mag_.seen = true;
    ++mag_.count;
  }

  void OnBaro(const gz::msgs::FluidPressure &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    baro_.last_value[0] = msg.pressure();
    baro_.seen = true;
    ++baro_.count;
  }

  void OnGps(const gz::msgs::NavSat &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gps_.last_value[0] = msg.latitude_deg();
    gps_.last_value[1] = msg.longitude_deg();
    gps_.last_value[2] = msg.altitude();
    gps_.seen = true;
    ++gps_.count;
  }

  gz::transport::Node node_;
  std::mutex mutex_;
  Channel imu_, mag_, baro_, gps_;
  double report_every_s_ = 1.0;
  double last_report_s_ = -1.0;
};

}  // namespace hitl_bridge

GZ_ADD_PLUGIN(hitl_bridge::HitlBridge, gz::sim::System,
              hitl_bridge::HitlBridge::ISystemConfigure,
              hitl_bridge::HitlBridge::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(hitl_bridge::HitlBridge, "hitl_bridge::HitlBridge")
