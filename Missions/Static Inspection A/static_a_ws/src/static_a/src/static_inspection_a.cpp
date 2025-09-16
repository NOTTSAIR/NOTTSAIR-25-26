#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <eufs_msgs/msg/can_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

using ackermann_msgs::msg::AckermannDriveStamped;
using eufs_msgs::msg::CanState;
using std_msgs::msg::Bool;
using std_msgs::msg::String;
using namespace std::chrono_literals;

static constexpr int AMI_DDT_INSPECTION_A = 18;

class StaticInspectionA : public rclcpp::Node {
public:
  StaticInspectionA() : Node("eufs_static_a")
  {
    // ---- parameters (defaults mean no CLI args) ----
    topic_            = declare_parameter<std::string>("topic", "/cmd");
    steer_amp_rad_    = declare_parameter<double>("steer_amp_rad", 0.6);
    t_sweep_s_        = declare_parameter<double>("t_sweep_s", 6.0);
    t_accel_s_        = declare_parameter<double>("t_accel_s", 10.0);
    t_decel_s_        = declare_parameter<double>("t_decel_s", 5.0);
    rate_hz_          = declare_parameter<double>("rate_hz", 50.0);
    auto_set_mission_ = declare_parameter<bool>("auto_set_mission", true);
    accel_up_         = declare_parameter<double>("accel_up", 1.0);     // m/s^2
    accel_down_       = declare_parameter<double>("accel_down", -2.0);  // m/s^2 (neg)

    // ---- pubs ----
    cmd_pub_               = create_publisher<AckermannDriveStamped>(topic_, 10);
    mission_pub_           = create_publisher<CanState>("/ros_can/set_mission", 1);
    mission_completed_pub_ = create_publisher<Bool>("/ros_can/mission_completed", 1);

    // ---- subs ----
    state_sub_ = create_subscription<CanState>(
      "/ros_can/state", 10,
      [this](const CanState::SharedPtr msg){ last_state_ = *msg; have_state_ = true; });

    driving_flag_sub_ = create_subscription<Bool>(
      "/state_machine/driving_flag", 10,
      [this](const Bool::SharedPtr b){ have_driving_flag_ = true; driving_flag_ = b->data; });

    state_str_sub_ = create_subscription<String>(
      "/ros_can/state_str", 10,
      [this](const String::SharedPtr s){
        have_state_str_ = true;
        // consider any string containing "DRIVING" as the go-signal
        is_driving_str_ = (s->data.find("DRIVING") != std::string::npos);
      });

    // timer
    dt_ = 1.0 / rate_hz_;
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 * dt_)),
      std::bind(&StaticInspectionA::tick, this));

    RCLCPP_INFO(get_logger(),
      "Static A (ACCEL mode): topic=%s | a_up=%.2f a_down=%.2f",
      topic_.c_str(), accel_up_, accel_down_);

    // Request mission once (optional). We still WAIT until we SEE the state.
    if (auto_set_mission_) {
      CanState req; req.ami_state = AMI_DDT_INSPECTION_A;
      mission_pub_->publish(req);
      RCLCPP_INFO(get_logger(), "Requested AMI_DDT_INSPECTION_A (%d)", AMI_DDT_INSPECTION_A);
    }
  }

private:
  // triangular steering: 0→+A→−A→0 across t_sweep_s_
  double steeringProfile(double t) const {
    const double T = t_sweep_s_, A = steer_amp_rad_;
    if (t <= 0.0 || t >= T) return 0.0;
    const double seg = T / 3.0;
    if (t < seg)        return (A / seg) * t;
    if (t < 2*seg)      return A - (2*A/seg) * (t - seg);
    return -A + (A/seg) * (t - 2*seg);
  }

  // accel profile [m/s^2]
  double accelProfile(double t) const {
    const double T1 = t_sweep_s_;
    const double T2 = T1 + t_accel_s_;
    const double T3 = T2 + t_decel_s_;
    if (t < T1) return 0.0;
    if (t < T2) return accel_up_;
    if (t < T3) return accel_down_;
    return 0.0;
  }

  // Gate opens when mission is DDT_INSPECTION_A AND (driving_flag true OR state_str says DRIVING)
  bool gateOpen() const {
    const bool ami_ok = have_state_ && (last_state_.ami_state == AMI_DDT_INSPECTION_A);
    const bool drive_ok =
      (have_driving_flag_ && driving_flag_) ||
      (have_state_str_   && is_driving_str_);
    return ami_ok && drive_ok;
  }

  void tick() {
    // keep sending zeros until the state machine is ready
    if (!gate_started_) {
      if (gateOpen()) {
        gate_started_ = true;
        start_time_ = nowSec();
        RCLCPP_INFO(get_logger(), "Gate open: AMI=DDT_INSPECTION_A and DRIVING. Starting Static A.");
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for AMI=DDT_INSPECTION_A and AS=DRIVING...");
        send(0.0, 0.0, 0.0);
        return;
      }
    }

    const double t = nowSec() - start_time_;
    const double t_end = t_sweep_s_ + t_accel_s_ + t_decel_s_;

    const double steer = steeringProfile(t);
    const double accel = accelProfile(t);
    send(0.0, steer, accel); // speed ignored in accel mode

    if (!completed_ && t >= t_end) {
      completed_ = true;
      Bool done; done.data = true;
      mission_completed_pub_->publish(done);
      RCLCPP_INFO(get_logger(), "Static A complete -> /ros_can/mission_completed TRUE");
    }
  }

  void send(double v, double steer, double accel) {
    AckermannDriveStamped m;
    m.header.stamp = now();
    m.header.frame_id = "base_link";
    m.drive.steering_angle = static_cast<float>(steer);
    m.drive.acceleration   = static_cast<float>(accel);
    m.drive.speed          = static_cast<float>(v); // ignored in accel mode
    cmd_pub_->publish(m);
  }

  double nowSec() const { return this->now().seconds(); }

  // ---- params ----
  std::string topic_;
  double steer_amp_rad_{0.6};
  double t_sweep_s_{6.0}, t_accel_s_{10.0}, t_decel_s_{5.0}, rate_hz_{50.0};
  bool auto_set_mission_{true};
  double accel_up_{1.0}, accel_down_{-2.0};

  // ---- state ----
  double dt_{0.0};
  double start_time_{0.0};
  bool gate_started_{false};
  bool completed_{false};

  bool have_state_{false};
  bool have_driving_flag_{false};
  bool have_state_str_{false};
  bool driving_flag_{false};
  bool is_driving_str_{false};
  CanState last_state_{};

  // ROS I/O
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<CanState>::SharedPtr mission_pub_;
  rclcpp::Publisher<Bool>::SharedPtr mission_completed_pub_;
  rclcpp::Subscription<CanState>::SharedPtr state_sub_;
  rclcpp::Subscription<Bool>::SharedPtr driving_flag_sub_;
  rclcpp::Subscription<String>::SharedPtr state_str_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticInspectionA>());
  rclcpp::shutdown();
  return 0;
}

