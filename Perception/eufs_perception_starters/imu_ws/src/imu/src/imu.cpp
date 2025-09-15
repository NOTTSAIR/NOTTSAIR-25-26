#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <algorithm>

using std::placeholders::_1;

static void quat_to_rpy_deg(double x, double y, double z, double w,
                            double &roll, double &pitch, double &yaw)
{
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

  double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) pitch = std::copysign(M_PI / 2.0, sinp) * 180.0 / M_PI;
  else                        pitch = std::asin(sinp) * 180.0 / M_PI;

  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

class ImuViewer : public rclcpp::Node
{
public:
  ImuViewer() : Node("imu_viewer")
  {
    topic_      = declare_parameter<std::string>("topic", "/imu/data");
    refresh_hz_ = declare_parameter<double>("refresh_hz", 15.0);
    alpha_      = declare_parameter<double>("alpha", 0.15); // smoothing

    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        topic_, rclcpp::SensorDataQoS().keep_last(20).best_effort(),
        std::bind(&ImuViewer::onMsg, this, _1));

    int period_ms = std::max(5, (int)std::round(1000.0 / std::max(5.0, refresh_hz_)));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&ImuViewer::onTimer, this));

    cv::namedWindow("IMU viewer", cv::WINDOW_NORMAL);
    t0_ = now();
  }

private:
  void onMsg(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double r, p, y;
    quat_to_rpy_deg(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w, r, p, y);

    // rad/s -> deg/s
    double wx = msg->angular_velocity.x * 180.0 / M_PI;
    double wy = msg->angular_velocity.y * 180.0 / M_PI;
    double wz = msg->angular_velocity.z * 180.0 / M_PI;

    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    smooth(rpy_[0], r); smooth(rpy_[1], p); smooth(rpy_[2], y);
    smooth(gyro_[0], wx); smooth(gyro_[1], wy); smooth(gyro_[2], wz);
    smooth(acc_[0], ax);  smooth(acc_[1], ay);  smooth(acc_[2], az);

    msg_count_++;
    auto t = now();
    if ((t - t0_) > 0.5) {
      rate_hz_ = msg_count_ / (t - t0_);
      t0_ = t;
      msg_count_ = 0;
    }
  }

  void onTimer()
  {
    const int W = 1000, H = 420;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(20,20,20));

    int y = 28;
    putLine(img, "Topic:  " + topic_, 24, y); y += 28;
    putLine(img, format("Rate:   %.1f Hz", rate_hz_), 24, y); y += 28;
    putLine(img, format("R/P/Y:  %+.1f   %+.1f   %+.1f deg", rpy_[0], rpy_[1], rpy_[2]), 24, y); y += 28;
    putLine(img, format("Gyro :  %+.1f   %+.1f   %+.1f deg/s", gyro_[0], gyro_[1], gyro_[2]), 24, y); y += 28;
    putLine(img, format("Accel:  %+.2f   %+.2f   %+.2f m/s^2", acc_[0], acc_[1], acc_[2]), 24, y); y += 28;

    int top_y = 170;
    bar(img, 24,  top_y, 300, 36, rpy_[0], -180, 180, "roll");
    bar(img, 350, top_y, 300, 36, rpy_[1],  -90,  90, "pitch");
    bar(img, 676, top_y, 300, 36, rpy_[2], -180, 180, "yaw");

    cv::imshow("IMU viewer", img);
    int k = cv::waitKey(1) & 0xFF;
    if (k == 'q') rclcpp::shutdown();
  }

  static std::string format(const char *fmt, double a, double b=0, double c=0)
  {
    char buf[128]; std::snprintf(buf, sizeof(buf), fmt, a, b, c);
    return std::string(buf);
  }

  void putLine(cv::Mat &img, const std::string &txt, int x, int y)
  {
    cv::putText(img, txt, {x,y}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,0,0}, 4, cv::LINE_AA);
    cv::putText(img, txt, {x,y}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255,255,255}, 2, cv::LINE_AA);
  }

  void bar(cv::Mat &img, int x, int y, int w, int h, double v, double vmin, double vmax, const std::string &label)
  {
    cv::rectangle(img, {x,y}, {x+w,y+h}, {60,60,60}, -1);
    cv::rectangle(img, {x,y}, {x+w,y+h}, {220,220,220}, 2);
    cv::line(img, {x+w/2,y}, {x+w/2,y+h}, {120,120,120}, 1);
    double t = (v - vmin) / std::max(1e-6, vmax - vmin);
    t = std::clamp(t, 0.0, 1.0);
    int px = x + int(t * w);
    cv::rectangle(img, {x,y}, {px,y+h}, {90,90,200}, -1);
    cv::putText(img, label + ": " + format("%+.1f", v), {x+8, y-8},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,255,255}, 2, cv::LINE_AA);
  }

  double now() { return this->get_clock()->now().seconds(); }
  void smooth(double &s, double x) { s = (1.0 - alpha_) * s + alpha_ * x; }

  // params
  std::string topic_;
  double refresh_hz_;
  double alpha_;

  // state
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  double rpy_[3]{0,0,0};
  double gyro_[3]{0,0,0};
  double acc_[3]{0,0,0};
  double rate_hz_{0.0};
  double t0_{0.0};
  int msg_count_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuViewer>());
  rclcpp::shutdown();
  return 0;
}

