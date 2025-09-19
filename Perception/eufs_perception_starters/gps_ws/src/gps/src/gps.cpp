#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <deque>
#include <string>
#include <algorithm>

using std::placeholders::_1;

struct TimedPoint { double x{}, y{}, t{}; };

static double now_sec(rclcpp::Node *n) { return n->get_clock()->now().seconds(); }

// Convert lat/lon → local x,y (north/east) relative to origin
static void lla_to_xy(double lat0, double lon0, double lat, double lon, double &x, double &y) {
  const double R = 6378137.0;
  double lat0r = lat0 * M_PI / 180.0;
  double dlat  = (lat - lat0) * M_PI / 180.0;
  double dlon  = (lon - lon0) * M_PI / 180.0;
  double east  = R * dlon * std::cos(lat0r);
  double north = R * dlat;
  x = north;
  y = -east;  // flip so east = left
}

class GpsViewer : public rclcpp::Node {
public:
  GpsViewer() : Node("gps_viewer") {
    topic_        = declare_parameter<std::string>("topic", "/gps");
    refresh_hz_   = declare_parameter<double>("refresh_hz", 15.0);
    history_secs_ = declare_parameter<double>("history_secs", 120.0);
    max_points_   = declare_parameter<int>("max_points", 20000);
    smooth_alpha_ = declare_parameter<double>("smooth_alpha", 0.2); // smoothing factor (0–1)

    auto qos = rclcpp::SensorDataQoS().best_effort();
    sub_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        topic_, qos, std::bind(&GpsViewer::onFix, this, _1));

    int period_ms = std::max(5, (int)std::lround(1000.0 / std::max(5.0, refresh_hz_)));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&GpsViewer::onTimer, this));

    cv::namedWindow("GPS Viewer", cv::WINDOW_NORMAL);
    RCLCPP_INFO(get_logger(), "Subscribing to %s [NavSatFix, BestEffort].", topic_.c_str());
  }

private:
  void onFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) return;

    if (!have_origin_) {
      origin_lat_ = msg->latitude;
      origin_lon_ = msg->longitude;
      have_origin_ = true;
      filtered_x_ = 0.0;
      filtered_y_ = 0.0;
      RCLCPP_INFO(get_logger(), "Origin set (%.8f, %.8f) as (0,0)", origin_lat_, origin_lon_);
    }

    double raw_x, raw_y;
    lla_to_xy(origin_lat_, origin_lon_, msg->latitude, msg->longitude, raw_x, raw_y);

    // Apply exponential moving average for smoothing
    filtered_x_ = smooth_alpha_ * raw_x + (1.0 - smooth_alpha_) * filtered_x_;
    filtered_y_ = smooth_alpha_ * raw_y + (1.0 - smooth_alpha_) * filtered_y_;

    TimedPoint p{filtered_x_, filtered_y_, now_sec(this)};
    trail_.push_back(p);
    if ((int)trail_.size() > max_points_) trail_.pop_front();
  }

  void onTimer() {
    const int S = 800;
    cv::Mat img(S, S, CV_8UC3, cv::Scalar(20,20,20));

    if (!have_origin_ || trail_.empty()) {
      cv::putText(img, "Waiting for " + topic_, {20,40},
                  cv::FONT_HERSHEY_SIMPLEX, 0.9, {255,255,255}, 2);
      cv::imshow("GPS Viewer", img);
      cv::waitKey(1);
      return;
    }

    TimedPoint latest = trail_.back();

    // auto-scale (95th percentile distance)
    std::vector<double> dx, dy;
    dx.reserve(trail_.size()); dy.reserve(trail_.size());
    for (auto &p : trail_) { dx.push_back(std::abs(p.x)); dy.push_back(std::abs(p.y)); }
    auto percentile = [](std::vector<double> v, double q){
      if (v.empty()) return 10.0;
      std::sort(v.begin(), v.end());
      double idx = q * (v.size() - 1);
      size_t i0 = (size_t)std::floor(idx), i1 = (size_t)std::ceil(idx);
      double t = idx - i0;
      return v[i0]*(1.0 - t) + v[i1]*t;
    };
    double half = std::max(10.0, std::max(percentile(dx,0.95), percentile(dy,0.95)));

    auto to_px = [&](double xf, double yl){
      double u = ((yl + half) / (2*half)) * (S - 1);
      double v = ((xf + half) / (2*half)) * (S - 1);
      int c = std::clamp((int)std::lround(u), 0, S-1);
      int r = std::clamp((int)std::lround((S - 1) - v), 0, S-1);
      return cv::Point(c, r);
    };

    // draw trail with fading alpha
    double tnow = now_sec(this);
    for (auto &p : trail_) {
      double age = tnow - p.t;
      double a = std::clamp(1.0 - age / history_secs_, 0.2, 1.0);
      cv::circle(img, to_px(p.x, p.y), 2, cv::Scalar(0, 255*a, 255), -1);
    }

    // highlight current position
    cv::circle(img, to_px(latest.x, latest.y), 8, {0,255,0}, -1);
    cv::circle(img, to_px(latest.x, latest.y), 12, {0,0,0}, 2);

    // axes
    cv::line(img, {S/2,0}, {S/2,S-1}, {120,120,120}, 1);
    cv::line(img, {0,S/2}, {S-1,S/2}, {120,120,120}, 1);

    // HUD
    cv::putText(img, "GPS Viewer (smoothed, origin=0,0)", {10,26},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);
    cv::putText(img, "Latest: x=" + std::to_string(latest.x) +
                        " m, y=" + std::to_string(latest.y) + " m", {10,52},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);

    cv::imshow("GPS Viewer", img);
    if ((cv::waitKey(1) & 0xFF) == 'q') rclcpp::shutdown();
  }

  // Parameters
  std::string topic_;
  double refresh_hz_{15.0}, history_secs_{120.0};
  int max_points_{20000};
  double smooth_alpha_{0.2}; // smoothing factor (closer to 0 = smoother)

  // Origin
  bool have_origin_{false};
  double origin_lat_{0.0}, origin_lon_{0.0};
  double filtered_x_{0.0}, filtered_y_{0.0};

  std::deque<TimedPoint> trail_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsViewer>());
  rclcpp::shutdown();
  return 0;
}

