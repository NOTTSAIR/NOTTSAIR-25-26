#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

using std::placeholders::_1;

class LidarBEV : public rclcpp::Node {
public:
  LidarBEV() : Node("lidar_bev_diag") {
    topic_ = declare_parameter<std::string>("topic", "/velodyne_points");

    // Read params into locals so std::max types match
    int dec_param = declare_parameter<int>("decimate", 3);
    decimate_ = std::max(1, dec_param);

    int sz_param = declare_parameter<int>("scatter_size", 900);
    scatter_size_ = std::max(500, sz_param);

    double hz_param = declare_parameter<double>("refresh_hz", 25.0);
    refresh_hz_ = std::max(1.0, hz_param);

    zoom_          = declare_parameter<double>("zoom_init", 0.6);
    zoom_min_      = 0.1;
    zoom_max_      = 5.0;
    zoom_in_step_  = 0.85;
    zoom_out_step_ = 1.18;

    centre_x_ = 0.0;
    centre_y_ = 0.0;
    centre_locked_ = true;

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS().keep_last(5).best_effort(),
      std::bind(&LidarBEV::onCloud, this, _1));

    int period_ms = std::max(5, (int)std::lround(1000.0 / refresh_hz_));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&LidarBEV::onTimer, this));

    cv::namedWindow("LiDAR BEV diag", cv::WINDOW_NORMAL);
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_ = *msg;
  }

  static double percentile(std::vector<float> v, double q) {
    if (v.empty()) return 10.0;
    std::sort(v.begin(), v.end());
    double idx = q * (v.size() - 1);
    size_t i0 = (size_t)std::floor(idx);
    size_t i1 = (size_t)std::ceil(idx);
    double t = idx - i0;
    return v[i0] * (1.0 - t) + v[i1] * t;
  }

  void onTimer() {
    sensor_msgs::msg::PointCloud2 cloud;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cloud = cloud_;
    }

    cv::Mat canvas(scatter_size_, scatter_size_, CV_8UC3, cv::Scalar(20, 20, 20));
    if (cloud.width == 0 || cloud.height == 0 || cloud.data.empty()) {
      cv::putText(canvas, "Waiting for " + topic_, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  {255,255,255}, 2, cv::LINE_AA);
      cv::imshow("LiDAR BEV diag", canvas);
      handleKeys();
      return;
    }

    // Extract points
    std::vector<float> xs, ys;
    xs.reserve(cloud.width * cloud.height);
    ys.reserve(cloud.width * cloud.height);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
    size_t total = cloud.width * cloud.height;
    for (size_t i = 0; i < total; ++i, ++it_x, ++it_y) {
      if (decimate_ > 1 && (i % decimate_) != 0) continue;
      float xf = *it_x;
      float yl = *it_y;
      if (std::isfinite(xf) && std::isfinite(yl)) {
        xs.push_back(xf);
        ys.push_back(yl);
      }
    }

    if (xs.empty()) {
      cv::putText(canvas, "No valid points", {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  {255,255,255}, 2, cv::LINE_AA);
      cv::imshow("LiDAR BEV diag", canvas);
      handleKeys();
      return;
    }

    // Robust range estimate
    std::vector<float> r; r.reserve(xs.size());
    for (size_t i = 0; i < xs.size(); ++i) r.push_back(std::sqrt(xs[i]*xs[i] + ys[i]*ys[i]));
    double r95 = percentile(r, 0.95);
    base_half_ = std::clamp(r95, 10.0, 80.0);

    if (centre_locked_) { centre_x_ = 0.0; centre_y_ = 0.0; }
    double half = std::max(1.0, base_half_) * zoom_;
    int S = scatter_size_;

    // axes at current centre (image mid)
    cv::line(canvas, {S/2, 0}, {S/2, S-1}, {120,120,120}, 1);
    cv::line(canvas, {0, S/2}, {S-1, S/2}, {120,120,120}, 1);

    // Map points to pixels
    for (size_t i = 0; i < xs.size(); ++i) {
      double x = xs[i];
      double y = ys[i];
      double u = (y - centre_y_ + half) / (2.0 * half); // left positive
      double v = (x - centre_x_ + half) / (2.0 * half); // forward positive
      int c = (int)std::lround(u * (S - 1));
      int rpx = (int)std::lround((1.0 - v) * (S - 1));
      if (c >= 0 && c < S && rpx >= 0 && rpx < S) {
        canvas.at<cv::Vec3b>(rpx, c) = cv::Vec3b(255, 255, 255);
      }
    }

    // HUD
    char hud3[128];
    std::snprintf(hud3, sizeof(hud3), "Zoom %.2f | Centre (%.1f, %.1f) m | View +/- %.1f m",
                  zoom_, centre_x_, centre_y_, half);
    cv::putText(canvas, "Forward (x+)", {10, 22}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {240,240,240}, 2);
    cv::putText(canvas, "Topic: " + topic_, {10, 44}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {240,240,240}, 2);
    cv::putText(canvas, "Keys: q quit, z in, x out, arrows/WASD pan, c centre-lock",
                {10, 66}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {240,240,240}, 2);
    cv::putText(canvas, hud3, {10, 88}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {240,240,240}, 2);

    cv::imshow("LiDAR BEV diag", canvas);
    handleKeys();
  }

  void handleKeys() {
    int key = cv::waitKey(1) & 0xFF;
    double step = 0.1 * std::max(1.0, base_half_) * zoom_;
    if (key == 'q') {
      rclcpp::shutdown();
    } else if (key == 'z' || key == 'Z') {
      zoom_ = std::max(zoom_min_, zoom_ * zoom_in_step_);
    } else if (key == 'x' || key == 'X') {
      zoom_ = std::min(zoom_max_, zoom_ * zoom_out_step_);
    } else if (key == 'c') {
      centre_locked_ = !centre_locked_;
    } else if (key == 'w' || key == 'W') {          // up
      centre_locked_ = false; centre_x_ += step;
    } else if (key == 's' || key == 'S') {          // down
      centre_locked_ = false; centre_x_ -= step;
    } else if (key == 'a' || key == 'A') {          // left
      centre_locked_ = false; centre_y_ -= step;
    } else if (key == 'd' || key == 'D') {          // right
      centre_locked_ = false; centre_y_ += step;
    }
  }

  // params
  std::string topic_;
  int decimate_{3};
  double refresh_hz_{25.0};
  int scatter_size_{900};

  // view state
  double zoom_{0.6}, zoom_min_{0.1}, zoom_max_{5.0}, zoom_in_step_{0.85}, zoom_out_step_{1.18};
  double centre_x_{0.0}, centre_y_{0.0};
  bool centre_locked_{true};
  double base_half_{20.0};

  // cloud and timing
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  sensor_msgs::msg::PointCloud2 cloud_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarBEV>());
  rclcpp::shutdown();
  return 0;
}

