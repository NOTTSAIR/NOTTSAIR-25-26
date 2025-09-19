#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rmw/qos_profiles.h>

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <functional>

using std::placeholders::_1;

class CamFourView : public rclcpp::Node
{
public:
  CamFourView() : Node("camera")
  {
    topics_ = declare_parameter<std::vector<std::string>>(
        "topics",
        {"/zed/left/image_rect_color",
         "/zed/right/image_rect_color",
         "/zed/image_raw",
         "/zed/depth/image_raw"});

    window_title_ = declare_parameter<std::string>("window_title", "camera");
    font_scale_   = declare_parameter<double>("font_scale", 0.8);
    thickness_    = declare_parameter<int>("thickness", 2);

    // Make two subs per topic with different QoS. Whichever matches the publisher will receive.
    rmw_qos_profile_t qos_reliable = rmw_qos_profile_default;
    qos_reliable.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_reliable.durability  = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    rmw_qos_profile_t qos_besteffort = rmw_qos_profile_sensor_data; // best effort + volatile

    for (const auto &t : topics_) {
      subs_.push_back(image_transport::create_subscription(
          this, t,
          std::bind(&CamFourView::onImage, this, _1, t, "reliable"),
          "raw", qos_reliable));
      subs_.push_back(image_transport::create_subscription(
          this, t,
          std::bind(&CamFourView::onImage, this, _1, t, "best_effort"),
          "raw", qos_besteffort));
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(40),   // about 25 Hz GUI refresh
        std::bind(&CamFourView::onTimer, this));

    cv::namedWindow(window_title_, cv::WINDOW_NORMAL);
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
               const std::string &topic,
               const std::string &qos_label)
  {
    try {
      cv::Mat img;
      const auto &enc = msg->encoding;

      if (enc == sensor_msgs::image_encodings::BGR8) {
        img = cv_bridge::toCvShare(msg, enc)->image.clone();
      } else if (enc == sensor_msgs::image_encodings::RGB8) {
        cv::Mat rgb = cv_bridge::toCvShare(msg, enc)->image;
        cv::cvtColor(rgb, img, cv::COLOR_RGB2BGR);
      } else if (enc == sensor_msgs::image_encodings::MONO8) {
        cv::Mat mono = cv_bridge::toCvShare(msg, enc)->image;
        cv::cvtColor(mono, img, cv::COLOR_GRAY2BGR);
      } else if (enc == sensor_msgs::image_encodings::TYPE_32FC1 ||
                 enc == sensor_msgs::image_encodings::TYPE_16UC1) {
        cv::Mat depth = cv_bridge::toCvShare(msg, enc)->image;
        double minv = 0.0, maxv = 0.0;
        cv::minMaxLoc(depth, &minv, &maxv);
        if (!(maxv > minv)) { maxv = minv + 1.0; }
        cv::Mat norm8;
        depth.convertTo(norm8, CV_8U, 255.0 / (maxv - minv), -minv * 255.0 / (maxv - minv));
        cv::cvtColor(norm8, img, cv::COLOR_GRAY2BGR);
      } else {
        // Fallback
        img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_[topic] = img;
        last_qos_[topic] = qos_label;
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "cv_bridge failed for %s: %s", topic.c_str(), e.what());
    }
  }

  void onTimer()
  {
    std::vector<cv::Mat> frames;
    std::vector<std::string> labels;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &t : topics_) {
        auto it = latest_.find(t);
        if (it != latest_.end() && !it->second.empty()) {
          frames.push_back(it->second);
          auto qit = last_qos_.find(t);
          labels.push_back(t + (qit != last_qos_.end() ? " [" + qit->second + "]" : ""));
        }
      }
    }

    if (frames.empty()) {
      cv::Mat img(220, 760, CV_8UC3, cv::Scalar(20,20,20));
      cv::putText(img, "Waiting for image topics...", {20,120},
                  cv::FONT_HERSHEY_SIMPLEX, 0.9, {255,255,255}, 2, cv::LINE_AA);
      cv::imshow(window_title_, img);
      if ((cv::waitKey(1) & 0xFF) == 'q') rclcpp::shutdown();
      return;
    }

    if (frames.size() > 4) {
      frames.resize(4);
      labels.resize(4);
    }

    const int H = 240, W = 320;
    for (auto &f : frames) {
      if (!f.empty() && (f.cols != W || f.rows != H))
        cv::resize(f, f, cv::Size(W, H));
    }

    cv::Mat mosaic(H*2, W*2, CV_8UC3, cv::Scalar(0,0,0));
    for (size_t i = 0; i < frames.size(); ++i) {
      int r = static_cast<int>(i / 2);
      int c = static_cast<int>(i % 2);
      frames[i].copyTo(mosaic(cv::Rect(c*W, r*H, W, H)));
      cv::putText(mosaic, labels[i], {c*W + 8, r*H + 24},
                  cv::FONT_HERSHEY_SIMPLEX, font_scale_, {0,255,0}, thickness_, cv::LINE_AA);
    }

    cv::imshow(window_title_, mosaic);
    if ((cv::waitKey(1) & 0xFF) == 'q') rclcpp::shutdown();
  }

  // params
  std::vector<std::string> topics_;
  std::string window_title_;
  double font_scale_;
  int thickness_;

  // state
  std::vector<image_transport::Subscriber> subs_;
  std::map<std::string, cv::Mat> latest_;
  std::map<std::string, std::string> last_qos_;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CamFourView>());
  rclcpp::shutdown();
  return 0;
}

