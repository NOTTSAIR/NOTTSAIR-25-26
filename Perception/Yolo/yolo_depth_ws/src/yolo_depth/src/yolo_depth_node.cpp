// src/yolo_depth_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>

#include <torch/torch.h>
#include <torch/script.h>

#include <rmw/qos_profiles.h>

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>

using sensor_msgs::msg::Image;
using vision_msgs::msg::Detection2DArray;
using vision_msgs::msg::Detection2D;
using vision_msgs::msg::ObjectHypothesisWithPose;

struct LetterboxInfo {
  float scale{1.f};
  int pad_w{0};
  int pad_h{0};
};

static LetterboxInfo letterbox(const cv::Mat &src, cv::Mat &dst, int new_w, int new_h) {
  const int w = src.cols, h = src.rows;
  const float r = std::min(static_cast<float>(new_w) / w,
                           static_cast<float>(new_h) / h);
  const int unpad_w = static_cast<int>(std::round(w * r));
  const int unpad_h = static_cast<int>(std::round(h * r));
  cv::resize(src, dst, cv::Size(unpad_w, unpad_h));
  const int pad_w = new_w - unpad_w;
  const int pad_h = new_h - unpad_h;
  const int top = pad_h / 2, bottom = pad_h - top;
  const int left = pad_w / 2, right = pad_w - left;
  cv::copyMakeBorder(dst, dst, top, bottom, left, right,
                     cv::BORDER_CONSTANT, cv::Scalar(114,114,114));
  return {r, left, top};
}

enum class ConeColour { YELLOW, BLUE, UNKNOWN };

static ConeColour classify_colour_bgr(const cv::Mat &bgr_roi,
                                      double min_fraction = 0.10,
                                      int shrink_px = 4) {
  if (bgr_roi.empty()) return ConeColour::UNKNOWN;

  cv::Rect inner(shrink_px, shrink_px,
                 std::max(0, bgr_roi.cols - 2 * shrink_px),
                 std::max(0, bgr_roi.rows - 2 * shrink_px));
  if (inner.width <= 0 || inner.height <= 0) return ConeColour::UNKNOWN;

  cv::Mat roi = bgr_roi(inner).clone();
  cv::Mat hsv; cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

  // H in [0,179], S,V in [0,255]
  cv::Mat mask_y, mask_b;
  cv::inRange(hsv, cv::Scalar(15, 60, 60),  cv::Scalar(35, 255, 255), mask_y);   // yellow
  cv::inRange(hsv, cv::Scalar(90, 60, 60),  cv::Scalar(130,255,255), mask_b);    // blue

  const double total = static_cast<double>(roi.total());
  if (total <= 0.0) return ConeColour::UNKNOWN;

  const double fy = static_cast<double>(cv::countNonZero(mask_y)) / total;
  const double fb = static_cast<double>(cv::countNonZero(mask_b)) / total;

  if (fy < min_fraction && fb < min_fraction) return ConeColour::UNKNOWN;
  return (fy >= fb) ? ConeColour::YELLOW : ConeColour::BLUE;
}

// normalise various Ultralytics TorchScript layouts to [N, M] on CPU
static bool normalise_detections_tensor(torch::Tensor out, torch::Tensor &dets_out) {
  if (out.dim() == 3 && out.size(0) == 1) out = out.squeeze(0);
  if (out.dim() != 2) return false;
  if (out.size(0) == 5) out = out.transpose(0,1).contiguous(); // 5 x N -> N x 5
  dets_out = out.to(torch::kCPU).contiguous();
  return true;
}

class YoloDepthNode : public rclcpp::Node {
public:
  YoloDepthNode() : Node("yolo_depth_node") {
    // Parameters
    rgb_topic_        = declare_parameter<std::string>("rgb_topic", "/zed/left/image_rect_color");
    depth_topic_      = declare_parameter<std::string>("depth_topic", "/zed/depth/image_raw");
    annotated_topic_  = declare_parameter<std::string>("annotated_image_topic", "/cones/annotated_depth");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/cones/detections");

    model_path_       = declare_parameter<std::string>("model_path", "best.torchscript");
    inp_w_            = declare_parameter<int>("input_width", 640);
    inp_h_            = declare_parameter<int>("input_height", 640);
    conf_thr_         = declare_parameter<double>("conf_threshold", 0.35);
    nms_thr_          = declare_parameter<double>("nms_threshold", 0.45);

    min_colour_fraction_ = declare_parameter<double>("min_colour_fraction", 0.10);
    shrink_px_           = declare_parameter<int>("colour_shrink_px", 4);

    use_cuda_         = declare_parameter<bool>("use_cuda", true);
    draw_labels_      = declare_parameter<bool>("draw_labels", true);
    draw_boxes_       = declare_parameter<bool>("draw_boxes", true);
    window_title_     = declare_parameter<std::string>("window_title", "YOLO Depth POC");

    sample_mode_     = declare_parameter<std::string>("sample_mode", "grid"); // centre or grid
    grid_n_          = declare_parameter<int>("grid_n", 5);
    inner_ratio_     = declare_parameter<double>("inner_ratio", 0.4);
    min_valid_frac_  = declare_parameter<double>("min_valid_frac", 0.2);
    min_range_m_     = declare_parameter<double>("min_range_m", 0.5);
    max_range_m_     = declare_parameter<double>("max_range_m", 60.0);

    // Device and model
    if (use_cuda_ && torch::cuda::is_available()) {
      device_ = torch::kCUDA;
      RCLCPP_INFO(get_logger(), "Using CUDA");
    } else {
      device_ = torch::kCPU;
      if (use_cuda_) RCLCPP_WARN(get_logger(), "CUDA requested but not available, using CPU");
      else RCLCPP_INFO(get_logger(), "Using CPU");
    }

    try {
      module_ = torch::jit::load(model_path_, device_);
      module_.eval();
    } catch (const c10::Error &e) {
      RCLCPP_FATAL(get_logger(), "Failed to load TorchScript model '%s': %s",
                   model_path_.c_str(), e.what());
      throw;
    }

    // Publishers
    img_pub_ = create_publisher<Image>(annotated_topic_, 10);
    det_pub_ = create_publisher<Detection2DArray>(detections_topic_, 10);

    // Subscribers, sensor_data QoS to match camera streams
    sub_rgb_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
    sub_depth_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);

    using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(20), sub_rgb_, sub_depth_);
    sync_->registerCallback(std::bind(&YoloDepthNode::onSynced, this,
                                      std::placeholders::_1, std::placeholders::_2));

    cv::namedWindow(window_title_, cv::WINDOW_NORMAL);
    RCLCPP_INFO(get_logger(), "Subscribed to %s and %s",
                rgb_topic_.c_str(), depth_topic_.c_str());
  }

  ~YoloDepthNode() override { cv::destroyWindow(window_title_); }

private:
  static bool read_depth_as_32f(const Image::ConstSharedPtr &msg, cv::Mat &depth32) {
    try {
      if (msg->encoding == "32FC1" || msg->encoding == "passthrough") {
        depth32 = cv_bridge::toCvShare(msg, "32FC1")->image;
        return true;
      }
      if (msg->encoding == "16UC1") {
        cv::Mat d16 = cv_bridge::toCvShare(msg, "16UC1")->image;
        d16.convertTo(depth32, CV_32F, 1.0 / 1000.0); // mm -> m
        return true;
      }
      // Try to force convert if vendor encoding
      cv::Mat any = cv_bridge::toCvShare(msg)->image;
      if (any.type() == CV_16UC1) { any.convertTo(depth32, CV_32F, 1.0/1000.0); return true; }
      if (any.type() == CV_32FC1) { depth32 = any; return true; }
    } catch (const std::exception &) {
      return false;
    }
    return false;
  }

  double sample_range_from_bbox_depth(const cv::Mat &depth_m, const cv::Rect &bb) const {
    cv::Rect inner = bb;
    const int shrink_w = static_cast<int>(bb.width  * (1.0 - inner_ratio_) * 0.5);
    const int shrink_h = static_cast<int>(bb.height * (1.0 - inner_ratio_) * 0.5);
    inner.x += shrink_w; inner.y += shrink_h;
    inner.width  = std::max(1, bb.width  - 2 * shrink_w);
    inner.height = std::max(1, bb.height - 2 * shrink_h);
    inner &= cv::Rect(0, 0, depth_m.cols, depth_m.rows);

    std::vector<double> vals; vals.reserve(grid_n_ * grid_n_);
    if (sample_mode_ == "centre") {
      const int u = inner.x + inner.width / 2;
      const int v = inner.y + inner.height / 2;
      const float z = depth_m.at<float>(v, u);
      if (std::isfinite(z) && z > 0.f && z >= min_range_m_ && z <= max_range_m_) vals.push_back(z);
    } else {
      for (int gy = 0; gy < grid_n_; ++gy) {
        for (int gx = 0; gx < grid_n_; ++gx) {
          int u = inner.x + static_cast<int>((gx + 0.5) * inner.width  / grid_n_);
          int v = inner.y + static_cast<int>((gy + 0.5) * inner.height / grid_n_);
          const float z = depth_m.at<float>(v, u);
          if (std::isfinite(z) && z > 0.f && z >= min_range_m_ && z <= max_range_m_) {
            vals.push_back(z);
          }
        }
      }
    }
    const size_t need = static_cast<size_t>(std::ceil(min_valid_frac_ * grid_n_ * grid_n_));
    if (vals.size() < std::max<size_t>(1, need)) return std::numeric_limits<double>::quiet_NaN();

    std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
    return vals[vals.size() / 2]; // median range in metres
  }

  void onSynced(const Image::ConstSharedPtr &rgb_msg,
                const Image::ConstSharedPtr &depth_msg)
  {
    // Convert inputs
    cv::Mat bgr, depth_m;
    try {
      if (rgb_msg->encoding == "bgr8") {
        bgr = cv_bridge::toCvShare(rgb_msg, "bgr8")->image.clone();
      } else if (rgb_msg->encoding == "rgb8") {
        cv::Mat rgb = cv_bridge::toCvShare(rgb_msg, "rgb8")->image;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      } else if (rgb_msg->encoding == "mono8") {
        cv::Mat mono = cv_bridge::toCvShare(rgb_msg, "mono8")->image;
        cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
      } else {
        bgr = cv_bridge::toCvShare(rgb_msg, "bgr8")->image.clone();
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge RGB error: %s", e.what());
      return;
    }

    if (!read_depth_as_32f(depth_msg, depth_m)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read depth image as 32FC1 or 16UC1");
      return;
    }
    if (bgr.empty() || depth_m.empty()) return;

    // Prepare input tensor: BGR -> RGB float [0,1], NCHW
    cv::Mat resized, rgb;
    LetterboxInfo lb = letterbox(bgr, resized, inp_w_, inp_h_);
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);
    auto input = torch::from_blob(rgb.data, {1, inp_h_, inp_w_, 3}, torch::kFloat)
                   .permute({0,3,1,2}).to(device_);

    // YOLO forward
    torch::Tensor out;
    try {
      std::vector<torch::jit::IValue> inputs; inputs.emplace_back(input);
      torch::NoGradGuard no_grad;
      auto iv = module_.forward(inputs);
      if (iv.isTensor()) out = iv.toTensor();
      else if (iv.isTuple()) out = iv.toTuple()->elements()[0].toTensor();
      else if (iv.isList())  out = iv.toList().get(0).toTensor();
      else { RCLCPP_ERROR(get_logger(), "Unexpected model output type"); return; }
    } catch (const c10::Error &e) {
      RCLCPP_FATAL(get_logger(),
        "Model forward failed. Re export TorchScript matching imgsz %dx%d. Error: %s",
        inp_w_, inp_h_, e.what());
      return;
    }

    // Normalise detections to [N, M] on CPU
    torch::Tensor dets;
    if (!normalise_detections_tensor(out, dets)) {
      RCLCPP_ERROR(get_logger(), "Could not normalise model output");
      return;
    }

    const int M = static_cast<int>(dets.size(1));
    const bool single_class_5 = (M == 5);  // xywh + score
    const bool multi_class    = (M >= 6);  // xywh + obj + K scores

    auto A = dets.accessor<float,2>();
    const int N = static_cast<int>(dets.size(0));

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    boxes.reserve(N); scores.reserve(N);

    for (int i = 0; i < N; ++i) {
      const float cx = A[i][0];
      const float cy = A[i][1];
      const float w  = A[i][2];
      const float h  = A[i][3];

      float score = 0.f;
      if (single_class_5) {
        score = A[i][4];
      } else if (multi_class) {
        const float obj = A[i][4];
        float best = 0.f;
        for (int c = 5; c < M; ++c) best = std::max(best, A[i][c]);
        score = obj * best;
      } else {
        continue;
      }
      if (score < conf_thr_) continue;

      // Map back from letterbox to original image coords
      const float x = (cx - w/2.f - lb.pad_w) / lb.scale;
      const float y = (cy - h/2.f - lb.pad_h) / lb.scale;
      const float bw = w / lb.scale;
      const float bh = h / lb.scale;

      const int l = std::max(0, static_cast<int>(std::round(x)));
      const int t = std::max(0, static_cast<int>(std::round(y)));
      const int r = std::min(bgr.cols - 1, static_cast<int>(std::round(x + bw)));
      const int b = std::min(bgr.rows - 1, static_cast<int>(std::round(y + bh)));
      const int ww = std::max(0, r - l);
      const int hh = std::max(0, b - t);
      if (ww <= 0 || hh <= 0) continue;

      boxes.emplace_back(l, t, ww, hh);
      scores.emplace_back(score);
    }

    // NMS
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(conf_thr_),
                      static_cast<float>(nms_thr_), keep);

    // Draw and publish
    cv::Mat annotated = bgr.clone();
    Detection2DArray det_array; det_array.header = rgb_msg->header;

    for (int idx : keep) {
      const cv::Rect &bb = boxes[idx];
      const cv::Rect clip = bb & cv::Rect(0, 0, bgr.cols, bgr.rows);

      // Optional colour classification on crop
      ConeColour colour = classify_colour_bgr(bgr(clip), min_colour_fraction_, shrink_px_);
      std::string name;
      cv::Scalar col;
      if (colour == ConeColour::YELLOW) { name = "Yellow cone"; col = cv::Scalar(0,255,255); }
      else if (colour == ConeColour::BLUE){ name = "Blue cone";   col = cv::Scalar(255,128,0); }
      else { name = "Cone"; col = cv::Scalar(0,255,0); }

      // Sample depth inside bbox to get range in metres
      double range_m = sample_range_from_bbox_depth(depth_m, bb);

      // Caption
      char text[128];
      if (std::isfinite(range_m))
        std::snprintf(text, sizeof(text), "%s %.2f m", name.c_str(), range_m);
      else
        std::snprintf(text, sizeof(text), "%s ?", name.c_str());

      // Draw
      if (draw_boxes_) {
        cv::rectangle(annotated, bb, col, 2);
        if (draw_labels_) {
          int base = 0;
          cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &base);
          int x = std::max(0, bb.x);
          int y = std::max(0, bb.y - ts.height - 6);
          // background box
          int rect_w = ts.width + 6;
          int rect_h = ts.height + 6;
          int rect_x = std::min(std::max(0, x), std::max(0, annotated.cols - rect_w));
          int rect_y = std::min(std::max(0, y), std::max(0, annotated.rows - rect_h));
          cv::rectangle(annotated, cv::Rect(rect_x, rect_y, rect_w, rect_h), col, cv::FILLED);

          // text origin (clamped)
          cv::Point org(std::max(rect_x + 3, 0), std::max(rect_y + ts.height, 0));
          cv::putText(annotated, text, org,
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 2, cv::LINE_AA);
        }
      }

      // Also publish 2D detections
      Detection2D det;
      det.header = rgb_msg->header;
      det.bbox.size_x = bb.width;
      det.bbox.size_y = bb.height;
      det.bbox.center.x = bb.x + bb.width  / 2.0;
      det.bbox.center.y = bb.y + bb.height / 2.0;
      det.bbox.center.theta = 0.0;

      ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = name;
      hyp.hypothesis.score    = scores[idx];
      det.results.push_back(hyp);
      det_array.detections.push_back(det);
    }

    img_pub_->publish(*cv_bridge::CvImage(rgb_msg->header, "bgr8", annotated).toImageMsg());
    det_pub_->publish(det_array);

    cv::imshow(window_title_, annotated);
    if ((cv::waitKey(1) & 0xFF) == 'q') rclcpp::shutdown();
  }

  // Parameters
  std::string rgb_topic_, depth_topic_;
  std::string annotated_topic_, detections_topic_, window_title_, model_path_;
  int inp_w_{640}, inp_h_{640};
  double conf_thr_{0.35}, nms_thr_{0.45};
  double min_colour_fraction_{0.10};
  int shrink_px_{4};
  bool use_cuda_{true}, draw_labels_{true}, draw_boxes_{true};

  std::string sample_mode_;
  int grid_n_;
  double inner_ratio_, min_valid_frac_, min_range_m_, max_range_m_;

  // State
  torch::jit::script::Module module_;
  torch::Device device_{torch::kCPU};
  rclcpp::Publisher<Image>::SharedPtr img_pub_;
  rclcpp::Publisher<Detection2DArray>::SharedPtr det_pub_;

  message_filters::Subscriber<Image> sub_rgb_;
  message_filters::Subscriber<Image> sub_depth_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<Image, Image>>> sync_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloDepthNode>());
  rclcpp::shutdown();
  return 0;
}
