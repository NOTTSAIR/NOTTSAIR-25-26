#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <opencv2/opencv.hpp>

#include <torch/torch.h>
#include <torch/script.h>

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

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
  // Seen in the wild:
  //  (1, N, 5+K)  or (N, 5+K)        xywh + obj + K class scores
  //  (1, 5, N)    or (5, N)          single class: xywh + score
  //  sometimes (N, 5)                single class: xywh + score
  if (out.dim() == 3 && out.size(0) == 1) out = out.squeeze(0);
  if (out.dim() != 2) return false;

  // transpose if 5 x N
  if (out.size(0) == 5) out = out.transpose(0,1).contiguous();

  dets_out = out.to(torch::kCPU).contiguous();
  return true;
}

class YoloNode : public rclcpp::Node {
public:
  YoloNode() : Node("yolo_node") {
    // parameters
    input_topic_       = declare_parameter<std::string>("input_image_topic", "/image_raw");
    annotated_topic_   = declare_parameter<std::string>("annotated_image_topic", "/yolo/image_annotated");
    detections_topic_  = declare_parameter<std::string>("detections_topic", "/yolo/detections");
    window_title_      = declare_parameter<std::string>("window_title", "YOLO Cones");

    model_path_        = declare_parameter<std::string>("model_path", "best.torchscript");
    inp_w_             = declare_parameter<int>("input_width", 640);
    inp_h_             = declare_parameter<int>("input_height", 640);
    conf_thr_          = declare_parameter<double>("conf_threshold", 0.35);
    nms_thr_           = declare_parameter<double>("nms_threshold", 0.45);

    min_colour_fraction_ = declare_parameter<double>("min_colour_fraction", 0.10);
    shrink_px_           = declare_parameter<int>("colour_shrink_px", 4);

    use_cuda_          = declare_parameter<bool>("use_cuda", true);
    qos_best_effort_   = declare_parameter<bool>("qos_best_effort", true);
    draw_labels_       = declare_parameter<bool>("draw_labels", true);
    draw_boxes_        = declare_parameter<bool>("draw_boxes", true);

    // choose device first
    if (use_cuda_ && torch::cuda::is_available()) {
      device_ = torch::kCUDA;
      RCLCPP_INFO(get_logger(), "Using CUDA");
    } else {
      device_ = torch::kCPU;
      if (use_cuda_) RCLCPP_ERROR(get_logger(), "CUDA requested but not available");
      else RCLCPP_INFO(get_logger(), "Using CPU");
    }

    // load model ON the chosen device
    try {
      module_ = torch::jit::load(model_path_, device_);
      module_.eval();
    } catch (const c10::Error &e) {
      RCLCPP_FATAL(get_logger(), "Failed to load TorchScript model '%s': %s",
                   model_path_.c_str(), e.what());
      throw;
    }

    // QoS and pubs/subs
    rclcpp::QoS qos(10);
    if (qos_best_effort_) qos.best_effort();

    img_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, 10);
    det_pub_ = create_publisher<Detection2DArray>(detections_topic_, 10);

    img_sub_ = image_transport::create_subscription(
      this, input_topic_,
      std::bind(&YoloNode::onImage, this, std::placeholders::_1),
      "raw", qos.get_rmw_qos_profile()
    );

    cv::namedWindow(window_title_, cv::WINDOW_NORMAL);
    RCLCPP_INFO(get_logger(), "Subscribed to %s", input_topic_.c_str());
  }

  ~YoloNode() override { cv::destroyWindow(window_title_); }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    // convert to BGR
    cv::Mat bgr;
    try {
      if (msg->encoding == "bgr8") {
        bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
      } else if (msg->encoding == "rgb8") {
        cv::Mat rgb = cv_bridge::toCvShare(msg, "rgb8")->image;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == "mono8") {
        cv::Mat mono = cv_bridge::toCvShare(msg, "mono8")->image;
        cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
      } else {
        bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    if (bgr.empty()) return;
    processFrame(bgr, msg->header);
  }

  void processFrame(const cv::Mat &bgr, const std_msgs::msg::Header &hdr) {
    // letterbox
    cv::Mat resized;
    LetterboxInfo lb = letterbox(bgr, resized, inp_w_, inp_h_);

    // BGR -> RGB float [0,1], NCHW, move to device
    cv::Mat rgb; cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);
    auto input = torch::from_blob(rgb.data, {1, inp_h_, inp_w_, 3}, torch::kFloat)
                   .permute({0,3,1,2}).to(device_);

    // forward
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
        "Model forward failed. If you changed input_width/height, re export TorchScript with the same imgsz.\n"
        "Node imgsz: %dx%d. Error: %s", inp_w_, inp_h_, e.what());
      return;
    }

    // normalise to [N, M] on CPU for parsing
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

      // map back from letterbox
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

    // outputs
    cv::Mat annotated = bgr.clone();
    Detection2DArray det_array; det_array.header = hdr;

    for (int idx : keep) {
      const cv::Rect &bb = boxes[idx];
      const cv::Rect clip = bb & cv::Rect(0, 0, bgr.cols, bgr.rows);

      // colour post classification
      ConeColour colour = classify_colour_bgr(bgr(clip), min_colour_fraction_, shrink_px_);
      std::string label;
      cv::Scalar col;
      if (colour == ConeColour::YELLOW) { label = "Yellow cone"; col = cv::Scalar(0,255,255); }
      else if (colour == ConeColour::BLUE){ label = "Blue cone";   col = cv::Scalar(255,128,0); }
      else { label = "Cone"; col = cv::Scalar(0,255,0); }

      // Detection2D
      Detection2D det;
      det.header = hdr;
      det.bbox.size_x = bb.width;
      det.bbox.size_y = bb.height;
      det.bbox.center.x = bb.x + bb.width  / 2.0;
      det.bbox.center.y = bb.y + bb.height / 2.0;
      det.bbox.center.theta = 0.0;

      ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = label;
      hyp.hypothesis.score = scores[idx];
      det.results.push_back(hyp);
      det_array.detections.push_back(det);

      // draw
      if (draw_boxes_) {
        cv::rectangle(annotated, bb, col, 2);
        if (draw_labels_) {
          char sc[32]; std::snprintf(sc, sizeof(sc), " %.2f", scores[idx]);
          std::string text = label + sc;
          int base = 0;
          cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &base);
          int x = std::max(0, bb.x);
          int y = std::max(0, bb.y - ts.height - 6);
          cv::rectangle(annotated, cv::Rect(x, y, ts.width + 6, ts.height + 6), col, cv::FILLED);
          cv::putText(annotated, text, cv::Point(x + 3, y + ts.height),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 2, cv::LINE_AA);
        }
      }
    }

    // publish and display
    img_pub_->publish(*cv_bridge::CvImage(hdr, "bgr8", annotated).toImageMsg());
    det_pub_->publish(det_array);

    cv::imshow(window_title_, annotated);
    if ((cv::waitKey(1) & 0xFF) == 'q') rclcpp::shutdown();
  }

  // params
  std::string input_topic_, annotated_topic_, detections_topic_, window_title_, model_path_;
  int inp_w_{640}, inp_h_{640};
  double conf_thr_{0.35}, nms_thr_{0.45};
  double min_colour_fraction_{0.10};
  int shrink_px_{4};
  bool use_cuda_{true}, qos_best_effort_{true}, draw_labels_{true}, draw_boxes_{true};

  // state
  torch::jit::script::Module module_;
  torch::Device device_{torch::kCPU};
  image_transport::Subscriber img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Publisher<Detection2DArray>::SharedPtr det_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloNode>());
  rclcpp::shutdown();
  return 0;
}

