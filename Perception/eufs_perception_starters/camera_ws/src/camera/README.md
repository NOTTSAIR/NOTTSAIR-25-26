# Code Explanation:

## 1. Libraries and includes:

- `rclcpp`: This is the main ROS 2 C++ client library. This gives us access to the most common elements of the ROS 2 System.
- `sensors_msgs`: This library contains message definitions for sensor data, including images, and we are using `image.hpp` to get access to the images produced by the camera.
- `image_transport`: This library makes working with images easier and more flexible, especially when compressing the image.
- `cv_bridge`: This library provides an interface between ROS and OpenCV, allowing us to convert ROS image messages to OpenCV images that will allow us to easily manipulate the images in the future.
- `opencv2/opencv.hpp`: This is the OpenCV library, which allows us to manipulate images and perform computer vision tasks.
- `rmw/qos_profiles.h`: This library provides access to Quality of Service (QoS) profiles, which are used to configure how messages are sent and received between publishers and subscribers in ROS 2.
- `string`, `vector`, `map`: These three libraries are just to help use different data structures to help represent our data in a good and efficient way.
- `mutex`: This library helps provide synchronization between threads, which is important when dealing with callbacks and multiple threads in ROS 2.
- `functional`: This library provides support for function objects, which can be used to create callbacks and other function-like behavior in C++.

## 2. Class Definition:

- `class CamFourView: public rclcpp::Node`: This defines and creates a new class called `CamFourView` which inherits everything from the class `rclcpp::Node`.
- `CamFourView() : Node("camera_viewer")`: This is the constructor for the `CamFourView` class. It initializes the node with the name "camera_viewer".`

### 2.1 Class constructor:

- `topics_ = declare_parameter<std::vector<std::string>>`: This line declares a parameter for a ROS 2 node. This is used to give a default parameter value while keeping the code clean.
	The use of `std::vector` is to just have the parameter be a list of strings.
- `rmw_qos_profile_t qos_reliable = rmw_qos_profile_default;`: This creates a QoS profile that copies the default settings for publishers/subscribers.
- `qos_reliable.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;`: This sets reliability to reliable, which means every message must be delivered (similar to TCP in networks)
- `qos_reliable.durability  = RMW_QOS_POLICY_DURABILITY_VOLATILE;`: This sets durability to volatile, which means that the messages aren't saved for any late-joining subscribers. In other words subscribers only get new messages after it connects.
- `rmw_qos_profile_t qos_besteffort = rmw_qos_profile_sensor_data;`: This creates a second QoS profile that uses "best effort" as it's reliability and "volatile" as its durability.
	Using "best-effort" means that messages may be lost if the system is busy (similar to UDP in networks)
- `for (const auto & topic : topics_) { ... }`: This loop goes through each topic in the `topics_` list.
	The loop creates two subscribers for each topic, and stores them in a vector called `subs_`. The vector also stores a tag saying which subscriber it recieved the image from.

- `timer_ = this->create_wall_timer(...)`: This creates a timer that calls the `timer_callback` function every 40 milliseconds. This calls the function `onTimer` 25 times a second, so the GUI is refreshed 25 times a second.

### 2.2 Private Members:

#### 2.2.1 `onImage` function:

##### 2.2.1.1 Function definition:

- `void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg, 
const std::string &topic, const std::string &qos_labels_)`: This function runs every time an image is recieved. 

##### 2.2.1.2 Parameters:

- `const sensor_msgs::msg::Image::ConstSharedPtr &msg`: This is a pointer to the actual ROS 2 image message that is recieved.
- `const std::string &topic`: This is the topic that the image was recieved from.
- `const std::string &qos_labels_`: This is a string that tells us which QoS profile was used to recieve the image.
	The QoS profiles are either "reliable" or "best-effort", as stated above in the constructor.

##### 2.2.1.3 Function body:

- `try {...} catch(cv_bridge::Exception & e) { ... })`: This is a try-catch block that attempts to convert the ROS image message to an OpenCV image. If the conversion fails, it catches the exception and prints out the error message.
- `cv::Mat img;`: This creates an empty OpenCV matrix to hold the image data.
- `const auto &enc = msg->encoding;`: This gets the encoding of the image from the ROS message. The encoding tells us how the image data is formatted (e.g., RGB, BGR, grayscale, etc.).
	The encoding is important because it tells us how the image data is formatted, which is necessary for proper conversion to an OpenCV image and necessary for manipulating the image properly.

###### 2.2.1.3.1 If statement:
1. `enc == sensor_msgs::image_encodings::BGR8`: This checks if the encoding of the image recieved is in BGR8 format.
	- `img = cv_bridge::toCvShare(msg, enc)->image.clone();`: If the condition above is met, then img will be converted to an OpenCV image through `cv_bridge::toCvShare`.
2.  `enc == sensor_msgs::image_encodings::RGB8`: This checks if the encoding of the image recieved is in RGB8 format.
	- `cv::Mat rgb = cv_bridge::toCvShare(msg, enc)->image;` This makes a matrix called `rgb` that holds the image data in RGB format after converting it to an OpenCV matrix.
	- `cv::cvtColor(rgb, img, cv::COLOR_RGB2BGR);`: This converts the matrix `rgb` from RGB format to a BGR format and stores the converted image in the `img` variable.
3. `enc == sensor_msgs::image_encodings::MONO8`: This checks if the encoding of the image recieved is in MONO8 format.
	- `cv::Mat mono = cv_bridge::toCvShare(msg, enc)->image;`: This makes a matrix called `mono` that holds the image data in MONO format after converting it to an OpenCV matrix.
	- `cv::cvtColor(mono, img, cv::COLOR_GRAY2BGR);`: This converts the matrix `mono` from grayscale format to a BGR format and stores the converted image in the `img` variable.
4. `enc == sensor_msgs::image_encodings::TYPE_32FC1 || enc == sensor_msgs::image_encodings::TYPE_16UC1`: This checks if the encoding of the image recieved is in either TYPE_32FC1 or TYPE_16UC1 format.
	- TYPE32FC1 is a 32-bit floating point single-channel image, and TYPE_16UC1 is a 16-bit unsigned integer single-channel image.
	- `cv::Mat depth = cv_bridge::toCvShare(msg, enc)->image;`: This makes a matrix called `depth` that holds the image data in either TYPE_32FC1 or TYPE_16UC1 format after converting it to an OpenCV matrix.
	- `double minv = 0.0, maxv = 0.0;`: This creates two double variables to hold the minimum and maximum values of the depth image.
	- `if (!(maxv > minv)) { maxv = minv + 1.0; }`: In case the maxv and minv are the same, this makes sure that maxv is always greater than minv by at least 1.0 (to not break the division step).
	- `cv::Mat norm8; depth.convertTo(norm8, CV_8U, 255.0 / (maxv - minv), -minv * 255.0 / (maxv - minv));`: This creates a matrix called `norm8`, and converts the `depth` image to an 8-bit unsigned integer format, scaling the values to fit within the range of 0 to 255.
	- `cv::cvtColor(norm8, img, cv::COLOR_GRAY2BGR);`: This converts the `norm8` matrix from grayscale format to a BGR format and stores the converted image in the `img` variable.
5. `else {img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();}`: If none of the above conditions are met, then the image is converted to BGR8 format by default.

##### 2.2.1.3 Function body continued:

- `{...}`: The empty curly braces aren't there for show, they create a new local scope for the code to use, so any variables created inside the braces are destroyed when the code exits the braces.
- `std::lock_guard<std::mutex> lock(mutex_);`: This creates a lock guard that locks the mutex `mutex_` for the duration of the scope. Locking the mutex ensures that only one thread can access the shared resource (in this case, the `images_` map) at a time.
	This is important so that no threads can interfere with the updating of `latest_` and `qos_labels_`.
- `latest_[topic] = img;`: This stores the latest image for the given topic in the `latest_` map.
- `qos_labels_[topic] = qos_labels_;`: This stores the QoS label for the given topic in the `qos_labels_` map.

#### 2.2.2 `onTimer` function:

##### 2.2.2.1 Function definition:

- `void onTimer()`: This function runs every time the timer created in the constructor goes off (every 40 milliseconds).

##### 2.2.2.2 Function body:

- `std::vector<cv::Mat> frames;`: This creates a vector of OpenCV matrices.
- `std::vector<std::string> labels;`: This creates a vector of strings to hold the labels for each frame.
- `{...}`: Again, the empty curly braces create a new local scope for the code to use.
- `std::lock_guard<std::mutex> lock(mutex_);`: This creates a lock guard that locks the mutex `mutex_` for the duration of the scope. Locking the mutex ensures that only one thread can access the shared resource (in this case, the `latest_` and `qos_labels_` maps) at a time.
- `for (const auto &t : topics_) { ... }`: This loops through each topic in `topics_`.
- `auto it = latest_.find(t);`: This finds the latest image for the topic `t` in the `latest_` map.
- `if (it != latest_.end() && !it->second.empty())`: This checks if `it` is not at the end of the map (meaning the topic exists in the map) and if the image is not empty.
	For clarification if `it` is not in the map, it returns `latest_.end()`.
- The following happens if the condition above is met:
	- `frames.push_back(it->second);`: This adds the latest image for the topic `t` to the `frames` vector. if the condition above is met.
	- `auto qit = last_qos_.find(t);`: This finds the QoS label for the topic `t` in the `last_qos_` map.
	- `labels.push_back(t + (qit != last_qos_.end() ? " [" + qit->second + "]" : ""));`: This adds the topic name and its QoS label (if it exists) to the `labels` vector.
- `if (frames.empty()) { ... }`: If the program hasn't recieved any images the following will happen:
	- `cv::Mat img(220, 760, CV_8UC3, cv::Scalar(20,20,20));`: This creates a blank image.
	- `cv::putText(img, "Waiting for image topics...", {20,120}, cv::FONT_HERSHEY_SIMPLEX, 0.9, {255,255,255}, 2, cv::LINE_AA);`: This adds text to the blank image saying "Waiting for image topics...".
	- `cv::imshow(window_title_, img); if ((cv::waitKey(1) & 0xFF) == 'q') rclcpp::shutdown();`: The first part displays that frame in a window, and the second part checks if the 'q' key is pressed. If it is, it shuts down the ROS 2 node.
- `const int H = 240, W = 320;`: Creates two constants, Height and Width, to be used for resizing the images and window size.
- `for (auto &f : frames) { ... }`: This loops through each frame in the `frames` vector. In this loop:
	- `if (!f.empty() && (f.cols != W || f.rows != H))`: This checks if the frame is not empty and if its dimensions are not equal to the desired dimensions (W x H). If the conditions are met then the following line runs:
		- `cv::resize(f, f, cv::Size(W, H));`: This resizes the frame to the desired dimensions (W x H).
- `cv::Mat mosaic(H*2, W*2, CV_8UC3, cv::Scalar(0,0,0));`: This creates a blank mosaic image that is 2x2 grid of the desired dimensions (H x W).
- `for (size_t i = 0; i < frames.size(); ++i) { ... }`: This loops through each frame in the `frames` vector by index. In this loop:
	- `int r = static_cast<int>(i / 2);, int c = static_cast<int>(i % 2);`: This calculates the row and column indices for placing the frame in the mosaic. `static_cast<int>` is used to convert the result of the division and modulus operations to integers.
	- `if (r < 2 && c < 2) { ... }`: This checks if the row and column indices are within the bounds of the mosaic (2x2 grid). If they are, then the following happens:
		- `frames[i].copyTo(mosaic(cv::Rect(c*W, r*H, W, H)));`: This copies the frame to the appropriate position in the mosaic.
		- `cv::putText(mosaic, labels[i], {c*W + 10, r*H + 20}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,0}, 2, cv::LINE_AA);`: This adds the label for the frame to the mosaic at the appropriate position.
- `cv::imshow(window_title_, mosaic); if ((cv::waitKey(1) & 0xFF) == 'q') rclcpp::shutdown();`: The first part displays the mosaic in a window, and the second part checks if the 'q' key is pressed. If it is, it shuts down the ROS 2 node.

## 3. Main function:

- `rclcpp::init(argc, argv);`: Initiates a ROS 2 node.
- `rclcpp::spin(std::make_shared<CamFourView>());`: This creates an instance of the `CamFourView` class and starts the ROS 2 event loop, allowing the node to process incoming messages and execute callbacks.
- `rclcpp::shutdown();`: This shuts down the ROS 2 node and cleans up resources when the node is no longer needed.