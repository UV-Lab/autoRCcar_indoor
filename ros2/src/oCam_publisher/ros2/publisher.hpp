#include <algorithm>
#include <filesystem>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "api/withrobot_camera.hpp" /* withrobot camera API */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Custom signal handler is used since get_frame() of the api does not seem
// to work well with ros2's signal handler
void sigint_handler(int signal_value) { (void)signal_value; }

bool update_device_path(std::string &devPath) {
    std::vector<std::string> paths;

    // Withrobot camera id would be like
    // "usb-WITHROBOT_Inc._oCam-1CGN-U-T_SN_35E27013-video-index0"
    for (const auto &entry : std::filesystem::directory_iterator("/dev/v4l/by-id")) {
        if (entry.is_character_file() && (entry.path().filename().string().find("1CGN-U-T") != std::string::npos)) {
            auto path = entry.path().parent_path();
            path /= std::filesystem::read_symlink(entry.path());
            path = std::filesystem::canonical(path);
            paths.push_back(path);
        }
    }

    if (paths.empty()) return false;

    // Singel camera can contain two video pahts, normally ealier one gives
    // image
    std::sort(paths.begin(), paths.end());
    devPath = paths.front();
    return true;
}

class ImagePublisher : public rclcpp::Node {
   public:
    ImagePublisher() : Node("oCam_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 1);
    }

    void initialize_config() {
        declare_parameter("device_path", "");  // Blank for auto detection

        /*
         * oCam-1CGN supported image formats
         * USB 3.0
         * 	[0] "8-bit Greyscale 1280 x 720 60 fps"
         *	[1] "8-bit Greyscale 1280 x 960 45 fps"
         *	[2] "8-bit Greyscale 320 x 240 160 fps"
         * 	[3] "8-bit Greyscale 640 x 480 80 fps"
         *
         * USB 2.0
         * 	[0] "8-bit Greyscale 1280 x 720 30 fps"
         *	[1] "8-bit Greyscale 1280 x 960 22.5 fps"
         *	[2] "8-bit Greyscale 320 x 240 160 fps"
         * 	[3] "8-bit Greyscale 640 x 480 80 fps"
         */
        declare_parameter("Width", 640);
        declare_parameter("Height", 480);
        declare_parameter("FPS", 30);

        // See v4l2-ctl -d[#] --all to check control options
        // "Gain"(default[min, step, max]) : 64(64 [0, 1, 127])
        declare_parameter("Brightness", 110);
        // "Exposure (Absolute)", (default[min, step, max]) : 39(39 [1, 1, 625])
        declare_parameter("Exposure", 130);
        declare_parameter("Auto exposure mode",
                          1);  // 1 - Manual mode, 3 - Apeture priority mode
    }

    void publish_image(const cv::Mat &img) {
        sensor_msgs::msg::Image msg;
        msg.header.frame_id = count_++;  // Set the frame ID for the image
        msg.height = img.rows;
        msg.width = img.cols;
        msg.encoding = "rgb8";

        // Convert the OpenCV image to ROS2 sensor_msgs/Image format
        uint32_t size = img.total() * img.elemSize();
        msg.data.resize(size);
        memcpy(&msg.data[0], img.data, size);

        publisher_->publish(msg);
    }

    int32_t count_ = 0;

   private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};
