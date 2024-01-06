#include "publisher.hpp"

#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    node->initialize_config();

    std::string devPath = node->get_parameter("device_path").as_string();
    // Use designated port when given
    if (!devPath.empty()) {
        if (!std::filesystem::exists(devPath)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not found camera device: %s", devPath.c_str());
            return -1;
        }
    } else {
        if (!find_v4l_device_path(devPath)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "oCam-1MGN-U-T is not found in default path: /dev/v4l/by-id");
            return -1;
        }
    }

    Withrobot::Camera camera(devPath);
    camera.set_format(node->get_parameter("Width").as_int(), node->get_parameter("Height").as_int(),
                      Withrobot::fourcc_to_pixformat('G', 'B', 'G', 'R'), 1, node->get_parameter("FPS").as_int());

    camera.set_control("Exposure Time, Absolute", node->get_parameter("Exposure").as_int());
    camera.set_control("Gain", node->get_parameter("Brightness").as_int());
    camera.set_control("Auto Exposure", node->get_parameter("Auto exposure mode").as_int());

    // Print infomations
    Withrobot::camera_format camFormat;
    camera.get_current_format(camFormat);

    const std::string camName = camera.get_dev_name();
    const std::string camSerialNumber = camera.get_serial_number();

    printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    printf(
        "----------------- Current format informations "
        "-----------------\n");
    camFormat.print();
    printf(
        "------------------------------------------------------------"
        "---\n");

    std::cout << "Current Gain: " << camera.get_control("Gain") << std::endl;
    std::cout << "Current Exposure Time: " << camera.get_control("Exposure Time, Absolute") << std::endl;
    std::cout << "Current Auto Exposure Mode: " << camera.get_control("Auto Exposure") << std::endl;

    // Start streaming
    if (!camera.start()) {
        perror("Failed to start.");
        exit(0);
    }

    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    cv::Mat colorImg(cv::Size(camFormat.width, camFormat.height), CV_8UC3);

    std::signal(SIGINT, sigint_handler);
    while (rclcpp::ok()) {
        /* Copy a single frame(image) from camera(oCam-1MGN). This is a blocking
         * function. */
        int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);

        /* If the error occured, restart the camera. */
        if (size == -1) {
            printf("error number: %d\n", errno);
            perror("Cannot get image from camera");
            camera.stop();
            camera.start();
            continue;
        }

        cv::cvtColor(srcImg, colorImg, cv::COLOR_BayerGB2RGB);
        node->publish_image(colorImg);

        rclcpp::spin_some(node);
    }
    camera.stop();
    printf("Streaming stopped\n");

    rclcpp::shutdown();
    return 0;
}
