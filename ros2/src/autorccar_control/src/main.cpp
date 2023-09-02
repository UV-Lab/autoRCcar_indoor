#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>

#include "pure_pursuit.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

pure_pursuit pp;
int loop = 0;
int fid = -1;

class Control : public rclcpp::Node {

private:
    rclcpp::Subscription<autorccar_interfaces::msg::NavState>::SharedPtr sub_nav;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_gcs;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_key;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    std::shared_ptr<rclcpp::Node> node_;

public:
    Control() : Node("car_control") {
        sub_nav = this->create_subscription<autorccar_interfaces::msg::NavState>("nav_topic", 10, std::bind(&Control::callback_nav, this, _1));
        sub_gcs = this->create_subscription<std_msgs::msg::Int8>("command_topic", 10, std::bind(&Control::callback_gcs, this, _1));
        sub_key = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Control::callback_key, this, _1));

        pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("param_ctl", 10);

	node_ = std::make_shared<rclcpp::Node>("wait_for_message_node");
	std_msgs::msg::Float64MultiArray pathEq;
        auto ret = rclcpp::wait_for_message(pathEq, node_, "/path_topic");

        fid = pp.uart_init("/dev/ttyUSB0");

        if (ret) {
            pp.TargetCourse(pathEq);
            pp.mode = 0;  // start
        }


    }
    void callback_gcs(const std_msgs::msg::Int8::SharedPtr msg) {
        pp.command = msg->data;
    }

    void callback_key(const geometry_msgs::msg::Twist::SharedPtr msg) {
        pp.key_steer = msg->angular.z;
        pp.key_esc = msg->linear.x;
    }

    void callback_nav(const autorccar_interfaces::msg::NavState::SharedPtr msg) {
        pp.state_update(msg);
        pp.controlVehicleSpeed();  // PID Control
        if (!pp.checkGoal())
          pp.search_targetCourse_index();
        pp.calSteeringAngle();

        if (loop%2 == 0) { // 100Hz to 50Hz
          pp.uart_tx(fid, pub_);
	        //pp.PWM_publish(pub_);  // to MCU(ESP32)
        }

        pp.prev_time = pp.now_time;
        loop++;
    }


};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();

  close(fid);

  return 0;
}

