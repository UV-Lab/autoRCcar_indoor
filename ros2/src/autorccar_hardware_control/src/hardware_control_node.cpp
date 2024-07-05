#include <chrono>

#include "autorccar_interfaces/msg/control_command.hpp"
#include "hardware_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

namespace {

using autorccar::hardware_control::DriveCommand;

autorccar_interfaces::msg::ControlCommand ToMsg(const autorccar::hardware_control::ControlCommand& control_command) {
    autorccar_interfaces::msg::ControlCommand msg;
    msg.speed = control_command.speed;
    msg.steering_angle = control_command.steering_angle;
    return msg;
}

}  // namespace

class HardwareControlNode : public rclcpp::Node {
   public:
    explicit HardwareControlNode(const rclcpp::NodeOptions& options) : Node("hardware_control", options) {
        // read parameters
        ReadParameters();

        // initialize controller
        hardware_controller_ = std::make_unique<autorccar::hardware_control::HardwareControl>(parameters_);

        // publisher
        hardware_control_command_publisher_ = create_publisher<autorccar_interfaces::msg::ControlCommand>(
            "hardware_control/control_command", rclcpp::SystemDefaultsQoS());

        // subscriber
        rclcpp::SubscriptionOptions control_command_subscription_options;
        auto control_command_callback = [this](autorccar_interfaces::msg::ControlCommand::UniquePtr msg) {
            this->ControlCommandCallback(*msg);
        };
        control_command_subscription_options.event_callbacks.deadline_callback =
            [this](const rclcpp::QOSDeadlineRequestedInfo& /*event*/) {
                autorccar::hardware_control::ControlCommand control_command;
                control_command.speed = 0.0;
                control_command.steering_angle = 0.0;
                hardware_control_command_publisher_->publish(
                    ToMsg(hardware_controller_->SendControlCommand(control_command)));
                hardware_controller_->SetDriveCommand(DriveCommand::kStop);
                std::cout << "Control command deadline missed. RC car stopped. Send a Start command from GCS to resume."
                          << std::endl;
            };
        control_command_subscriber_ = create_subscription<autorccar_interfaces::msg::ControlCommand>(
            "planning_control/control_command",
            rclcpp::SystemDefaultsQoS().deadline(std::chrono::milliseconds(control_command_deadline_)),
            control_command_callback, control_command_subscription_options);

        auto gcs_command_callback = [this](std_msgs::msg::Int8::UniquePtr msg) { this->GcsCommandCallback(*msg); };
        gcs_command_subscriber_ =
            create_subscription<std_msgs::msg::Int8>("gcs/command", rclcpp::SystemDefaultsQoS(), gcs_command_callback);
    }

   private:
    void ReadParameters() {
        get_parameter_or<double>("rccar_config.max_speed", parameters_.max_speed, parameters_.max_speed);
        get_parameter_or<double>("rccar_config.max_steering_angle", parameters_.max_steering_angle,
                                 parameters_.max_steering_angle);
        get_parameter_or<std::string>("hardware_control.serial_port_name", parameters_.serial_port_name,
                                      parameters_.serial_port_name);
        get_parameter_or<int>("hardware_control.serial_baudrate", parameters_.serial_baudrate,
                              parameters_.serial_baudrate);
        get_parameter_or<int>("hardware_control.control_command_deadline", control_command_deadline_,
                              control_command_deadline_);
    }

    void ControlCommandCallback(const autorccar_interfaces::msg::ControlCommand& msg) const {
        autorccar::hardware_control::ControlCommand control_command;
        control_command.speed = msg.speed;
        control_command.steering_angle = msg.steering_angle;
        hardware_control_command_publisher_->publish(ToMsg(hardware_controller_->SendControlCommand(control_command)));
    }

    void GcsCommandCallback(const std_msgs::msg::Int8& msg) const {
        if (msg.data == 0) {
            hardware_controller_->SetDriveCommand(DriveCommand::kStop);
        } else if (msg.data == 1) {
            hardware_controller_->SetDriveCommand(DriveCommand::kStart);
        } else {
            hardware_controller_->SetDriveCommand(DriveCommand::kStop);
            std::cout << "Have undefined drive command." << std::endl;
        }
    }

    std::unique_ptr<autorccar::hardware_control::HardwareControl> hardware_controller_;
    autorccar::hardware_control::Parameters parameters_;

    // publisher
    rclcpp::Publisher<autorccar_interfaces::msg::ControlCommand>::SharedPtr hardware_control_command_publisher_;

    // subscriber
    rclcpp::Subscription<autorccar_interfaces::msg::ControlCommand>::SharedPtr control_command_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gcs_command_subscriber_;

    // variables
    int control_command_deadline_{1000};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<HardwareControlNode>(options));
    rclcpp::shutdown();

    return 0;
}
