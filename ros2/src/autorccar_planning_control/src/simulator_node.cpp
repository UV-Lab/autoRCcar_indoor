#include <chrono>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

#include "autorccar_interfaces/msg/control_command.hpp"
#include "autorccar_interfaces/msg/nav_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simulator.h"

class SimulatorNode : public rclcpp::Node {
   public:
    explicit SimulatorNode(const rclcpp::NodeOptions& options) : Node("planning_control", options) {
        // read parameters
        get_parameter_or<double>("rccar_config.wheelbase", parameters_.wheelbase, parameters_.wheelbase);
        get_parameter_or<int>("simulator.simulation_hz", sim_hz_, sim_hz_);
        get_parameter_or<int>("simulator.nav_pub_hz", nav_pub_hz_, nav_pub_hz_);
        parameters_.dt = 1.0 / sim_hz_;

        // initialize
        simulator_ = std::make_unique<autorccar::planning_control::simulator::Simulator>(parameters_);

        // timer callbacks
        auto simulation_timer_callback = [this] { simulator_->UpdateModel(simulator_->GetControlCommand()); };
        simulation_timer_ = create_wall_timer(std::chrono::microseconds(static_cast<int64_t>(1.0 / sim_hz_ * 1e6)),
                                              simulation_timer_callback);

        auto nav_pub_timer_callback = [this] { PublishNavState(); };
        nav_pub_timer_ = create_wall_timer(std::chrono::microseconds(static_cast<int64_t>(1.0 / nav_pub_hz_ * 1e6)),
                                           nav_pub_timer_callback);

        // publisher
        nav_state_publisher_ =
            create_publisher<autorccar_interfaces::msg::NavState>("nav_topic", rclcpp::SystemDefaultsQoS());

        // subscriber

        auto control_command_callback = [this](autorccar_interfaces::msg::ControlCommand::UniquePtr msg) {
            this->ControlCommandCallback(*msg);
        };
        control_command_subscriber_ = create_subscription<autorccar_interfaces::msg::ControlCommand>(
            "planning_control/control_command",
            rclcpp::SystemDefaultsQoS().deadline(std::chrono::milliseconds(control_command_deadline_)),
            control_command_callback);

        auto initialpose_callback = [this](geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) {
            this->InitialposeCallback(*msg);
        };
        initialpose_subscriber_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", rclcpp::SystemDefaultsQoS(), initialpose_callback);
    }

   private:
    void PublishNavState() {
        if (is_initialized_) {
            autorccar::planning_control::simulator::SimulationState state = simulator_->GetCurrentState();
            autorccar_interfaces::msg::NavState nav_state_msg;

            nav_state_msg.timestamp.sec = static_cast<int>(state.timestamp);
            nav_state_msg.timestamp.nanosec =
                static_cast<int>((state.timestamp - static_cast<int>(state.timestamp)) * 1.0e9);
            nav_state_msg.origin.x = 10000.0;
            nav_state_msg.origin.y = 10000.0;
            nav_state_msg.origin.z = 10000.0;
            nav_state_msg.position.x = state.x;
            nav_state_msg.position.y = state.y;
            nav_state_msg.position.z = 0.0;
            nav_state_msg.velocity.x = state.speed;
            nav_state_msg.velocity.y = 0.0;
            nav_state_msg.velocity.z = 0.0;
            Eigen::Quaterniond quaternion(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()));
            nav_state_msg.quaternion.w = quaternion.w();
            nav_state_msg.quaternion.x = quaternion.x();
            nav_state_msg.quaternion.y = quaternion.y();
            nav_state_msg.quaternion.z = quaternion.z();
            nav_state_msg.angular_velocity.x = 0.0;
            nav_state_msg.angular_velocity.y = 0.0;
            nav_state_msg.angular_velocity.z = state.yaw_rate;

            nav_state_publisher_->publish(nav_state_msg);
        }
    }
    void ControlCommandCallback(const autorccar_interfaces::msg::ControlCommand& msg) const {
        autorccar::planning_control::simulator::ControlCommand control_command;
        control_command.speed = msg.speed;
        control_command.steering_angle = msg.steering_angle;
        simulator_->SetControlCommand(control_command);
    }

    void InitialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
        Eigen::Quaterniond quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
        Eigen::Vector3d vec_forward = quaternion * Eigen::Vector3d::UnitX();

        autorccar::planning_control::simulator::SimulationState initial_state;
        initial_state.x = msg.pose.pose.position.x;
        initial_state.y = msg.pose.pose.position.y;
        initial_state.yaw = atan2(vec_forward.y(), vec_forward.x());
        simulator_->SetInitialState(initial_state);

        is_initialized_ = true;
    }

    std::unique_ptr<autorccar::planning_control::simulator::Simulator> simulator_;
    autorccar::planning_control::simulator::Parameters parameters_;

    // timer
    rclcpp::TimerBase::SharedPtr simulation_timer_;
    rclcpp::TimerBase::SharedPtr nav_pub_timer_;

    // publisher
    rclcpp::Publisher<autorccar_interfaces::msg::NavState>::SharedPtr nav_state_publisher_;

    // subscriber
    rclcpp::Subscription<autorccar_interfaces::msg::ControlCommand>::SharedPtr control_command_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscriber_;

    // variables
    bool is_initialized_{false};
    int sim_hz_{500};
    int nav_pub_hz_{100};
    int control_command_deadline_{1000};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<SimulatorNode>(options));
    rclcpp::shutdown();

    return 0;
}
