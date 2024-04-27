#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

#include "autorccar_interfaces/msg/control_command.hpp"
#include "autorccar_interfaces/msg/nav_state.hpp"
#include "autorccar_interfaces/msg/path.hpp"
#include "common.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "hw_control.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planning_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {

using Point = Eigen::Vector2d;
using autorccar::planning_control::planning_control::DriveCommand;

}  // namespace

hw_control hw;

class PlanningControlNode : public rclcpp::Node {
   public:
    explicit PlanningControlNode(const rclcpp::NodeOptions& options) : Node("planning_control", options) {
        // uart
        hw.fd = hw.uart_init("/dev/ttyUSB0");

        // read parameters
        ReadParameters();

        // initialize controller
        planning_controller_ =
            std::make_unique<autorccar::planning_control::planning_control::PlanningControl>(parameters_);

        // publisher
        control_command_publisher_ = create_publisher<autorccar_interfaces::msg::ControlCommand>(
            "planning_control/control_command", rclcpp::SystemDefaultsQoS());
        global_path_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "rviz/global_path_marker", rclcpp::SystemDefaultsQoS());
        current_pos_marker_publisher_ =
            create_publisher<visualization_msgs::msg::Marker>("rviz/current_pos_marker", rclcpp::SystemDefaultsQoS());
        look_ahead_point_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
            "rviz/look_ahead_point_marker", rclcpp::SystemDefaultsQoS());
        current_heading_odometry_publisher_ =
            create_publisher<nav_msgs::msg::Odometry>("rviz/current_heading", rclcpp::SystemDefaultsQoS());
        local_path_marker_publisher_ =
            create_publisher<nav_msgs::msg::Path>("rviz/local_path", rclcpp::SystemDefaultsQoS());

        // subscriber
        auto nav_state_callback = [this](autorccar_interfaces::msg::NavState::UniquePtr msg) {
            this->NavStateCallback(*msg);
        };
        nav_state_subscriber_ =
            create_subscription<autorccar_interfaces::msg::NavState>("nav_topic", 10, nav_state_callback);

        auto occupancy_grid_callback = [this](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
            std::cout << "got occupancy grid." << std::endl;
            this->OccupancyGridCallback(*msg);
        };
        occupancy_grid_subscriber_ =
            create_subscription<nav_msgs::msg::OccupancyGrid>("occupancy_grid/local", 10, occupancy_grid_callback);

        auto gcs_command_callback = [this](std_msgs::msg::Int8::UniquePtr msg) { this->GcsCommandCallback(*msg); };
        gcs_command_subscriber_ =
            create_subscription<std_msgs::msg::Int8>("gcs/command", rclcpp::SystemDefaultsQoS(), gcs_command_callback);

        auto global_path_callback = [this](autorccar_interfaces::msg::Path::UniquePtr msg) {
            this->GlobalPathCallback(*msg);
        };
        global_path_subscriber_ = create_subscription<autorccar_interfaces::msg::Path>(
            "gcs/global_path", rclcpp::SystemDefaultsQoS(), global_path_callback);
    }

   private:
    void ReadParameters() {
        get_parameter_or<double>("rccar_config.wheelbase", parameters_.wheelbase, parameters_.wheelbase);
        get_parameter_or<double>("rccar_config.max_steering_angle", parameters_.max_steering_angle,
                                 parameters_.max_steering_angle);
        get_parameter_or<double>("controller.goal_reach_threshold", parameters_.control.goal_reach_threshold,
                                 parameters_.control.goal_reach_threshold);
        get_parameter_or<double>("controller.target_speed", parameters_.target_speed, parameters_.target_speed);
        get_parameter_or<int>("controller.nav_hz", nav_hz_, nav_hz_);
        get_parameter_or<int>("controller.control_hz", control_hz_, control_hz_);
        nav_sampling_period_ = nav_hz_ / control_hz_;
        parameters_.control.control_dt = 1.0 / control_hz_;
        get_parameter_or<double>("controller.accel", parameters_.control.accel, parameters_.control.accel);
        get_parameter_or<double>("controller.decel", parameters_.control.decel, parameters_.control.decel);
        get_parameter_or<double>("controller.pure_pursuiter.look_ahead_distance",
                                 parameters_.control.pure_pursuit.look_ahead_distance,
                                 parameters_.control.pure_pursuit.look_ahead_distance);
        get_parameter_or<double>("frenet_optimal.max_speed", parameters_.frenet.max_speed,
                                 parameters_.frenet.max_speed);
        get_parameter_or<double>("frenet_optimal.max_accel", parameters_.frenet.max_accel,
                                 parameters_.frenet.max_accel);
        get_parameter_or<double>("frenet_optimal.max_curvature", parameters_.frenet.max_curvature,
                                 parameters_.frenet.max_curvature);
        get_parameter_or<double>("frenet_optimal.max_road_width", parameters_.frenet.max_road_width,
                                 parameters_.frenet.max_road_width);
        get_parameter_or<double>("frenet_optimal.d_road_width", parameters_.frenet.d_road_width,
                                 parameters_.frenet.d_road_width);
        get_parameter_or<double>("frenet_optimal.dt", parameters_.frenet.dt, parameters_.frenet.dt);
        get_parameter_or<double>("frenet_optimal.max_t", parameters_.frenet.max_t, parameters_.frenet.max_t);
        get_parameter_or<double>("frenet_optimal.min_t", parameters_.frenet.min_t, parameters_.frenet.min_t);
        get_parameter_or<double>("frenet_optimal.d_target_speed", parameters_.frenet.d_target_speed,
                                 parameters_.frenet.d_target_speed);
        get_parameter_or<int>("frenet_optimal.n_speed_sample", parameters_.frenet.n_speed_sample,
                              parameters_.frenet.n_speed_sample);
        get_parameter_or<double>("frenet_optimal.robot_radius", parameters_.frenet.robot_radius,
                                 parameters_.frenet.robot_radius);
        get_parameter_or<double>("frenet_optimal.k_j", parameters_.frenet.k_j, parameters_.frenet.k_j);
        get_parameter_or<double>("frenet_optimal.k_t", parameters_.frenet.k_t, parameters_.frenet.k_t);
        get_parameter_or<double>("frenet_optimal.k_d", parameters_.frenet.k_d, parameters_.frenet.k_d);
        get_parameter_or<double>("frenet_optimal.k_lat", parameters_.frenet.k_lat, parameters_.frenet.k_lat);
        get_parameter_or<double>("frenet_optimal.k_lon", parameters_.frenet.k_lon, parameters_.frenet.k_lon);
    }

    void NavStateCallback(const autorccar_interfaces::msg::NavState& msg) {
        if (++nav_sample_count_ > nav_sampling_period_) return;
        nav_sample_count_ = 0;

        VisualizeNavState(msg);
        GenerateControlCommand(msg);
        VisualizeLocalPath(planning_controller_->GetCurrentLocalPath());
    }

    void OccupancyGridCallback(const nav_msgs::msg::OccupancyGrid& /*msg*/) { planning_controller_->PlanOnce(); }

    void GenerateControlCommand(const autorccar_interfaces::msg::NavState& msg) const {
        autorccar::planning_control::common::State state;

        state.timestamp = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9;
        state.pos << msg.position.x, msg.position.y, msg.position.z;
        state.vel << msg.velocity.x, msg.velocity.y, msg.velocity.z;
        state.quat.w() = msg.quaternion.w;
        state.quat.x() = msg.quaternion.x;
        state.quat.y() = msg.quaternion.y;
        state.quat.z() = msg.quaternion.z;
        state.accel << msg.acceleration.x, msg.acceleration.y, msg.acceleration.z;
        state.ang_vel << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
        planning_controller_->SetCurrentState(state);

        autorccar::planning_control::planning_control::ControlCommand control_command;
        control_command = planning_controller_->GenerateMotionCommand();

        autorccar_interfaces::msg::ControlCommand control_command_msg;
        control_command_msg.speed = control_command.speed;
        control_command_msg.steering_angle = control_command.steering_angle;

        control_command_publisher_->publish(control_command_msg);

        hw.msg.x = control_command_msg.steering_angle;
        hw.msg.y = control_command_msg.speed;
        hw.msg.z = hw.cmdMode;
        hw.uart_tx(hw.fd, hw.msg);
    }

    void GcsCommandCallback(const std_msgs::msg::Int8& msg) const {
        hw.cmdMode = msg.data;
        if (msg.data == 0) {
            planning_controller_->SetDriveCommand(DriveCommand::kStop);
        } else if (msg.data == 1) {
            planning_controller_->SetCurrentTargetSpeed(0.0);
            planning_controller_->SetDriveCommand(DriveCommand::kStart);
        } else {
            planning_controller_->SetDriveCommand(DriveCommand::kStop);
            std::cout << "Have undefined drive command." << std::endl;
        }
    }

    void GlobalPathCallback(const autorccar_interfaces::msg::Path& msg) const {
        std::vector<Point> global_path;
        std::vector<double> speeds;
        for (const auto& path_point : msg.path_points) {
            Point point;
            double speed;
            point.x() = path_point.x;
            point.y() = path_point.y;
            speed = path_point.speed;
            global_path.push_back(point);
            speeds.push_back(speed);
        }
        VisualizeGlobalPath(global_path);
        planning_controller_->SetGlobalPath(std::move(global_path), std::move(speeds));
    }

    void VisualizeNavState(const autorccar_interfaces::msg::NavState& msg) const {
        // visualize current position
        visualization_msgs::msg::Marker current_pos_msg;
        current_pos_msg.header.frame_id = "map";
        current_pos_msg.header.stamp = this->now();
        current_pos_msg.action = visualization_msgs::msg::Marker::ADD;
        current_pos_msg.type = visualization_msgs::msg::Marker::SPHERE;
        current_pos_msg.ns = "current_pos";
        current_pos_msg.id = 0;
        current_pos_msg.pose.orientation.w = 1;
        current_pos_msg.scale.x = 0.5;
        current_pos_msg.scale.y = 0.5;
        current_pos_msg.scale.z = 0.5;
        current_pos_msg.color.r = 1.0;
        current_pos_msg.color.g = 1.0;
        current_pos_msg.color.b = 0.0;
        current_pos_msg.color.a = 1.0;

        geometry_msgs::msg::Point point;

        point.x = msg.position.x;
        point.y = msg.position.y;
        point.z = 0.0;

        current_pos_msg.pose.position.x = msg.position.x;
        current_pos_msg.pose.position.y = msg.position.y;
        current_pos_msg.pose.position.z = 0.0;

        current_pos_msg.points.push_back(point);
        current_pos_marker_publisher_->publish(current_pos_msg);

        // visualize look ahead point
        visualization_msgs::msg::Marker look_ahead_point_msg;
        look_ahead_point_msg.header.frame_id = "map";
        look_ahead_point_msg.header.stamp = this->now();
        look_ahead_point_msg.action = visualization_msgs::msg::Marker::ADD;
        look_ahead_point_msg.type = visualization_msgs::msg::Marker::SPHERE;
        look_ahead_point_msg.ns = "current_pos";
        look_ahead_point_msg.id = 0;
        look_ahead_point_msg.pose.orientation.w = 1;
        look_ahead_point_msg.scale.x = 0.5;
        look_ahead_point_msg.scale.y = 0.5;
        look_ahead_point_msg.scale.z = 0.5;
        look_ahead_point_msg.color.r = 0.0;
        look_ahead_point_msg.color.g = 1.0;
        look_ahead_point_msg.color.b = 1.0;
        look_ahead_point_msg.color.a = 1.0;

        Point look_ahead_point = planning_controller_->GetLookAheadPoint();
        point.x = look_ahead_point.x();
        point.y = look_ahead_point.y();
        point.z = 0.0;

        look_ahead_point_msg.pose.position.x = look_ahead_point.x();
        look_ahead_point_msg.pose.position.y = look_ahead_point.y();
        look_ahead_point_msg.pose.position.z = 0.0;

        look_ahead_point_msg.points.push_back(point);
        look_ahead_point_marker_publisher_->publish(look_ahead_point_msg);

        // visualize current heading
        nav_msgs::msg::Odometry current_heading_msg;
        current_heading_msg.header.frame_id = "map";
        current_heading_msg.header.stamp = this->now();

        current_heading_msg.pose.pose.position.x = msg.position.x;
        current_heading_msg.pose.pose.position.y = msg.position.y;
        current_heading_msg.pose.pose.position.z = 0.0;

        current_heading_msg.pose.pose.orientation.w = msg.quaternion.w;
        current_heading_msg.pose.pose.orientation.x = msg.quaternion.x;
        current_heading_msg.pose.pose.orientation.y = msg.quaternion.y;
        current_heading_msg.pose.pose.orientation.z = msg.quaternion.z;

        current_heading_odometry_publisher_->publish(current_heading_msg);
    }

    void VisualizeGlobalPath(const std::vector<Point>& global_path) const {
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker node;
        node.header.frame_id = "map";
        node.header.stamp = this->now();
        node.action = visualization_msgs::msg::Marker::ADD;
        node.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        node.ns = "global_path_node";
        node.id = 0;
        node.pose.orientation.w = 1;
        node.scale.x = 0.5;
        node.scale.y = 0.5;
        node.scale.z = 0.5;
        node.color.r = 1.0;
        node.color.g = 1.0;
        node.color.b = 1.0;
        node.color.a = 1.0;

        visualization_msgs::msg::Marker edge;
        edge.header.frame_id = "map";
        edge.header.stamp = this->now();
        edge.action = visualization_msgs::msg::Marker::ADD;
        edge.type = visualization_msgs::msg::Marker::LINE_LIST;
        edge.ns = "global_path_edge";
        edge.id = 1;
        edge.pose.orientation.w = 1;
        edge.scale.x = 0.1;
        edge.color.r = 1.0;
        edge.color.g = 1.0;
        edge.color.b = 1.0;
        edge.color.a = 1.0;

        for (auto it = global_path.begin(); it != global_path.end(); it++) {
            geometry_msgs::msg::Point point;
            if ((it + 1) != global_path.end()) {
                point.x = it->x();
                point.y = it->y();
                point.z = 0.0;
                node.points.push_back(point);
                edge.points.push_back(point);

                point.x = (it + 1)->x();
                point.y = (it + 1)->y();
                point.z = 0.0;
                edge.points.push_back(point);
            } else {
                point.x = it->x();
                point.y = it->y();
                point.z = 0.0;
                node.points.push_back(point);
            }
        }

        marker_array.markers.push_back(node);
        marker_array.markers.push_back(edge);

        global_path_marker_publisher_->publish(marker_array);
    }

    void VisualizeLocalPath(const std::vector<Point>& local_path) const {
        nav_msgs::msg::Path path_marker;
        path_marker.header.set__frame_id("map").set__stamp(now());
        for (const auto& point : local_path) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.set__frame_id("map").set__stamp(path_marker.header.stamp);
            pose_stamped.pose.position.set__x(point.x()).set__y(point.y()).set__z(0.0);
            pose_stamped.pose.orientation.set__x(0.0).set__y(0.0).set__z(0.0).set__w(1.0);
            path_marker.poses.push_back(pose_stamped);
        }
        local_path_marker_publisher_->publish(path_marker);
    }

    std::unique_ptr<autorccar::planning_control::planning_control::PlanningControl> planning_controller_;
    autorccar::planning_control::planning_control::Parameters parameters_;

    // publisher
    rclcpp::Publisher<autorccar_interfaces::msg::ControlCommand>::SharedPtr control_command_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_path_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr current_pos_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr look_ahead_point_marker_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr current_heading_odometry_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_marker_publisher_;

    // subscriber
    rclcpp::Subscription<autorccar_interfaces::msg::NavState>::SharedPtr nav_state_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gcs_command_subscriber_;
    rclcpp::Subscription<autorccar_interfaces::msg::Path>::SharedPtr global_path_subscriber_;

    // variables
    int nav_hz_ = 100;
    int control_hz_ = 50;
    int nav_sample_count_ = 0;
    int nav_sampling_period_ = 2;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<PlanningControlNode>(options));
    rclcpp::shutdown();

    return 0;
}
