#include "frenet_optimal_path.h"

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

namespace autorccar {
namespace planning_control {
namespace frenet_optimal_path {

QuarticPolynomial::QuarticPolynomial(const double xs, const double vxs, const double axs, const double vxe,
                                     const double axe, const double time)
    : x0_(xs), x1_(vxs), x2_(axs / 2.0) {
    Eigen::MatrixXd a_matrix = CalculateAMatrix(time);
    Eigen::MatrixXd b_matrix = CalculateBMatrix(vxe, axe, time);

    Eigen::MatrixXd c_matrix = a_matrix.inverse() * b_matrix;

    x3_ = c_matrix(0);
    x4_ = c_matrix(1);
}

Eigen::MatrixXd QuarticPolynomial::CalculateAMatrix(const double time) const {
    Eigen::MatrixXd a_matrix = Eigen::MatrixXd::Zero(2, 2);
    a_matrix(0, 0) = 3 * time * time;
    a_matrix(0, 1) = 4 * time * time * time;
    a_matrix(1, 0) = 6 * time;
    a_matrix(1, 1) = 12 * time * time;

    return a_matrix;
}

Eigen::MatrixXd QuarticPolynomial::CalculateBMatrix(const double vxe, const double axe, const double time) const {
    Eigen::MatrixXd b_matrix = Eigen::MatrixXd::Zero(2, 1);
    b_matrix(0) = vxe - x1_ - 2 * x2_ * time;
    b_matrix(1) = axe - 2 * x2_;

    return b_matrix;
}

double QuarticPolynomial::CalculatePoint(double t) const {
    return x0_ + (x1_ * t) + (x2_ * t * t) + (x3_ * t * t * t) + (x4_ * t * t * t * t);
}

double QuarticPolynomial::CalculateFirstDerivative(double t) const {
    return x1_ + (2 * x2_ * t) + (3 * x3_ * t * t) + (4 * x4_ * t * t * t);
}

double QuarticPolynomial::CalculateSecondDerivative(double t) const {
    return (2 * x2_) + (6 * x3_ * t) + (12 * x4_ * t * t);
}

double QuarticPolynomial::CalculateThirdDerivative(double t) const { return (6 * x3_) + (24 * x4_ * t); }

QuinticPolynomial::QuinticPolynomial(const double xs, const double vxs, const double axs, const double xe,
                                     const double vxe, const double axe, const double time)
    : x0_(xs), x1_(vxs), x2_(axs / 2.0) {
    Eigen::MatrixXd a_matrix = CalculateAMatrix(time);
    Eigen::MatrixXd b_matrix = CalculateBMatrix(xe, vxe, axe, time);

    Eigen::MatrixXd c_matrix = a_matrix.inverse() * b_matrix;

    x3_ = c_matrix(0);
    x4_ = c_matrix(1);
    x5_ = c_matrix(2);
}

Eigen::MatrixXd QuinticPolynomial::CalculateAMatrix(const double time) const {
    Eigen::MatrixXd a_matrix = Eigen::MatrixXd::Zero(3, 3);
    a_matrix(0, 0) = time * time * time;
    a_matrix(0, 1) = time * time * time * time;
    a_matrix(0, 2) = time * time * time * time * time;

    a_matrix(1, 0) = 3 * time * time;
    a_matrix(1, 1) = 4 * time * time * time;
    a_matrix(1, 2) = 5 * time * time * time * time;

    a_matrix(2, 0) = 6 * time;
    a_matrix(2, 1) = 12 * time * time;
    a_matrix(2, 2) = 20 * time * time * time;

    return a_matrix;
}

Eigen::MatrixXd QuinticPolynomial::CalculateBMatrix(const double xe, const double vxe, const double axe,
                                                    const double time) const {
    Eigen::MatrixXd b_matrix = Eigen::MatrixXd::Zero(3, 1);
    b_matrix(0) = xe - x0_ - x1_ * time - x2_ * time * time;
    b_matrix(1) = vxe - x1_ - 2 * x2_ * time;
    b_matrix(2) = axe - 2 * x2_;

    return b_matrix;
}

double QuinticPolynomial::CalculatePoint(double t) const {
    return x0_ + (x1_ * t) + (x2_ * t * t) + (x3_ * t * t * t) + (x4_ * t * t * t * t) + (x5_ * t * t * t * t * t);
}

double QuinticPolynomial::CalculateFirstDerivative(double t) const {
    return x1_ + (2 * x2_ * t) + (3 * x3_ * t * t) + (4 * x4_ * t * t * t) + (5 * x5_ * t * t * t * t);
}

double QuinticPolynomial::CalculateSecondDerivative(double t) const {
    return (2 * x2_) + (6 * x3_ * t) + (12 * x4_ * t * t) + (20 * x5_ * t * t * t);
}

double QuinticPolynomial::CalculateThirdDerivative(double t) const {
    return (6 * x3_) + (24 * x4_ * t) + (60 * x5_ * t * t);
}

FrenetOptimalPath::FrenetOptimalPath(const Parameters& parameters) : parameters_(parameters) {
    std::cout << "Frenet_optimal_path parametrs: " << std::endl;
    std::cout << "max_speed: " << parameters_.max_speed << std::endl;
    std::cout << "max_accel: " << parameters_.max_accel << std::endl;
    std::cout << "max_curvature: " << parameters_.max_curvature << std::endl;
    std::cout << "max_road_width: " << parameters_.max_road_width << std::endl;
    std::cout << "d_road_width: " << parameters_.d_road_width << std::endl;
    std::cout << "dt: " << parameters_.dt << std::endl;
    std::cout << "max_t: " << parameters_.max_t << std::endl;
    std::cout << "min_t: " << parameters_.min_t << std::endl;
    std::cout << "target_speed: " << parameters_.target_speed << std::endl;
    std::cout << "d_target_speed: " << parameters_.d_target_speed << std::endl;
    std::cout << "n_speed_sample: " << parameters_.n_speed_sample << std::endl;
    std::cout << "robot_radius: " << parameters_.robot_radius << std::endl;
    std::cout << "k_j: " << parameters_.k_j << std::endl;
    std::cout << "k_t: " << parameters_.k_t << std::endl;
    std::cout << "k_d: " << parameters_.k_d << std::endl;
    std::cout << "k_lat: " << parameters_.k_lat << std::endl;
    std::cout << "k_lon: " << parameters_.k_lon << std::endl;
}

void FrenetOptimalPath::Planning(const std::unique_ptr<CubicSplinePath>& global_path, const State& current_state) {
    FrenetState current_frenet_state = ComputeCurrentFrenetState(global_path, current_state);
    CalculateFrenetPaths(current_frenet_state);
    CalculateGlobalPaths(global_path);
    CheckPaths();

    double min_cost = frenet_paths_.at(0).cf;
    for (size_t i = 1; i < frenet_paths_.size(); i++) {
        if (min_cost >= frenet_paths_.at(i).cf) {
            min_cost = frenet_paths_.at(i).cf;
            current_frenet_path_ = frenet_paths_.at(i);
        }
    }
}

FrenetState FrenetOptimalPath::ComputeCurrentFrenetState(const std::unique_ptr<CubicSplinePath>& global_path,
                                                         const State& current_state) const {
    FrenetState current_frenet_state;
    current_frenet_state.speed = current_state.vel.x();
    current_frenet_state.accel = current_state.accel.x();
    current_frenet_state.lateral_speed = current_state.vel.y();
    current_frenet_state.lateral_accel = current_state.accel.y();

    Point current_pos{current_state.pos.x(), current_state.pos.y()};
    Reference path_ref = global_path->ReferencePoint(current_pos);

    current_frenet_state.course_distance = path_ref.distance;

    Eigen::Vector2d path_heading_vec{std::cos(path_ref.heading), std::sin(path_ref.heading)};
    Eigen::Vector2d distance_error_vec = current_pos - path_ref.point;
    double cross_prod_scalar =
        path_heading_vec.x() * distance_error_vec.y() - path_heading_vec.y() * distance_error_vec.x();
    current_frenet_state.lateral_distance =
        (cross_prod_scalar < 0.0) ? -distance_error_vec.norm() : distance_error_vec.norm();

    return current_frenet_state;
}

void FrenetOptimalPath::CalculateFrenetPaths(const FrenetState& current_state) {
    const double& max_road_width = parameters_.max_road_width;
    const double& d_road_width = parameters_.d_road_width;
    const double& max_t = parameters_.max_t;
    const double& min_t = parameters_.min_t;
    const double& dt = parameters_.dt;
    const double& k_j = parameters_.k_j;
    const double& k_t = parameters_.k_t;
    const double& k_d = parameters_.k_d;
    const double& k_lat = parameters_.k_lat;
    const double& k_lon = parameters_.k_lon;
    const double& target_speed = parameters_.target_speed;
    const double& d_t_s = parameters_.d_target_speed;
    const double& n_s_sample = parameters_.d_target_speed;
    const double& c_speed = current_state.speed;
    const double& c_s = current_state.course_distance;
    const double& c_accel = current_state.accel;
    const double& c_d = current_state.lateral_distance;
    const double& c_d_d = current_state.lateral_speed;
    const double& c_dd_d = current_state.lateral_accel;

    int n_road_sample = static_cast<int>(2 * max_road_width + 1);
    int n_time_sample = static_cast<int>((max_t - min_t) / dt + 1);
    int n_speed_sample = static_cast<int>(2 * n_s_sample + 1);

    frenet_paths_.clear();
    frenet_paths_.reserve(n_road_sample * n_time_sample * n_speed_sample);

    for (double di = -max_road_width; di <= max_road_width; di += d_road_width) {
        // Lateral motion planning
        for (double ti = min_t; ti <= max_t; ti += dt) {
            FrenetPath fp;
            int n_fp_sample = static_cast<int>(ti / dt + 1);
            fp.d.reserve(n_fp_sample);
            fp.d_d.reserve(n_fp_sample);
            fp.dd_d.reserve(n_fp_sample);
            fp.ddd_d.reserve(n_fp_sample);
            QuinticPolynomial lat_qp(c_d, c_d_d, c_dd_d, di, 0.0, 0.0, ti);
            for (double t = 0.0; t < ti; t += dt) {
                fp.d.emplace_back(lat_qp.CalculatePoint(t));
                fp.d_d.emplace_back(lat_qp.CalculateFirstDerivative(t));
                fp.dd_d.emplace_back(lat_qp.CalculateSecondDerivative(t));
                fp.ddd_d.emplace_back(lat_qp.CalculateThirdDerivative(t));
            }

            // Longitudinal motion planning
            for (double tv = target_speed - d_t_s * n_s_sample; tv <= target_speed + d_t_s * n_s_sample; tv += d_t_s) {
                FrenetPath tfp(fp);
                QuarticPolynomial lon_qp(c_s, c_speed, c_accel, tv, 0.0, ti);
                tfp.s.reserve(n_fp_sample);
                tfp.d_s.reserve(n_fp_sample);
                tfp.dd_s.reserve(n_fp_sample);
                tfp.ddd_s.reserve(n_fp_sample);
                for (double t = 0.0; t < ti; t += dt) {
                    tfp.s.emplace_back(lon_qp.CalculatePoint(t));
                    tfp.d_s.emplace_back(lon_qp.CalculateFirstDerivative(t));
                    tfp.dd_s.emplace_back(lon_qp.CalculateSecondDerivative(t));
                    tfp.ddd_s.emplace_back(lon_qp.CalculateThirdDerivative(t));
                }

                double j_p = std::accumulate(tfp.ddd_d.begin(), tfp.ddd_d.end(), 0.0,
                                             [](double x, double y) { return x + y * y; });
                double j_s = std::accumulate(tfp.ddd_s.begin(), tfp.ddd_s.end(), 0.0,
                                             [](double x, double y) { return x + y * y; });
                double ds = std::pow(target_speed - tfp.d_s.back(), 2);

                tfp.cd = k_j * j_p + k_t * ti + k_d * std::pow(tfp.d.back(), 2);
                tfp.cv = k_j * j_s + k_t * ti + k_d * ds;
                tfp.cf = k_lat * tfp.cd + k_lon * tfp.cv;

                frenet_paths_.push_back(tfp);
            }
        }
    }
}

void FrenetOptimalPath::CalculateGlobalPaths(const std::unique_ptr<CubicSplinePath>& global_path) {
    for (int i = 0; i < static_cast<int>(frenet_paths_.size()); i++) {
        // Calculate global positions
        FrenetPath& fp = frenet_paths_.at(i);
        fp.path.reserve(fp.s.size());
        for (int j = 0; j < static_cast<int>(fp.s.size()); j++) {
            if (global_path->GetRemainDistance(fp.s.at(j)) < 1.0e-9) break;
            Reference ref = global_path->ReferencePoint(fp.s.at(j));
            double dist = fp.d.at(j);
            Point pos{ref.point.x() + dist * std::cos(ref.heading + M_PI_2),
                      ref.point.y() + dist * std::sin(ref.heading + M_PI_2)};
            fp.path.push_back(pos);
        }

        // Calculate yaw and ds
        fp.yaw.reserve(fp.path.size());
        for (int j = 0; j < static_cast<int>(fp.path.size()) - 1; j++) {
            Eigen::Vector2d delta_vec = fp.path.at(j + 1) - fp.path.at(j);
            fp.yaw.push_back(std::atan2(delta_vec.y(), delta_vec.x()));
            fp.ds.push_back(std::hypot(delta_vec.x(), delta_vec.y()));
        }
        fp.yaw.push_back(fp.yaw.back());
        fp.ds.push_back(fp.ds.back());

        // Calculate curvature
        fp.c.reserve(fp.yaw.size() - 1);
        for (int j = 0; j < static_cast<int>(fp.yaw.size()) - 1; j++) {
            fp.c.push_back((fp.yaw.at(j + 1) - fp.yaw.at(j)) / fp.ds.at(j));
        }
    }
}

void FrenetOptimalPath::CheckPaths() {
    // Check max speed
    double max_speed = parameters_.max_speed;
    frenet_paths_.erase(std::remove_if(frenet_paths_.begin(), frenet_paths_.end(),
                                       [max_speed](FrenetPath& fp) {
                                           for (int i = 0; i < static_cast<int>(fp.d_s.size()); i++) {
                                               if (fp.d_s.at(i) > max_speed) return true;
                                           }
                                           return false;
                                       }),
                        frenet_paths_.end());

    // Check max accel
    double max_accel = parameters_.max_accel;
    frenet_paths_.erase(std::remove_if(frenet_paths_.begin(), frenet_paths_.end(),
                                       [max_accel](FrenetPath& fp) {
                                           for (int i = 0; i < static_cast<int>(fp.dd_s.size()); i++) {
                                               if (fp.dd_s.at(i) > max_accel) return true;
                                           }
                                           return false;
                                       }),
                        frenet_paths_.end());

    // Check max curvature
    double max_curvature = parameters_.max_curvature;
    frenet_paths_.erase(std::remove_if(frenet_paths_.begin(), frenet_paths_.end(),
                                       [max_curvature](FrenetPath& fp) {
                                           for (int i = 0; i < static_cast<int>(fp.c.size()); i++) {
                                               if (fp.c.at(i) > max_curvature) return true;
                                           }
                                           return false;
                                       }),
                        frenet_paths_.end());
    CheckCollision();
}

void FrenetOptimalPath::CheckCollision() {
    // TODO(luke7637): add collision check algorithm with cost map
}

bool FrenetOptimalPath::IsPathGenerated() { return current_frenet_path_.path.size() >= 2; }

FrenetPath FrenetOptimalPath::GetCurrentFrenetPath() const {
    if (current_frenet_path_.path.size() < 2) {
        return {};
    }
    return current_frenet_path_;
}

}  // namespace frenet_optimal_path
}  // namespace planning_control
}  // namespace autorccar
