#include "cubic_spline_path.h"

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autorccar {
namespace planning_control {

CubicSpline1D::CubicSpline1D(const std::vector<double>& x_points, const std::vector<double>& y_points)
    : num_points_(static_cast<int>(x_points.size())) {
    if (static_cast<int>(y_points.size()) != num_points_) {
        std::cout << "x_points size and y_points size must be same." << std::endl;
    }
    std::vector<double> diff_x;
    diff_x.reserve(num_points_ - 1);
    for (int i = 1; i < num_points_; i++) {
        diff_x.push_back(x_points.at(i) - x_points.at(i - 1));
    }

    x_.resize(num_points_);
    std::copy(x_points.begin(), x_points.end(), x_.begin());

    a_.resize(num_points_);
    std::copy(y_points.begin(), y_points.end(), a_.begin());

    Eigen::MatrixXd a_matrix = CalculateAMatrix(diff_x);
    Eigen::MatrixXd b_matrix = CalculateBMatrix(diff_x);

    Eigen::MatrixXd c_matrix = a_matrix.inverse() * b_matrix;

    c_.push_back(c_matrix(0));

    for (int i = 0; i < num_points_ - 1; i++) {
        c_.push_back(c_matrix(i + 1));
        b_.push_back(1.0 / diff_x.at(i) * (a_.at(i + 1) - a_.at(i)) -
                     diff_x.at(i) / 3.0 * (2.0 * c_.at(i) + c_.at(i + 1)));
        d_.push_back((c_.at(i + 1) - c_.at(i)) / (3.0 * diff_x.at(i)));
    }
}

Eigen::MatrixXd CubicSpline1D::CalculateAMatrix(const std::vector<double>& diff_x) const {
    Eigen::MatrixXd a_matrix = Eigen::MatrixXd::Zero(num_points_, num_points_);
    a_matrix(0, 0) = 1.0;
    for (int i = 1; i < num_points_ - 1; i++) {
        a_matrix(i, i - 1) = diff_x.at(i - 1);
        a_matrix(i, i + 1) = diff_x.at(i);
        a_matrix(i, i) = 2.0 * (diff_x.at(i - 1) + diff_x.at(i));
    }
    a_matrix(num_points_ - 1, num_points_ - 1) = 1.0;

    return a_matrix;
}

Eigen::MatrixXd CubicSpline1D::CalculateBMatrix(const std::vector<double>& diff_x) const {
    Eigen::MatrixXd b_matrix = Eigen::MatrixXd::Zero(num_points_, 1);
    for (int i = 0; i < num_points_ - 2; i++) {
        b_matrix(i + 1) =
            3.0 * (a_.at(i + 2) - a_.at(i + 1)) / diff_x.at(i + 1) - 3.0 * (a_.at(i + 1) - a_.at(i)) / diff_x.at(i);
    }
    return b_matrix;
}

double CubicSpline1D::CalculatePosition(double x, int idx) const {
    double dx = x - x_.at(idx);
    return a_.at(idx) + b_.at(idx) * dx + c_.at(idx) * dx * dx + d_.at(idx) * dx * dx * dx;
}

double CubicSpline1D::CalculateFirstDerivative(double x, int idx) const {
    double dx = x - x_.at(idx);
    return b_.at(idx) + 2.0 * c_.at(idx) * dx + 3.0 * d_.at(idx) * dx * dx;
}

double CubicSpline1D::CalculateSecondDerivative(double x, int idx) const {
    double dx = x - x_.at(idx);
    return 2.0 * c_.at(idx) + 6.0 * d_.at(idx) * dx;
}

CubicSpline2D::CubicSpline2D(const Path& path) {
    CalculateDistanceParameter(path);

    std::vector<double> x_points;
    std::vector<double> y_points;

    x_points.reserve(path.size());
    y_points.reserve(path.size());

    for (const auto& p : path) {
        x_points.push_back(p.x());
        y_points.push_back(p.y());
    }

    cubic_spline_x_ = std::make_unique<CubicSpline1D>(distance_, x_points);
    cubic_spline_y_ = std::make_unique<CubicSpline1D>(distance_, y_points);
}

void CubicSpline2D::CalculateDistanceParameter(const Path& path) {
    distance_.push_back(0.0);
    for (size_t i = 1; i < path.size(); i++) {
        double dx = path.at(i).x() - path.at(i - 1).x();
        double dy = path.at(i).y() - path.at(i - 1).y();
        delta_distance_.push_back(std::sqrt(dx * dx + dy * dy));
        distance_.push_back(distance_.at(i - 1) + delta_distance_.at(i - 1));
    }
}

double CubicSpline2D::GetDistance(int idx) const { return distance_.at(idx); }

double CubicSpline2D::GetDeltaDistance(int idx) const { return delta_distance_.at(idx); }

Point CubicSpline2D::CalculatePosition(double s, int idx) const {
    return {cubic_spline_x_->CalculatePosition(s, idx), cubic_spline_y_->CalculatePosition(s, idx)};
}

double CubicSpline2D::CalculateHeading(double s, int idx) const {
    return std::atan2(cubic_spline_y_->CalculateFirstDerivative(s, idx),
                      cubic_spline_x_->CalculateFirstDerivative(s, idx));
}

double CubicSpline2D::CalculateCurvature(double s, int idx) const {
    double dx = cubic_spline_x_->CalculateFirstDerivative(s, idx);
    double ddx = cubic_spline_x_->CalculateSecondDerivative(s, idx);
    double dy = cubic_spline_y_->CalculateFirstDerivative(s, idx);
    double ddy = cubic_spline_y_->CalculateSecondDerivative(s, idx);

    return (ddy * dx - ddx * dy) / std::pow(dx * dx + dy * dy, 1.5);
}

CubicSplinePath::CubicSplinePath(Path&& input_path) : path_(std::move(input_path)), cubic_spline_path_(path_) {
    if (path_.size() < 2) {
        std::cout << "Path must have at least two points." << std::endl;
        path_generated_ = false;
    }
    total_distance_ = cubic_spline_path_.GetDistance(static_cast<int>(path_.size()) - 1);
    path_generated_ = true;
}

bool CubicSplinePath::IsPathGenerated() { return path_generated_; }

void CubicSplinePath::InvalidateCurrentPathIndex() { current_path_idx_valid_ = false; }

Reference CubicSplinePath::ReferencePoint(const Point& position) {
    if (!current_path_idx_valid_) {
        int closest_point_idx = 0;
        double min_squared_distance = std::numeric_limits<double>::max();
        for (size_t idx = 0; idx < path_.size(); idx++) {
            auto point = path_.at(idx);
            double squared_distance = (position - point).squaredNorm();
            if (squared_distance < min_squared_distance) {
                min_squared_distance = squared_distance;
                closest_point_idx = static_cast<int>(idx);
            }
        }
        current_path_idx_ = closest_point_idx;
        current_path_idx_valid_ = true;
    }

    auto [path_idx, delta_distance] = FindPathIndexAndDeltaDistance(position, current_path_idx_);

    Point closest_point;
    double heading;
    double curvature;
    double distance;
    double remain_distance;

    if (path_idx == static_cast<int>(path_.size()) - 1) {
        heading = cubic_spline_path_.CalculateHeading(cubic_spline_path_.GetDistance(path_idx), path_idx - 1);
        Point delta_position(delta_distance * cos(heading), delta_distance * sin(heading));
        closest_point = path_.at(path_idx) + delta_position;
        curvature = 0.0;
        distance = cubic_spline_path_.GetDistance(path_idx) + delta_distance;
        remain_distance = total_distance_ - distance;
        path_idx = path_idx - 1;
    } else if (path_idx == -1) {
        heading = cubic_spline_path_.CalculateHeading(0, 0);
        Point delta_position(delta_distance * cos(heading), delta_distance * sin(heading));
        closest_point = path_.at(0) + delta_position;
        curvature = 0.0;
        distance = delta_distance;
        remain_distance = total_distance_ - distance;
        path_idx = 0;
    } else {
        distance = cubic_spline_path_.GetDistance(path_idx) + delta_distance;
        closest_point = cubic_spline_path_.CalculatePosition(distance, path_idx);
        heading = cubic_spline_path_.CalculateHeading(distance, path_idx);
        curvature = cubic_spline_path_.CalculateCurvature(distance, path_idx);
        remain_distance = total_distance_ - distance;
    }

    current_path_idx_ = path_idx;

    return {closest_point, heading, curvature, distance, remain_distance};
}

Reference CubicSplinePath::ReferencePoint(const double distance) const {
    int path_idx = 0;
    if (distance >= total_distance_) {
        path_idx = static_cast<int>(path_.size()) - 1;
    } else if (distance <= 0) {
        path_idx = -1;
    } else {
        for (int i = 0; i < static_cast<int>(path_.size()) - 1; i++) {
            if (cubic_spline_path_.GetDistance(i) <= distance && distance < cubic_spline_path_.GetDistance(i + 1)) {
                path_idx = i;
            }
        }
    }

    Point closest_point;
    double heading;
    double curvature;
    double remain_distance;

    if (path_idx == static_cast<int>(path_.size()) - 1) {
        heading = cubic_spline_path_.CalculateHeading(cubic_spline_path_.GetDistance(path_idx), path_idx - 1);
        double delta_distance = distance - total_distance_;
        Point delta_position(delta_distance * cos(heading), delta_distance * sin(heading));
        closest_point = path_.at(path_idx) + delta_position;
        curvature = 0.0;
        remain_distance = total_distance_ - distance;
    } else if (path_idx == -1) {
        heading = cubic_spline_path_.CalculateHeading(0, 0);
        Point delta_position(distance * cos(heading), distance * sin(heading));
        closest_point = path_.at(0) + delta_position;
        curvature = 0.0;
        remain_distance = total_distance_ - distance;
    } else {
        closest_point = cubic_spline_path_.CalculatePosition(distance, path_idx);
        heading = cubic_spline_path_.CalculateHeading(distance, path_idx);
        curvature = cubic_spline_path_.CalculateCurvature(distance, path_idx);
        remain_distance = total_distance_ - distance;
    }

    return {closest_point, heading, curvature, distance, remain_distance};
}

double CubicSplinePath::GetRemainDistance(const Point& position) const {
    auto [path_idx, delta_distance] = FindPathIndexAndDeltaDistance(position, current_path_idx_);

    if (path_idx == static_cast<int>(path_.size()) - 1) {
        path_idx = path_idx - 1;
    } else if (path_idx == -1) {
        path_idx = 0;
    }
    double current_distance = cubic_spline_path_.GetDistance(path_idx) + delta_distance;

    return total_distance_ - current_distance;
}

double CubicSplinePath::GetRemainDistance(const double distance) const { return total_distance_ - distance; }

std::pair<int, double> CubicSplinePath::FindPathIndexAndDeltaDistance(const Point& position, int current_idx) const {
    if (current_idx == static_cast<int>(path_.size()) - 1) {
        double heading =
            cubic_spline_path_.CalculateHeading(cubic_spline_path_.GetDistance(current_idx), current_idx - 1);
        Point p0 = position - path_.at(current_idx);
        return {current_idx, p0.x() * cos(heading) + p0.y() * sin(heading)};
    } else if (current_idx == -1) {
        double heading = cubic_spline_path_.CalculateHeading(0, 0);
        Point p0 = position - path_.at(0);
        return {current_idx, p0.x() * cos(heading) + p0.y() * sin(heading)};
    }
    Point p0 = position - path_.at(current_idx);
    Point p1 = (path_.at(current_idx + 1) - path_.at(current_idx)).normalized();
    double dot_product = p0.dot(p1);
    if (dot_product > cubic_spline_path_.GetDeltaDistance(current_idx)) {
        return FindPathIndexAndDeltaDistance(position, current_idx + 1);
    } else if (dot_product < 0) {
        if (current_idx > 0) {
            p0 = position - path_.at(current_idx - 1);
            p1 = (path_.at(current_idx) - path_.at(current_idx - 1)).normalized();
            dot_product = p0.dot(p1);
            if (dot_product > cubic_spline_path_.GetDeltaDistance(current_idx - 1)) {
                return {current_idx, 0.0};
            }
        }
        return FindPathIndexAndDeltaDistance(position, current_idx - 1);
    }

    return {current_idx, dot_product};
}

}  // namespace planning_control
}  // namespace autorccar
