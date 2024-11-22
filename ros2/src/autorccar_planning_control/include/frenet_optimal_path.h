#ifndef AUTORCCAR_PLANNING_CONTROL_FRENET_OPTIMAL_PATH_H_
#define AUTORCCAR_PLANNING_CONTROL_FRENET_OPTIMAL_PATH_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

#include "common.h"
#include "cubic_spline_path.h"

namespace autorccar {
namespace planning_control {
namespace frenet_optimal_path {

// aliases for convenience.
using Path = std::vector<Point>;
using Point = Eigen::Vector2d;
using common::BoundingBox;
using common::State;

struct FrenetPath {
    std::vector<double> d;
    std::vector<double> d_d;
    std::vector<double> dd_d;
    std::vector<double> ddd_d;
    std::vector<double> s;
    std::vector<double> d_s;
    std::vector<double> dd_s;
    std::vector<double> ddd_s;
    double cd = 0.0;
    double cv = 0.0;
    double cf = 0.0;

    Path path;
    std::vector<double> yaw;
    std::vector<double> ds;
    std::vector<double> c;
};

struct FrenetState {
    double speed = 0.0;
    double accel = 0.0;
    double lateral_distance = 0.0;
    double lateral_speed = 0.0;
    double lateral_accel = 0.0;
    double course_distance = 0.0;
};

struct Parameters {
    double max_speed = 0.0;
    double max_accel = 0.0;
    double max_curvature = 0.0;
    double max_road_width = 0.0;
    double d_road_width = 0.0;
    double dt = 0.0;
    double max_t = 0.0;
    double min_t = 0.0;
    double target_speed = 0.0;
    double d_target_speed = 0.0;
    int n_speed_sample = 0.0;
    double robot_radius = 0.0;
    double k_j = 0.0;
    double k_t = 0.0;
    double k_d = 0.0;
    double k_lat = 0.0;
    double k_lon = 0.0;
};

class QuarticPolynomial {
   public:
    QuarticPolynomial(const QuarticPolynomial&) = delete;
    QuarticPolynomial& operator=(const QuarticPolynomial&) = delete;
    QuarticPolynomial(QuarticPolynomial&&) = delete;
    QuarticPolynomial& operator=(QuarticPolynomial&&) = delete;

    explicit QuarticPolynomial(const double xs, const double vxs, const double axs, const double vxe, const double axe,
                               const double time);

    double CalculatePoint(double t) const;
    double CalculateFirstDerivative(double t) const;
    double CalculateSecondDerivative(double t) const;
    double CalculateThirdDerivative(double t) const;

   private:
    Eigen::MatrixXd CalculateAMatrix(const double time) const;
    Eigen::MatrixXd CalculateBMatrix(const double vxe, const double axe, const double time) const;

    double x0_;
    double x1_;
    double x2_;
    double x3_;
    double x4_;
};

class QuinticPolynomial {
   public:
    QuinticPolynomial(const QuinticPolynomial&) = delete;
    QuinticPolynomial& operator=(const QuinticPolynomial&) = delete;
    QuinticPolynomial(QuinticPolynomial&&) = delete;
    QuinticPolynomial& operator=(QuinticPolynomial&&) = delete;

    explicit QuinticPolynomial(const double xs, const double vxs, const double axs, const double xe, const double vxe,
                               const double axe, const double time);

    double CalculatePoint(double t) const;
    double CalculateFirstDerivative(double t) const;
    double CalculateSecondDerivative(double t) const;
    double CalculateThirdDerivative(double t) const;

   private:
    Eigen::MatrixXd CalculateAMatrix(const double time) const;
    Eigen::MatrixXd CalculateBMatrix(const double xe, const double vxe, const double axe, const double time) const;

    double x0_;
    double x1_;
    double x2_;
    double x3_;
    double x4_;
    double x5_;
};

class FrenetOptimalPath {
   public:
    FrenetOptimalPath(const FrenetOptimalPath&) = delete;
    FrenetOptimalPath& operator=(const FrenetOptimalPath&) = delete;
    FrenetOptimalPath(FrenetOptimalPath&&) = delete;
    FrenetOptimalPath& operator=(FrenetOptimalPath&&) = delete;

    explicit FrenetOptimalPath(const Parameters& parameters);

    void Planning(const std::unique_ptr<CubicSplinePath>& global_path, const State& current_state);
    void SetBoundingBoxes(std::vector<BoundingBox>&& bounding_boxes);
    bool IsPathGenerated();
    FrenetPath GetCurrentFrenetPath() const;

   private:
    FrenetState ComputeCurrentFrenetState(const std::unique_ptr<CubicSplinePath>& global_path,
                                          const State& current_state) const;
    void CalculateFrenetPaths(const FrenetState& current_state);
    void CalculateGlobalPaths(const std::unique_ptr<CubicSplinePath>& global_path);
    void CheckPaths();
    void CheckCollision();
    bool CheckPathCollision(const std::vector<Eigen::Vector2d>& path, const std::vector<BoundingBox>& bounding_boxes);

    Parameters parameters_;
    FrenetPath current_frenet_path_;
    std::vector<FrenetPath> frenet_paths_;
    std::vector<BoundingBox> bounding_boxes_;
};

}  // namespace frenet_optimal_path
}  // namespace planning_control
}  // namespace autorccar

#endif  // AUTORCCAR_PLANNING_CONTROL_FRENET_OPTIMAL_PATH_H_
