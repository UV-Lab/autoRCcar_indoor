#ifndef AUTORCCAR_PLANNING_CONTROL_CUBIC_SPLINE_PATH_H_
#define AUTORCCAR_PLANNING_CONTROL_CUBIC_SPLINE_PATH_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

namespace autorccar {
namespace planning_control {

using Point = Eigen::Vector2d;
using Path = std::vector<Point>;

struct Reference {
    Point point;
    double heading;
    double curvature;
};

class CubicSpline1D {
   public:
    CubicSpline1D(const CubicSpline1D&) = delete;
    CubicSpline1D& operator=(const CubicSpline1D&) = delete;
    CubicSpline1D(CubicSpline1D&&) = delete;
    CubicSpline1D& operator=(CubicSpline1D&&) = delete;

    explicit CubicSpline1D(const std::vector<double>& x_points, const std::vector<double>& y_points);

    double CalculatePosition(double x, int idx) const;
    double CalculateFirstDerivative(double x, int idx) const;
    double CalculateSecondDerivative(double x, int idx) const;

   private:
    Eigen::MatrixXd CalculateAMatrix(const std::vector<double>& diff_x) const;
    Eigen::MatrixXd CalculateBMatrix(const std::vector<double>& diff_x) const;

    std::vector<double> x_;
    std::vector<double> a_;
    std::vector<double> b_;
    std::vector<double> c_;
    std::vector<double> d_;
    int num_points_;
};

class CubicSpline2D {
   public:
    CubicSpline2D(const CubicSpline2D&) = delete;
    CubicSpline2D& operator=(const CubicSpline2D&) = delete;
    CubicSpline2D(CubicSpline2D&&) = delete;
    CubicSpline2D& operator=(CubicSpline2D&&) = delete;

    explicit CubicSpline2D(const Path& path);

    Point CalculatePosition(double s, int idx) const;
    double CalculateHeading(double s, int idx) const;
    double CalculateCurvature(double s, int idx) const;
    double GetDistance(int idx) const;
    double GetDeltaDistance(int idx) const;

   private:
    void CalculateDistanceParameter(const Path& path);

    std::vector<double> delta_distance_;
    std::vector<double> distance_;
    std::unique_ptr<CubicSpline1D> cubic_spline_x_;
    std::unique_ptr<CubicSpline1D> cubic_spline_y_;
};

class CubicSplinePath {
   public:
    CubicSplinePath(const CubicSplinePath&) = delete;
    CubicSplinePath& operator=(const CubicSplinePath&) = delete;
    CubicSplinePath(CubicSplinePath&&) = delete;
    CubicSplinePath& operator=(CubicSplinePath&&) = delete;

    explicit CubicSplinePath(Path&& input_path);

    bool IsPathGenerated();
    void InvalidateCurrentPathIndex();
    Reference ReferencePoint(const Point& position);
    double GetRemainDistance(const Point& position);

   private:
    std::pair<int, double> FindPathIndexAndDeltaDistance(const Point& position, int current_idx);
    Path path_;
    CubicSpline2D cubic_spline_path_;
    bool current_path_idx_valid_ = false;
    int current_path_idx_ = 0;
    bool path_generated_ = false;
};

}  // namespace planning_control
}  // namespace autorccar

#endif  // AUTORCCAR_PLANNING_CONTROL_CUBIC_SPLINE_PATH_H_