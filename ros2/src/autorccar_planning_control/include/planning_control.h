#ifndef AUTOCAR_PLANNING_CONTROL_PLANNING_CONTROL_H_
#define AUTOCAR_PLANNING_CONTROL_PLANNING_CONTROL_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cubic_spline_path.h"

namespace autorccar {
namespace planning_control {
namespace planning_control {

enum class DriveCommand { kStop = 0, kStart };

using Point = Eigen::Vector2d;

struct State {
    double timestamp = 0.0;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond quat;
    Eigen::Vector3d accel;
    Eigen::Vector3d ang_vel;
};

struct ControlCommand {
    double speed = 0.0;
    double steering_angle = 0.0;
};

struct PurePursuitParameters {
    double min_look_ahead_distance = 0.3;
    double look_ahead_distance = 0.3;
};

struct ControlParameters {
    double goal_reach_threshold = 0.3;
    double target_speed = 0.0;
    double accel = 0.0;
    double decel = 0.0;
    double control_dt = 0.0;
    PurePursuitParameters pure_pursuit;
};

struct Parameters {
    double wheelbase = 0.0;
    double max_steering_angle = 0.0;
    ControlParameters control;
};

class PlanningControl {
   public:
    PlanningControl(const PlanningControl&) = delete;
    PlanningControl& operator=(const PlanningControl&) = delete;
    PlanningControl(PlanningControl&&) = delete;
    PlanningControl& operator=(PlanningControl&&) = delete;

    explicit PlanningControl(const Parameters& parameters);

    void SetGlobalPath(std::vector<Point>&& global_path, std::vector<double>&& speeds);
    void SetDriveCommand(const DriveCommand drive_command);
    void SetCurrentTargetSpeed(const double speed);
    ControlCommand GenerateMotionCommand(const State& current_state);
    Point GetLookAheadPoint();

   private:
    double CalcSpeedCommand(const State& state, double target_speed);
    std::pair<bool, double> CalculateSteeringCommand(const State& state);
    double CalcHeadingError(const State& state) const;
    double CalcSteeringAngle(double deviation_angle) const;
    bool FindLookAheadPoint(const State& state);
    bool GoalReached(const State& state) const;
    bool GotStartCommand() const;

    Parameters parameters_;
    double look_ahead_distance_squared_ = 0.0;
    double goal_reach_threshold_squared_ = 0.0;
    DriveCommand drive_command_ = DriveCommand::kStop;
    double current_target_speed_ = 0.0;
    Point goal_;
    Point look_ahead_point_{0.0, 0.0};
    std::vector<Point> global_path_;
    std::unique_ptr<CubicSplinePath> cubic_spline_path_;
    bool got_global_path_ = false;
};

}  // namespace planning_control
}  // namespace planning_control
}  // namespace autorccar

#endif  // AUTOCAR_PLANNING_CONTROL_PLANNING_CONTROL_H_