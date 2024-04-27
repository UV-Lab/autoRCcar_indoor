#ifndef AUTOCAR_PLANNING_CONTROL_PLANNING_CONTROL_H_
#define AUTOCAR_PLANNING_CONTROL_PLANNING_CONTROL_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common.h"
#include "cubic_spline_path.h"
#include "frenet_optimal_path.h"

namespace autorccar {
namespace planning_control {
namespace planning_control {

enum class DriveCommand { kStop = 0, kStart };

// Aliases for convenience.
using Point = Eigen::Vector2d;
using common::ControlCommand;
using common::State;
using frenet_optimal_path::FrenetOptimalPath;
using frenet_optimal_path::FrenetPath;

struct PurePursuitParameters {
    double min_look_ahead_distance = 0.3;
    double look_ahead_distance = 0.3;
};

struct ControlParameters {
    double goal_reach_threshold = 0.3;
    double accel = 0.0;
    double decel = 0.0;
    double control_dt = 0.0;
    PurePursuitParameters pure_pursuit;
};

struct Parameters {
    double wheelbase = 0.0;
    double max_steering_angle = 0.0;
    double target_speed = 0.0;
    ControlParameters control;
    frenet_optimal_path::Parameters frenet;
};

class PlanningControl {
   public:
    PlanningControl(const PlanningControl&) = delete;
    PlanningControl& operator=(const PlanningControl&) = delete;
    PlanningControl(PlanningControl&&) = delete;
    PlanningControl& operator=(PlanningControl&&) = delete;

    explicit PlanningControl(const Parameters& parameters);

    void SetGlobalPath(std::vector<Point>&& global_path, std::vector<double>&& speeds);
    void SetDriveCommand(const DriveCommand& drive_command);
    void SetCurrentTargetSpeed(const double speed);
    void SetCurrentState(const State& state);
    void PlanOnce();
    ControlCommand GenerateMotionCommand();
    Point GetLookAheadPoint();
    Path GetCurrentLocalPath();

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
    bool got_global_path_ = false;
    State current_state_;
    std::unique_ptr<CubicSplinePath> global_path_;
    std::unique_ptr<CubicSplinePath> local_path_;
    std::unique_ptr<FrenetOptimalPath> frenet_optimal_path_;
    FrenetPath current_frenet_path_{};
};

}  // namespace planning_control
}  // namespace planning_control
}  // namespace autorccar

#endif  // AUTOCAR_PLANNING_CONTROL_PLANNING_CONTROL_H_
