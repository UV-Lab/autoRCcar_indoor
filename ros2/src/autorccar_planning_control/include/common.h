#ifndef AUTORCCAR_PLANNING_CONTROL_COMMON_H_
#define AUTORCCAR_PLANNING_CONTROL_COMMON_H_

#include <eigen3/Eigen/Dense>

namespace autorccar {
namespace planning_control {
namespace common {

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

}  // namespace common
}  // namespace planning_control
}  // namespace autorccar

#endif  // AUTORCCAR_PLANNING_CONTROL_SIMULATOR_SIMULATOR_H_
