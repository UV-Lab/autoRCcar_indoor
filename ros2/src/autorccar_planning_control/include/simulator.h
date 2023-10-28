#ifndef AUTORCCAR_PLANNING_CONTROL_SIMULATOR_SIMULATOR_H_
#define AUTORCCAR_PLANNING_CONTROL_SIMULATOR_SIMULATOR_H_

#include <eigen3/Eigen/Dense>
#include <vector>

namespace autorccar {
namespace planning_control {
namespace simulator {

struct Parameters {
    double wheelbase = 0.0;
    double dt = 0.005;
};

struct SimulationState {
    double timestamp = 0.0;
    double x = 0.0;
    double y = 0.0;
    double speed = 0.0;
    double yaw = 0.0;
    double yaw_rate = 0.0;
};

struct ControlCommand {
    double speed = 0.0;
    double steering_angle = 0.0;
};

class Simulator {
   public:
    explicit Simulator(const Parameters& parameters);
    Simulator& operator=(const Simulator&) = delete;
    Simulator(Simulator&&) = delete;
    Simulator& operator=(Simulator&&) = delete;

    void SetInitialState(const SimulationState& initial_state);
    void UpdateModel(const ControlCommand& control_command);
    SimulationState GetCurrentState() const;
    void SetControlCommand(const ControlCommand& control_command);
    ControlCommand GetControlCommand() const;

   private:
    void UpdateBicycleModel(const ControlCommand& control_command);
    SimulationState state_;
    ControlCommand control_command_;
    Parameters parameters_;
};

}  // namespace simulator
}  // namespace planning_control
}  // namespace autorccar

#endif  // AUTORCCAR_PLANNING_CONTROL_SIMULATOR_SIMULATOR_H_
