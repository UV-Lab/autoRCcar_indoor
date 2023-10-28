#include "simulator.h"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>

namespace autorccar {
namespace planning_control {
namespace simulator {

Simulator::Simulator(const Parameters& parameters) : parameters_(parameters) {
    std::cout << "simulator parameters:" << std::endl;
    std::cout << "wheelbase: " << parameters.wheelbase << std::endl;
    std::cout << "simulation delta time: " << parameters.dt << std::endl;
}

void Simulator::UpdateModel(const ControlCommand& control_command) { UpdateBicycleModel(control_command); }

void Simulator::UpdateBicycleModel(const ControlCommand& control_command) {
    const double& u_v = control_command.speed;
    const double& u_delta = control_command.steering_angle;
    const double& dt = parameters_.dt;
    const double& l = parameters_.wheelbase;

    state_.x += state_.speed * cos(state_.yaw) * dt;
    state_.y += state_.speed * sin(state_.yaw) * dt;
    state_.yaw += state_.yaw_rate * dt;
    state_.yaw_rate = (state_.speed / l) * tan(u_delta);
    state_.speed = state_.speed * 0.1 + u_v * (1 - 0.1);
    state_.timestamp += dt;
}

void Simulator::SetInitialState(const SimulationState& initial_state) {
    state_.timestamp = 0.0;
    state_.x = initial_state.x;
    state_.y = initial_state.y;
    state_.yaw = initial_state.yaw;
}

SimulationState Simulator::GetCurrentState() const { return state_; }

void Simulator::SetControlCommand(const ControlCommand& control_command) {
    control_command_.speed = control_command.speed;
    control_command_.steering_angle = control_command.steering_angle;
}

ControlCommand Simulator::GetControlCommand() const { return control_command_; }

}  // namespace simulator
}  // namespace planning_control
}  // namespace autorccar
