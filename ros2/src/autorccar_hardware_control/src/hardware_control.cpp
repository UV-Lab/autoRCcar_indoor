#include "hardware_control.h"

#include <cmath>
#include <fstream>
#include <iostream>

namespace autorccar {
namespace hardware_control {

HardwareControl::HardwareControl(const Parameters& parameters) : parameters_(parameters) {
    std::cout << "hardware_control parameters:" << std::endl;
    std::cout << "max_speed: " << parameters_.max_speed << std::endl;
    std::cout << "max_steering_angle: " << parameters_.max_steering_angle << std::endl;
    std::cout << "serial_port_name: " << parameters_.serial_port_name << std::endl;
    std::cout << "serial_baudrate: " << parameters_.serial_baudrate << std::endl;

    file_descriptor_ = SerialInitialize(parameters_.serial_port_name, parameters_.serial_baudrate);
}

void HardwareControl::SetDriveCommand(const DriveCommand& drive_command) { drive_command_ = drive_command; }

bool HardwareControl::GotStartCommand() const { return drive_command_ == DriveCommand::kStart; }

ControlCommand HardwareControl::SendControlCommand(ControlCommand& control_command) {
    if (!GotStartCommand()) {
        std::cout << "Have not received start command from GCS yet." << std::endl;
        SendStopMessage();
        return {};
    }

    control_command.speed = std::clamp(control_command.speed, 0.0, parameters_.max_speed);
    control_command.steering_angle =
        std::clamp(control_command.steering_angle, -parameters_.max_steering_angle, parameters_.max_steering_angle);

    Pwm control_pwm = ConvertCommandToPwm(control_command);

    control_pwm.speed = std::clamp(control_pwm.speed, kEscPwmMin, kEscPwmMax);
    control_pwm.steering = std::clamp(control_pwm.steering, kSteerPwmMin, kSteerPwmMax);

    SerializeAndSendMessage(control_pwm);

    return control_command;
}

int HardwareControl::SerialInitialize(const std::string serial_port_name, const int serial_baudrate) {
    int fd = open(serial_port_name.c_str(), O_RDWR | O_NOCTTY);
    struct termios toptions;

    /* get current serial port settings */
    tcgetattr(fd, &toptions);

    /* set baud both ways */
    if (auto it = baudrate_map.find(serial_baudrate); it != baudrate_map.end()) {
        cfsetispeed(&toptions, it->second);
    } else {
        std::cout << "The baudrate - " << serial_baudrate << " is not supported." << std::endl;
        std::abort();
    }

    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    /* Canonical mode */
    toptions.c_lflag |= ICANON;

    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);

    if (fd > 0) {
        std::cout << "[" << serial_port_name << "]" << " opened as " << fd << "." << std::endl;
    } else {
        std::cout << "Error " << errno << " from open: " << strerror(errno) << "." << std::endl;
        std::abort();
    }

    return fd;
}

Pwm HardwareControl::ConvertCommandToPwm(const ControlCommand& control_command) const {
    return {static_cast<int>(206.0306 * control_command.speed -
                             40.6541 * control_command.speed * control_command.speed + 5138.7189),
            static_cast<int>((-1.5) * control_command.steering_angle * (180.0 / M_PI)) + kSteerPwmN};
}

int HardwareControl::SerializeAndSendMessage(const Pwm& pwm) const {
    char msg_tx[8];

    msg_tx[0] = static_cast<char>(0xFF);  // header
    msg_tx[1] = static_cast<char>(0xFE);  // header
    msg_tx[2] = (pwm.steering >> 8) & 0xFF;
    msg_tx[3] = pwm.steering & 0xFF;
    msg_tx[4] = (pwm.speed >> 8) & 0xFF;
    msg_tx[5] = pwm.speed & 0xFF;
    msg_tx[6] = (static_cast<int>(drive_command_) >> 8) & 0xFF;
    msg_tx[7] = static_cast<int>(drive_command_) & 0xFF;
    return write(file_descriptor_, msg_tx, sizeof(msg_tx));
}

void HardwareControl::SendStopMessage() const {
    Pwm pwm{static_cast<int>(5138.7189), kSteerPwmN};
    SerializeAndSendMessage(pwm);
}

}  // namespace hardware_control
}  // namespace autorccar
