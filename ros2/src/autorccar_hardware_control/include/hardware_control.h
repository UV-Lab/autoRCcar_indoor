#ifndef AUTOCAR_HARDWARE_CONTROL_HARDWARE_CONTROL_H_
#define AUTOCAR_HARDWARE_CONTROL_HARDWARE_CONTROL_H_

#include <errno.h>
#include <sys/fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <map>

#include "autorccar_interfaces/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autorccar {
namespace hardware_control {

constexpr int kEscPwmMin{3277};
constexpr int kEscPwmN{4915};
constexpr int kEscPwmMax{6553};
constexpr int kSteerPwmMin{10};  // 0 (margin 10)
constexpr int kSteerPwmN{90};
constexpr int kSteerPwmMax{170};  // 180 (margin 10)

enum class DriveCommand { kStop = 0, kStart };

struct ControlCommand {
    double speed{0.0};
    double steering_angle{0.0};
};

struct Pwm {
    int speed{0};
    int steering{0};
};

const std::map<int, speed_t> baudrate_map = {{57600, B57600}, {115200, B115200}, {230400, B230400}, {460800, B460800}};

struct Parameters {
    double max_speed;
    double max_steering_angle;
    std::string serial_port_name;
    int serial_baudrate;
};

class HardwareControl {
   public:
    HardwareControl(const HardwareControl&) = delete;
    HardwareControl& operator=(const HardwareControl&) = delete;
    HardwareControl(HardwareControl&&) = delete;
    HardwareControl& operator=(HardwareControl&&) = delete;

    explicit HardwareControl(const Parameters& parameters);

    void SetDriveCommand(const DriveCommand& drive_command);
    ControlCommand SendControlCommand(ControlCommand& control_command);

   private:
    bool GotStartCommand() const;
    int SerialInitialize(const std::string serial_port_name, const int serial_baudrate);
    Pwm ConvertCommandToPwm(const ControlCommand& control_command) const;
    int SerializeAndSendMessage(const Pwm& pwm) const;
    void SendStopMessage() const;

    Parameters parameters_;
    int file_descriptor_;
    DriveCommand drive_command_{DriveCommand::kStop};
};

}  // namespace hardware_control
}  // namespace autorccar

#endif  // AUTOCAR_HARDWARE_CONTROL_HARDWARE_CONTROL_H_
