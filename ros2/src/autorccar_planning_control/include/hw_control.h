#ifndef __HW_CONTROL_H__
#define __HW_CONTROL_H__

#include <unistd.h>		//Used for UART
#include <sys/fcntl.h>		//Used for UART
#include <termios.h>		//Used for
#include <errno.h>

#include "rclcpp/rclcpp.hpp"
#include "autorccar_interfaces/msg/control_command.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#define ESC_PWM_MIN 3277
#define ESC_PWM_N 4915
#define ESC_PWM_MAX 6553
#define Steer_PWM_MIN 10 // 0 (margin 10)
#define Steer_PWM_N 90
#define Steer_PWM_MAX 170 // 180 (margin 10)


class hw_control
{
    public:
        int fd; // uart
        int cmdMode;
        //double steer, speed;
        
        geometry_msgs::msg::Vector3 msg;

        void uart_tx(const int fd, const geometry_msgs::msg::Vector3& msg);
        int uart_init(char* tty);
};

#endif
