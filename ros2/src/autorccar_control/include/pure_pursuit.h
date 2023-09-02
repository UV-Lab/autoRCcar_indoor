#ifndef __PURE_PURSUIT_H__
#define __PURE_PURSUIT_H__

#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "autorccar_interfaces/msg/nav_state.hpp"

#include <unistd.h>		//Used for UART
#include <sys/fcntl.h>		//Used for UART
#include <termios.h>		//Used for
#include <errno.h>

#define ESC_PWM_MIN 3277
#define ESC_PWM_N 4915
#define ESC_PWM_MAX 6553
#define Steer_PWM_MIN 10 // 0 (margin 10)
#define Steer_PWM_N 90
#define Steer_PWM_MAX 170 // 180 (margin 10)

using namespace Eigen;

class pure_pursuit
{
    private:
        double Kp = 0.3;         // speed proportional gain
        double Klf = 1;          // look forward gain
        double lxd = 1;        // [m] look-ahead distance
        double Lf = 1;           // [m] Updated look-ahead distance
        double dt = 0.1;         // [s] time tick
        double WB = 0.3;         // [m] wheel base of vehicle

        double limitSpeed = 5.5/3.6;   // [m/s] = [km/h]/3.6
        double GoalDist = 2;

        int old_tgIdx = 0;


    public:
        int fd; // uart
        double ai, delta;        // control input (Hardware)
        double x_vel, x_yaw;
        double prev_time, now_time;
        int Idx = 0;   // Path index
        int mode = 0;  // 0-start, 1-driving, 2-stop
        int command = 0; // from GCS
        int key_esc = 0;
        int key_steer = 0;

        double targetSpeed = 0;  // [m/s] = [km/h]/3.6
        double targetVel = 0;
        Vector3d x_att;
        MatrixXd traj;
        Matrix<double, 3, 1> x_pos;

        //pure_pursuit();

        void TargetCourse(const std_msgs::msg::Float64MultiArray eq);
        void search_targetCourse_index();

        void state_update(const autorccar_interfaces::msg::NavState::SharedPtr Xstate);

        void controlVehicleSpeed();
        bool checkGoal();
        void calSteeringAngle();

        void PWM_publish(rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_);
        void uart_tx(const int fd, rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_);
        int uart_init(char* tty);
};

#endif

