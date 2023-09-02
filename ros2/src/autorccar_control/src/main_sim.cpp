#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "pure_pursuit.h"

using namespace std::chrono_literals;
pure_pursuit pp;


class simCon : public rclcpp::Node
{
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    size_t count_;

  public:
    simCon()
    : Node("simulation_control"), count_(0) {
      pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("param_ctl", 10);
      timer_ = this->create_wall_timer(20ms, std::bind(&simCon::timer_callback, this)); // HW test >> 100ms

      int N=10;
      double cx[N+1] = {0, };
      double cy[N+1] = {0, };

      for( int i=1; i<N+1; i++) {
        cx[i] = cx[i-1] + 1;
        cy[i] = sin(cx[i]/5)*cx[i]/2.0;
      }

      cx[0] = 0;
      cx[1] = 0.424;
      cx[2] = 0.732;
      cx[3] = 0.855;
      cx[4] = 0.765;
      cx[5] = 0.482;
      cx[6] = 0.070;
      cx[7] = -0.361;
      cx[8] = -0.656;
      cx[9] = -0.618;
      cx[10] = 0;

      cy[0] = 0;
      cy[1] = 0.91;
      cy[2] = 1.88;
      cy[3] = 2.94;
      cy[4] = 4.12;
      cy[5] = 5.37;
      cy[6] = 6.65;
      cy[7] = 7.87;
      cy[8] = 8.94;
      cy[9] = 9.70;
      cy[10] = 10;



      pp.command = 1;
      pp.x_pos << 0, 0, 0;
      pp.x_vel = 0;
      pp.x_yaw = 0;
      pp.traj.resize(N+1,3);
      for(int i=0; i<N+1; i++) {
        pp.traj(i,0) = i;
        pp.traj(i,2) = cx[i];
        pp.traj(i,1) = cy[i];
        //std::cout << i <<", "<< cx[i] <<", " << cy[i] << std::endl;
      }
    }

    int cnt = 0;
    double dt = 0.1;         // [s] time tick
    double WB = 0.3;         // [m] wheel base of vehicle

    void timer_callback() {

      if (cnt > 50) {
        pp.controlVehicleSpeed();  // PID Control
        if (!pp.checkGoal())
          pp.search_targetCourse_index();
        pp.calSteeringAngle();
      }
      else {
        pp.delta = 0;
        pp.ai = 0;
      }

      pp.PWM_publish(pub_);  // to MCU(ESP32)

      pp.x_pos(0) = pp.x_pos(0) + pp.x_vel*sin(pp.x_yaw)*dt;
      pp.x_pos(1) = pp.x_pos(1) + pp.x_vel*cos(pp.x_yaw)*dt;
      pp.x_yaw = pp.x_yaw + pp.x_vel / WB * tan(pp.delta) * dt;
      pp.x_vel = pp.targetSpeed; //pp.ai * dt;

        //std::cout<<cnt<<", "<< pp.Idx<<", "<<pp.x_pos(0)<<", "<<pp.x_pos(1)<<", "<<pp.x_yaw<<", "<<pp.x_vel<<std::endl;
        //std::cout<<cnt<<", "<< pp.Idx<<", "<<pp.x_pos(0)<<", "<<pp.x_pos(1)<<", "<<pp.ai<<", "<<pp.delta*(180/M_PI)<<std::endl;
        //std::cout<<pp.checkGoal() <<", "<<cnt<<", "<< pp.Idx<<", "<<pp.x_pos(0)<<", "<<pp.x_pos(1)<<", "<<pp.x_yaw<<", "<<pp.x_vel<< ", "<<  std::endl;

        //RCLCPP_INFO(this->get_logger(), "%d : simulation end", cnt);
      cnt += 1;
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simCon>());
  rclcpp::shutdown();
  return 0;
}
