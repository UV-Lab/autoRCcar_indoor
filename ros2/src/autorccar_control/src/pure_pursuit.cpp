#include <iostream>
#include <fstream>
#include <math.h>

#include "pure_pursuit.h"
#include "toolbox.h"


void pure_pursuit::TargetCourse(const std_msgs::msg::Float64MultiArray eq) {
    double Ts;
    double X0, X1, X2, X3, X4;
    double Y0, Y1, Y2, Y3, Y4;
    int pathSampling = 10;

    X0 = eq.data[0];
    Y0 = eq.data[1];
    X1 = eq.data[2];
    Y1 = eq.data[3];
    X2 = eq.data[4];
    Y2 = eq.data[5];
    X3 = eq.data[6];
    Y3 = eq.data[7];
    X4 = eq.data[8];
    Y4 = eq.data[9];

    traj.resize(pathSampling+1,3);
    for(int i=0; i<=pathSampling; i++) {
        traj(i,0) = i;
        Ts = double(i)/pathSampling;  // Sampling Time
        traj(i,1) = pow(1-Ts,4)*X0 + 4*pow(1-Ts,3)*Ts*X1 + 6*pow(1-Ts,2)*pow(Ts,2)*X2 + 4*(1-Ts)*pow(Ts,3)*X3 + pow(Ts,4)*X4;
        traj(i,2) = pow(1-Ts,4)*Y0 + 4*pow(1-Ts,3)*Ts*Y1 + 6*pow(1-Ts,2)*pow(Ts,2)*Y2 + 4*(1-Ts)*pow(Ts,3)*Y3 + pow(Ts,4)*Y4;
    }
    std::cout << "Received Target Course\n" << traj << "\n" << std::endl;
}

void pure_pursuit::state_update(const autorccar_interfaces::msg::NavState::SharedPtr Xstate) {
    Eigen::Quaterniond quat;

    x_pos << Xstate->position.x, Xstate->position.y, Xstate->position.z;
    x_vel = sqrt( pow(Xstate->velocity.x,2) + pow(Xstate->velocity.y,2) );

    quat.w() = Xstate->quaternion.w;
    quat.vec() << Xstate->quaternion.x, Xstate->quaternion.y, Xstate->quaternion.z;
    x_att = Quat2Euler(quat);
    x_yaw = x_att(2);

    now_time = Xstate->timestamp.sec + Xstate->timestamp.nanosec*1e-9;
    dt = now_time - prev_time;
}

void pure_pursuit::controlVehicleSpeed() {
    double accel = 0.01/3.6;


    if (command == 0) {
        targetVel -= accel*2;
        if (targetVel <= 0)
            targetVel = 0;
        //mode = 2; // stop
    }
    else {
        targetVel += accel;
        if (targetVel >= limitSpeed)
            targetVel = limitSpeed;
    }

    targetSpeed = targetSpeed + Kp * (targetVel - x_vel);
    if (targetSpeed >= limitSpeed)
      targetSpeed = limitSpeed;
    if (targetSpeed <= 0)
      targetSpeed = 0;
    //std::cout << "\t\t\t\t\t\t\t\tcvs : " <<command<< ", "<< targetSpeed << ", " << targetVel << ", " << limitSpeed <<", " << x_vel << std::endl;
    //targetSpeed = targetSpeed + ai * dt;
}

bool pure_pursuit::checkGoal() {
    bool goal;
    double dist, dx, dy;
    int gIdx = traj.rows()-1;

    dx = x_pos(0) - traj(gIdx,1);  // North
    dy = x_pos(1) - traj(gIdx,2);  // East
    dist = sqrt(pow(dx,2) + pow(dy,2));

    if (dist < GoalDist) {
        command = 0;
        goal = true;
    }
    else
        goal = false;

    //std::cout <<"\t\t\t\t\t\t\t\tcg : " << command <<", "<< dist <<", "<<GoalArea<<", "<< gIdx << std::endl;
    return goal;
}

void pure_pursuit::search_targetCourse_index() {
    int ind;
    double this_dist, next_dist, dx, dy;
    ind = old_tgIdx;
    dx = x_pos(0) - traj(ind,1);  // North
    dy = x_pos(1) - traj(ind,2);  // East
    this_dist = sqrt(pow(dx,2) + pow(dy,2));
    while(1) {
        dx = x_pos(0) - traj(ind+1,1);
        dy = x_pos(1) - traj(ind+1,2);
        next_dist = sqrt(pow(dx,2) + pow(dy,2));
        if (this_dist < next_dist)
            break;
        if ((ind+1) < traj.rows())
            ind += 1;
        this_dist = next_dist;

        if (ind+1 >= traj.rows()) { // add code 10/22
            ind -= 1;
            break;
        }
    }
    old_tgIdx = ind;

    //Lf = Klf * x_vel + lxd; // update look ahead distance
    Lf = lxd;

    dx = x_pos(0) - traj(ind,1);  // North
    dy = x_pos(1) - traj(ind,2);  // East
    this_dist = sqrt(pow(dx,2) + pow(dy,2));
    while (Lf > this_dist) {
        if ((ind+1) >= traj.rows())
            break;
        ind += 1;

        this_dist = sqrt(pow(x_pos(0)-traj(ind,1),2) + pow(x_pos(1)-traj(ind,2),2));
    }
    Idx = ind;
}

void pure_pursuit::calSteeringAngle() {
    double tx, ty;
    double alpha;

    if (Idx < traj.rows()) {
        tx = traj(Idx,1);  // North
        ty = traj(Idx,2);  // East
    }
    else {
    	Idx = traj.rows()-1;
        tx = traj(Idx,1);  // North
        ty = traj(Idx,2);  // East
    }

    alpha = atan2(ty-x_pos(1), tx-x_pos(0)) - x_yaw;  // main.cpp

    delta = atan2(2 * WB * sin(alpha) / Lf, 1.0);
}


int pure_pursuit::uart_init(char* tty) {
    fd = open(tty, O_RDWR | O_NOCTTY);
    struct termios  toptions;

    /* get current serial port settings */
    tcgetattr(fd, &toptions);

    /* set baud both ways */
    cfsetispeed(&toptions, B115200);

    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    /* Canonical mode */
    toptions.c_lflag |= ICANON;

    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);

    if (fd > 0)
        printf("[ %s ] opened as %i\n", tty, fd);
    else
        printf("Error %i from open: %s\n", errno, strerror(errno));
    return fd;
}
void pure_pursuit::uart_tx(const int fd, rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_) {
    geometry_msgs::msg::Vector3 pub_msg;

    double conStr, conVel, newVel;
    char msg[8];
    double sf = -1.5;
    
    double fa = 206.0306;
    double fb = -40.6541;
    double fc = 5138.7189;
    
    if (dt>1)
        dt = 0.1;

    //delta = 45 *(M_PI/180);
    conStr = int(sf* delta * (180/M_PI)) + Steer_PWM_N; // [deg]

    //newVel = x_vel + ai * dt;  // [m/s]
    newVel = targetSpeed;  // [m/s]
    conVel = int((fa*newVel) + (fb*newVel*newVel) + fc);

    if (command == 0) // motor off
        conVel = ESC_PWM_N;

    if (command == 2) {
        conStr = Steer_PWM_N + key_steer;
        conVel = ESC_PWM_N + key_esc;
    }

    if (conStr >= Steer_PWM_MAX)
        conStr = Steer_PWM_MAX;
    if (conStr <= Steer_PWM_MIN)
        conStr = Steer_PWM_MIN;

    if (conVel >= 5350) //ESC_PWM_MAX)
        conVel = 5350; //ESC_PWM_MAX;
    if (conVel <= ESC_PWM_MIN)
        conVel = ESC_PWM_MIN;

    msg[0] = 0xff; // header
    msg[1] = 0xfe; // header
    msg[2] = ((int)conStr >> 8) & 0xff;
    msg[3] = (int)conStr & 0xff;
    msg[4] = ((int)conVel >> 8) & 0xff;
    msg[5] = (int)conVel & 0xff;
    msg[6] = ((int)command >> 8) & 0xff;
    msg[7] = (int)command & 0xff;


    int cnt = write(fd, msg, sizeof(msg));

    pub_msg.x = conStr;    // Servo(Steering) : 0(\)-90(|)-180(/)
    pub_msg.y = conVel;    // ESC : 0-backward-90-forward-180
    pub_msg.z = command;
    pub_->publish(pub_msg);

    //std::cout << Idx << ", " << (int)conStr <<", " << (int)conVel << ", " << (int)command<<", " << x_vel << ", " << x_yaw*(180/M_PI) << std::endl;

}


void pure_pursuit::PWM_publish(rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_) {
    geometry_msgs::msg::Vector3 msg;
    double conStr, conVel, newVel;

    if (dt>1)
        dt = 0.1;

    conStr = int(delta * (180/M_PI)) + 90; // [deg]
    //conStr = 180 - conStr;
    if (conStr >= 180)
        conStr = 180;
    if (conStr <= 0)
        conStr = 0;

    //newVel = x_vel + ai * dt;  // [m/s]
    newVel = targetSpeed;  // [m/s]
    conVel = int((80/limitSpeed)*newVel) + 100;

    if (command == 0) // motor off
        conVel = 100;

    msg.x = conStr;    // Servo(Steering) : 0(\)-90(|)-180(/)
    msg.y = conVel;    // ESC : 0-backward-90-forward-180
    msg.z = mode;
    pub_->publish(msg);

}
