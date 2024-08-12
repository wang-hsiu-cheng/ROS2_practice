/*
 * local_planner.h
 *
 *  Created on: Jul 17, 2024
 *      Author: tu
 */

// ** Fake Host - Local planner
// ** The path will be process here to publish cmd_vel

#ifndef INC_LOCAL_PLANNER_H_
#define INC_LOCAL_PLANNER_H_

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>
using namespace std;

class TurtleController : public rclcpp::Node
{
public:
    TurtleController();
    void move_turtle();

private:
    void initParam();
    int moveTo(double x, double y, double w);
    void initialize_path();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    /* path */
    std::vector<Eigen::Vector3d> path_;
    /* call back */
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);

};

void pointToDist(const float xGoal, const float yGoal, const float wGoal);
void initParam();
Eigen::Vector3d TF_World_to_Robot(float World_x, float World_y, double Robot_theta);
Eigen::Vector3d TF_Robot_to_World(float Robot_x, float Robot_y, double Robot_theta);
// void cmd_vel_pub(float Vx_, float Vy_, float W_);

/* initialize */
float VelX, VelY, AngVelW;
auto pub = geometry_msgs::msg::Twist();


/* p control param */
double maxVelocity;
double maxYawVel;
float deltaTime = 0.01;
const double maxAngularVelocity = 0.1;
const double minAngularVelocity = 0.05;
/* global param */
float botPositionX = 0;
float botPositionY = 0;
float goalDistance;

/* function param*/
float VelocityNow;
float xMoved = 0.0, yMoved = 0.0, wMoved = 0.0, Moved = 0.0;
float remain = goalDistance;
float x_vec, y_vec;
float Goal_w;
float remain_w;

float vel_0 = 0.05;
float vel_1 = maxVelocity - 0.05;
float vel_2 = maxVelocity;
float dist_0 = 0.05;
float dist_1 = maxVelocity / 1.5 + 0.1 / 3 - 0.05;
float dist_2 = maxVelocity / 1.5 + 0.1 / 3;

float tempt_constant;
float yaw_0;
float yaw_1;
float yaw_2;
float angle_0;
float angle_1;
float angle_2;

// cmd_vel
double Vx, Vy, W;
// real velocity
double rVx, rVy, rW;

#endif /* INC_LOCAL_PLANNER_H_ */